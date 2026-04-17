#ifndef OSRM_CUSTOMIZER_TEMPORAL_CELL_CUSTOMIZER_HPP
#define OSRM_CUSTOMIZER_TEMPORAL_CELL_CUSTOMIZER_HPP

#include "customizer/temporal_cell_metric.hpp"
#include "engine/temporal/temporal_profile_decoder.hpp"
#include "partitioner/cell_storage.hpp"
#include "partitioner/multi_level_partition.hpp"
#include "util/log.hpp"
#include "util/query_heap.hpp"

#include <boost/assert.hpp>

#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace osrm::customizer
{

class TemporalCellCustomizer
{
  private:
    using DeduplicatedFunctionKey =
        std::tuple<std::uint32_t,
                   TemporalFunctionDurationValue,
                   TemporalFunctionDurationValue,
                   TemporalFunctionFlags,
                   std::vector<TemporalFunctionBucketValue>,
                   std::vector<TemporalFunctionCoeffValue>>;
    using DeduplicatedFunctionIndex = std::map<DeduplicatedFunctionKey, TemporalFunctionID>;

    struct HeapData
    {
        bool from_clique = false;
    };

    struct PreparedFunctionData
    {
        EdgeDuration minimum_duration{INVALID_EDGE_DURATION};
        EdgeDuration freeflow_duration{INVALID_EDGE_DURATION};
        TemporalFunctionFlags flags{TEMPORAL_FUNCTION_FLAG_NONE};
        std::vector<TemporalFunctionBucketValue> values;
        std::vector<TemporalFunctionCoeffValue> coeffs;
    };

    struct ComputeDiagnostics
    {
        std::uint64_t bucket_iterations = 0;
        std::uint64_t heap_pops = 0;
        std::uint64_t destination_hits = 0;
        std::uint64_t skipped_source_rows = 0;
        std::uint64_t shortcut_row_lookups = 0;
        std::uint64_t shortcut_candidates = 0;
        std::uint64_t shortcut_duration_decodes = 0;
        std::uint64_t shortcut_queue_updates = 0;
        std::uint64_t base_edge_candidates = 0;
        std::uint64_t base_edge_duration_evaluations = 0;
        std::uint64_t base_edge_queue_updates = 0;
        std::uint64_t prepared_destination_profiles = 0;
        std::uint64_t invalid_destination_profiles = 0;

        ComputeDiagnostics &operator+=(const ComputeDiagnostics &rhs)
        {
            bucket_iterations += rhs.bucket_iterations;
            heap_pops += rhs.heap_pops;
            destination_hits += rhs.destination_hits;
            skipped_source_rows += rhs.skipped_source_rows;
            shortcut_row_lookups += rhs.shortcut_row_lookups;
            shortcut_candidates += rhs.shortcut_candidates;
            shortcut_duration_decodes += rhs.shortcut_duration_decodes;
            shortcut_queue_updates += rhs.shortcut_queue_updates;
            base_edge_candidates += rhs.base_edge_candidates;
            base_edge_duration_evaluations += rhs.base_edge_duration_evaluations;
            base_edge_queue_updates += rhs.base_edge_queue_updates;
            prepared_destination_profiles += rhs.prepared_destination_profiles;
            invalid_destination_profiles += rhs.invalid_destination_profiles;
            return *this;
        }
    };

    struct CommitDiagnostics
    {
        std::uint64_t invalid_metric_entries = 0;
        std::uint64_t appended_functions = 0;
        std::uint64_t reused_functions = 0;
        std::uint64_t appended_payload_units = 0;

        CommitDiagnostics &operator+=(const CommitDiagnostics &rhs)
        {
            invalid_metric_entries += rhs.invalid_metric_entries;
            appended_functions += rhs.appended_functions;
            reused_functions += rhs.reused_functions;
            appended_payload_units += rhs.appended_payload_units;
            return *this;
        }
    };

    struct AppendPreparedFunctionResult
    {
        TemporalFunctionID function_id = INVALID_TEMPORAL_FUNCTION_ID;
        bool reused_existing = false;
        std::uint64_t appended_payload_units = 0;
    };

    struct ComputedMetricEntry
    {
        std::size_t metric_index = 0;
        std::optional<PreparedFunctionData> function;
    };

    struct ComputedCellResult
    {
        std::uint64_t processed_source_rows = 0;
        ComputeDiagnostics diagnostics;
        std::vector<ComputedMetricEntry> entries;
    };

    static EdgeDuration SafeAddDuration(const EdgeDuration lhs, const EdgeDuration rhs)
    {
        if (lhs == INVALID_EDGE_DURATION || rhs == INVALID_EDGE_DURATION)
        {
            return INVALID_EDGE_DURATION;
        }

        const auto sum = from_alias<std::int64_t>(lhs) + from_alias<std::int64_t>(rhs);
        const auto clamped = std::min<std::int64_t>(
            sum, std::numeric_limits<EdgeDuration::value_type>::max() - 1LL);
        return EdgeDuration{static_cast<EdgeDuration::value_type>(clamped)};
    }

    static DeduplicatedFunctionKey
    MakeFunctionKey(const std::uint32_t encoding_version,
                    const EdgeDuration minimum_duration,
                    const EdgeDuration freeflow_duration,
                    const TemporalFunctionFlags flags,
                    std::vector<TemporalFunctionBucketValue> values,
                    std::vector<TemporalFunctionCoeffValue> coeffs)
    {
        return {encoding_version,
                from_alias<TemporalFunctionDurationValue>(minimum_duration),
                from_alias<TemporalFunctionDurationValue>(freeflow_duration),
                flags,
                std::move(values),
                std::move(coeffs)};
    }

    static DeduplicatedFunctionIndex
    BuildDeduplicatedFunctionIndex(const TemporalFunctionStorage &storage)
    {
        DeduplicatedFunctionIndex deduplicated_functions;

        for (TemporalFunctionID function_id = 0; function_id < storage.profile_offsets.size();
             ++function_id)
        {
            const auto offset = static_cast<std::size_t>(storage.profile_offsets[function_id]);
            const auto size = static_cast<std::size_t>(storage.profile_sizes[function_id]);

            std::vector<TemporalFunctionBucketValue> values;
            std::vector<TemporalFunctionCoeffValue> coeffs;

            if (storage.meta.encoding_version == TEMPORAL_FUNCTION_ENCODING_VERSION_DENSE)
            {
                values.assign(storage.values.begin() + offset, storage.values.begin() + offset + size);
            }
            else
            {
                coeffs.assign(storage.coeffs.begin() + offset, storage.coeffs.begin() + offset + size);
            }

            deduplicated_functions.emplace(
                MakeFunctionKey(storage.meta.encoding_version,
                                storage.GetMinDuration(function_id),
                                storage.GetFreeFlowDuration(function_id),
                                storage.GetFlags(function_id),
                                std::move(values),
                                std::move(coeffs)),
                function_id);
        }

        return deduplicated_functions;
    }

    template <typename EvaluatorT>
    static void InitializeStorageMeta(TemporalFunctionStorage &storage, const EvaluatorT &evaluator)
    {
        if (storage.meta.week_bucket_count == 0)
        {
            storage.meta.week_bucket_count = evaluator.GetWeekBucketCount();
        }
        else
        {
            BOOST_ASSERT(storage.meta.week_bucket_count == evaluator.GetWeekBucketCount());
        }

        if (storage.meta.bucket_size_minutes == 0)
        {
            storage.meta.bucket_size_minutes = evaluator.GetBucketSizeMinutes();
        }
        else
        {
            BOOST_ASSERT(storage.meta.bucket_size_minutes == evaluator.GetBucketSizeMinutes());
        }

        if (storage.meta.encoding_version == 0)
        {
            storage.meta.encoding_version = TEMPORAL_FUNCTION_ENCODING_VERSION_DENSE;
        }
    }

    static void EnsureFunctionMetadataAlignment(TemporalFunctionStorage &storage)
    {
        const auto function_count = storage.profile_offsets.size();
        storage.min_durations.resize(function_count,
                                     from_alias<TemporalFunctionDurationValue>(
                                         INVALID_EDGE_DURATION));
        storage.freeflow_durations.resize(function_count,
                                          from_alias<TemporalFunctionDurationValue>(
                                              INVALID_EDGE_DURATION));
        storage.profile_flags.resize(function_count, TEMPORAL_FUNCTION_FLAG_NONE);
    }

    PreparedFunctionData
    PrepareFunction(const TemporalFunctionMeta &meta,
                    std::vector<TemporalFunctionBucketValue> profile) const
    {
        BOOST_ASSERT(!profile.empty());
        BOOST_ASSERT(meta.week_bucket_count == profile.size());

        PreparedFunctionData prepared;
        prepared.minimum_duration =
            EdgeDuration{*std::min_element(profile.begin(), profile.end())};
        prepared.freeflow_duration = prepared.minimum_duration;

        if (engine::temporal::IsTemporalFunctionFIFO(profile, meta.bucket_size_minutes))
        {
            prepared.flags |= TEMPORAL_FUNCTION_FLAG_FIFO_VALID;
        }

        if (meta.encoding_version == TEMPORAL_FUNCTION_ENCODING_VERSION_DCT)
        {
            const auto coeff_target =
                preferred_coeff_count == 0
                    ? static_cast<std::uint32_t>(profile.size())
                    : std::min<std::uint32_t>(preferred_coeff_count,
                                              static_cast<std::uint32_t>(profile.size()));
            const auto compression = engine::temporal::CompressTemporalFunctionAdaptiveWithStatus(
                profile, coeff_target, max_mean_abs_error, max_bucket_abs_error);
            if (compression.used_fallback_coeff_count)
            {
                prepared.flags |= TEMPORAL_FUNCTION_FLAG_USED_FALLBACK_COEFF_COUNT;
            }
            prepared.coeffs = std::move(compression.coefficients);
        }
        else
        {
            prepared.values = std::move(profile);
        }

        return prepared;
    }

    AppendPreparedFunctionResult
    AppendPreparedFunction(TemporalFunctionStorage &storage,
                           DeduplicatedFunctionIndex &deduplicated_functions,
                           PreparedFunctionData prepared) const
    {
        auto key = MakeFunctionKey(storage.meta.encoding_version,
                                   prepared.minimum_duration,
                                   prepared.freeflow_duration,
                                   prepared.flags,
                                   std::move(prepared.values),
                                   std::move(prepared.coeffs));
        const auto existing = deduplicated_functions.find(key);
        if (existing != deduplicated_functions.end())
        {
            return {existing->second, true, 0};
        }

        const auto function_id = static_cast<TemporalFunctionID>(storage.profile_offsets.size());
        EnsureFunctionMetadataAlignment(storage);
        storage.min_durations.push_back(std::get<1>(key));
        storage.freeflow_durations.push_back(std::get<2>(key));
        storage.profile_flags.push_back(std::get<3>(key));
        std::uint64_t appended_payload_units = 0;

        if (storage.meta.encoding_version == TEMPORAL_FUNCTION_ENCODING_VERSION_DCT)
        {
            const auto &coeffs = std::get<5>(key);
            storage.profile_offsets.push_back(storage.coeffs.size());
            storage.profile_sizes.push_back(static_cast<std::uint32_t>(coeffs.size()));
            storage.coeffs.insert(storage.coeffs.end(), coeffs.begin(), coeffs.end());
            appended_payload_units = coeffs.size();
        }
        else
        {
            const auto &values = std::get<4>(key);
            storage.profile_offsets.push_back(storage.values.size());
            storage.profile_sizes.push_back(static_cast<std::uint32_t>(values.size()));
            storage.values.insert(storage.values.end(), values.begin(), values.end());
            appended_payload_units = values.size();
        }

        deduplicated_functions.emplace(std::move(key), function_id);
        return {function_id, false, appended_payload_units};
    }

    TemporalFunctionID AppendFunction(TemporalFunctionStorage &storage,
                                      DeduplicatedFunctionIndex &deduplicated_functions,
                                      const std::vector<TemporalFunctionBucketValue> &profile) const
    {
        return AppendPreparedFunction(
                   storage,
                   deduplicated_functions,
                   PrepareFunction(storage.meta,
                                   std::vector<TemporalFunctionBucketValue>(profile.begin(),
                                                                            profile.end())))
            .function_id;
    }

    template <typename GraphT, typename EvaluatorT>
    void RelaxNode(const GraphT &graph,
                   const partitioner::CellStorage &cells,
                   const std::vector<bool> &allowed_nodes,
                   const EvaluatorT &evaluator,
                   const TemporalCellMetric &metric,
                   const TemporalFunctionStorage &storage,
                   ComputeDiagnostics &diagnostics,
                   util::QueryHeap<NodeID,
                                   NodeID,
                                   EdgeDuration,
                                   HeapData,
                                   util::ArrayStorage<NodeID, int>> &heap,
                   const LevelID level,
                   const NodeID node,
                   const std::uint32_t departure_bucket,
                   const EdgeDuration current_duration) const
    {
        const bool first_level = level == 1;

        if (!first_level && !heap.GetData(node).from_clique)
        {
            ++diagnostics.shortcut_row_lookups;
            const auto subcell_id = partition.GetCell(level - 1, node);
            const auto subcell = metric.GetCell(cells, level - 1, subcell_id);
            auto subcell_destination = subcell.GetDestinationNodes().begin();
            auto subcell_min_duration = subcell.GetOutMinDuration(node).begin();
            const auto evaluation_bucket =
                evaluator.GetBucketAfterDuration(departure_bucket, current_duration);

            for (const auto subcell_function_id : subcell.GetOutFunctionID(node))
            {
                ++diagnostics.shortcut_candidates;
                const auto to = *subcell_destination;
                const auto lower_bound = *subcell_min_duration;

                if (allowed_nodes[to] &&
                    subcell_function_id != INVALID_TEMPORAL_FUNCTION_ID &&
                    lower_bound != INVALID_EDGE_DURATION)
                {
                    ++diagnostics.shortcut_duration_decodes;
                    const auto shortcut_duration =
                        storage.GetDuration(subcell_function_id, evaluation_bucket);
                    const auto to_duration = SafeAddDuration(current_duration, shortcut_duration);
                    if (to_duration != INVALID_EDGE_DURATION)
                    {
                        if (!heap.WasInserted(to))
                        {
                            heap.Insert(to, to_duration, {true});
                            ++diagnostics.shortcut_queue_updates;
                        }
                        else if (to_duration < heap.GetKey(to))
                        {
                            heap.DecreaseKey(to, to_duration);
                            heap.GetData(to) = {true};
                            ++diagnostics.shortcut_queue_updates;
                        }
                    }
                }

                ++subcell_destination;
                ++subcell_min_duration;
            }
        }

        const auto evaluation_bucket = evaluator.GetBucketAfterDuration(departure_bucket,
                                                                        current_duration);
        for (const auto edge : graph.GetInternalEdgeRange(level, node))
        {
            ++diagnostics.base_edge_candidates;
            const auto to = graph.GetTarget(edge);
            if (!allowed_nodes[to])
            {
                continue;
            }

            const auto &data = graph.GetEdgeData(edge);
            if (!data.forward ||
                (!first_level &&
                 partition.GetCell(level - 1, node) == partition.GetCell(level - 1, to)))
            {
                continue;
            }

            ++diagnostics.base_edge_duration_evaluations;
            const auto arc_duration =
                evaluator.GetBaseEdgeDuration(graph, node, edge, evaluation_bucket);
            const auto to_duration = SafeAddDuration(current_duration, arc_duration);
            if (to_duration == INVALID_EDGE_DURATION)
            {
                continue;
            }

            if (!heap.WasInserted(to))
            {
                heap.Insert(to, to_duration, {false});
                ++diagnostics.base_edge_queue_updates;
            }
            else if (to_duration < heap.GetKey(to))
            {
                heap.DecreaseKey(to, to_duration);
                heap.GetData(to) = {false};
                ++diagnostics.base_edge_queue_updates;
            }
        }
    }

  public:
    using Heap =
        util::QueryHeap<NodeID, NodeID, EdgeDuration, HeapData, util::ArrayStorage<NodeID, int>>;

  private:
    struct WorkerState
    {
        explicit WorkerState(const NodeID number_of_nodes) : heap(number_of_nodes) {}

        Heap heap;
    };

    static constexpr std::size_t PARALLEL_CELL_CHUNK_SIZE = 32;

  public:

    explicit TemporalCellCustomizer(const partitioner::MultiLevelPartition &partition_,
                                    const std::uint32_t preferred_coeff_count_ = 0,
                                    const float max_mean_abs_error_ = 1.0F,
                                    const float max_bucket_abs_error_ = 2.0F)
        : partition(partition_), preferred_coeff_count(preferred_coeff_count_),
          max_mean_abs_error(max_mean_abs_error_),
          max_bucket_abs_error(max_bucket_abs_error_)
    {
    }

    template <typename GraphT, typename EvaluatorT>
    void Customize(const GraphT &graph,
                   Heap &heap,
                   const partitioner::CellStorage &cells,
                   const std::vector<bool> &allowed_nodes,
                   const EvaluatorT &evaluator,
                   TemporalCellMetric &metric,
                   TemporalFunctionStorage &storage,
                   const LevelID level,
                   const CellID id) const
    {
        InitializeStorageMeta(storage, evaluator);
        auto deduplicated_functions = BuildDeduplicatedFunctionIndex(storage);
        auto computed =
            ComputeCell(graph, heap, cells, allowed_nodes, evaluator, metric, storage, level, id);
        CommitCellResult(metric, storage, deduplicated_functions, std::move(computed));
    }

    template <typename GraphT, typename EvaluatorT>
    void CustomizeCell(const GraphT &graph,
                       Heap &heap,
                       const partitioner::CellStorage &cells,
                       const std::vector<bool> &allowed_nodes,
                       const EvaluatorT &evaluator,
                       TemporalCellMetric &metric,
                       TemporalFunctionStorage &storage,
                       DeduplicatedFunctionIndex &deduplicated_functions,
                       const LevelID level,
                       const CellID id) const
    {
        InitializeStorageMeta(storage, evaluator);
        auto computed =
            ComputeCell(graph, heap, cells, allowed_nodes, evaluator, metric, storage, level, id);
        CommitCellResult(metric, storage, deduplicated_functions, std::move(computed));
    }

    template <typename GraphT, typename EvaluatorT>
    ComputedCellResult ComputeCell(const GraphT &graph,
                                   Heap &heap,
                                   const partitioner::CellStorage &cells,
                                   const std::vector<bool> &allowed_nodes,
                                   const EvaluatorT &evaluator,
                                   const TemporalCellMetric &metric,
                                   const TemporalFunctionStorage &storage,
                                   const LevelID level,
                                   const CellID id) const
    {
        ComputedCellResult result;
        const auto cell = cells.GetUnfilledCell(level, id);
        const auto &cell_data = cells.GetCellData(level, id);
        const auto destinations = cell.GetDestinationNodes();
        const auto sources = cell.GetSourceNodes();
        std::vector<NodeID> destination_nodes(destinations.begin(), destinations.end());
        const auto destination_count = destination_nodes.size();
        std::unordered_map<NodeID, std::size_t> destination_lookup;
        destination_lookup.reserve(destination_nodes.size());
        for (std::size_t destination_index = 0; destination_index < destination_nodes.size();
             ++destination_index)
        {
            destination_lookup.emplace(destination_nodes[destination_index], destination_index);
        }

        const auto bucket_count = storage.meta.week_bucket_count;
        result.processed_source_rows =
            static_cast<std::uint64_t>(std::distance(sources.begin(), sources.end()));
        result.entries.reserve(static_cast<std::size_t>(result.processed_source_rows) *
                               destination_count);

        std::size_t source_index = 0;
        for (const auto source : sources)
        {
            const auto row_offset = static_cast<std::size_t>(cell_data.value_offset) +
                                    source_index * destination_count;
            ++source_index;

            if (!allowed_nodes[source])
            {
                ++result.diagnostics.skipped_source_rows;
                for (std::size_t destination_index = 0; destination_index < destination_nodes.size();
                     ++destination_index)
                {
                    result.entries.push_back({row_offset + destination_index, std::nullopt});
                }
                continue;
            }

            std::vector<std::vector<TemporalFunctionBucketValue>> dense_functions(
                destination_nodes.size(),
                std::vector<TemporalFunctionBucketValue>(bucket_count,
                                                         from_alias<TemporalFunctionBucketValue>(
                                                             INVALID_EDGE_DURATION)));

            for (std::uint32_t departure_bucket = 0; departure_bucket < bucket_count;
                 ++departure_bucket)
            {
                ++result.diagnostics.bucket_iterations;
                heap.Clear();
                heap.Insert(source, EdgeDuration{0}, {false});
                auto remaining = destination_lookup;

                while (!heap.Empty() && !remaining.empty())
                {
                    ++result.diagnostics.heap_pops;
                    const auto node = heap.DeleteMin();
                    const auto current_duration = heap.GetKey(node);

                    const auto found = remaining.find(node);
                    if (found != remaining.end())
                    {
                        ++result.diagnostics.destination_hits;
                        dense_functions[found->second][departure_bucket] =
                            from_alias<TemporalFunctionBucketValue>(current_duration);
                        remaining.erase(found);
                    }

                    RelaxNode(graph,
                              cells,
                              allowed_nodes,
                              evaluator,
                              metric,
                              storage,
                              result.diagnostics,
                              heap,
                              level,
                              node,
                              departure_bucket,
                              current_duration);
                }
            }

            for (std::size_t destination_index = 0; destination_index < destination_nodes.size();
                 ++destination_index)
            {
                auto &profile = dense_functions[destination_index];
                const auto has_invalid_bucket =
                    std::any_of(profile.begin(), profile.end(), [](const auto duration) {
                        return duration ==
                               from_alias<TemporalFunctionBucketValue>(INVALID_EDGE_DURATION);
                    });

                const auto metric_index = row_offset + destination_index;
                if (has_invalid_bucket)
                {
                    ++result.diagnostics.invalid_destination_profiles;
                    result.entries.push_back({metric_index, std::nullopt});
                    continue;
                }

                ++result.diagnostics.prepared_destination_profiles;
                result.entries.push_back(
                    {metric_index, PrepareFunction(storage.meta, std::move(profile))});
            }
        }

        return result;
    }

    CommitDiagnostics CommitCellResult(TemporalCellMetric &metric,
                                       TemporalFunctionStorage &storage,
                                       DeduplicatedFunctionIndex &deduplicated_functions,
                                       ComputedCellResult result) const
    {
        CommitDiagnostics diagnostics;
        for (auto &entry : result.entries)
        {
            if (!entry.function)
            {
                ++diagnostics.invalid_metric_entries;
                metric.function_ids[entry.metric_index] = INVALID_TEMPORAL_FUNCTION_ID;
                metric.min_durations[entry.metric_index] = INVALID_EDGE_DURATION;
                continue;
            }

            const auto append_result =
                AppendPreparedFunction(storage, deduplicated_functions, std::move(*entry.function));
            if (append_result.reused_existing)
            {
                ++diagnostics.reused_functions;
            }
            else
            {
                ++diagnostics.appended_functions;
                diagnostics.appended_payload_units += append_result.appended_payload_units;
            }

            metric.function_ids[entry.metric_index] = append_result.function_id;
            metric.min_durations[entry.metric_index] =
                storage.GetMinDuration(append_result.function_id);
        }

        return diagnostics;
    }

    template <typename GraphT, typename EvaluatorT>
    void Customize(const GraphT &graph,
                   const partitioner::CellStorage &cells,
                   const std::vector<bool> &allowed_nodes,
                   const EvaluatorT &evaluator,
                   TemporalCellMetric &metric,
                   TemporalFunctionStorage &storage) const
    {
        const auto max_level = partition.GetNumberOfLevels() > 0
                                   ? static_cast<LevelID>(partition.GetNumberOfLevels() - 1)
                                   : LevelID{0};
        Customize(
            graph, cells, allowed_nodes, evaluator, metric, storage, LevelID{1}, max_level);
    }

    template <typename GraphT, typename EvaluatorT>
    void CustomizeSerial(const GraphT &graph,
                         const partitioner::CellStorage &cells,
                         const std::vector<bool> &allowed_nodes,
                         const EvaluatorT &evaluator,
                         TemporalCellMetric &metric,
                         TemporalFunctionStorage &storage) const
    {
        const auto max_level = partition.GetNumberOfLevels() > 0
                                   ? static_cast<LevelID>(partition.GetNumberOfLevels() - 1)
                                   : LevelID{0};
        CustomizeSerial(
            graph, cells, allowed_nodes, evaluator, metric, storage, LevelID{1}, max_level);
    }

    template <typename GraphT, typename EvaluatorT>
    void Customize(const GraphT &graph,
                   const partitioner::CellStorage &cells,
                   const std::vector<bool> &allowed_nodes,
                   const EvaluatorT &evaluator,
                   TemporalCellMetric &metric,
                   TemporalFunctionStorage &storage,
                   const LevelID first_level,
                   const LevelID last_level) const
    {
        InitializeStorageMeta(storage, evaluator);

        if (partition.GetNumberOfLevels() <= 1)
        {
            return;
        }

        const auto begin_level =
            std::max<std::size_t>(1UL, static_cast<std::size_t>(first_level));
        const auto end_level =
            std::min<std::size_t>(partition.GetNumberOfLevels() - 1,
                                  static_cast<std::size_t>(last_level));

        if (begin_level > end_level)
        {
            return;
        }

        using WorkerStates = tbb::enumerable_thread_specific<WorkerState>;
        WorkerStates workers([&graph]() { return WorkerState{graph.GetNumberOfNodes()}; });
        auto deduplicated_functions = BuildDeduplicatedFunctionIndex(storage);

        for (std::size_t level = begin_level; level <= end_level; ++level)
        {
            const auto level_cell_count =
                static_cast<std::size_t>(partition.GetNumberOfCells(static_cast<LevelID>(level)));
            const auto level_start = std::chrono::steady_clock::now();
            const auto log_interval = std::max<std::size_t>(1, level_cell_count / 20);
            std::uint64_t processed_source_rows = 0;
            std::uint64_t compute_time_ms_total = 0;
            std::uint64_t commit_time_ms_total = 0;
            ComputeDiagnostics level_compute_diagnostics;
            CommitDiagnostics level_commit_diagnostics;

            util::Log() << "Temporal overlay customization: starting level " << level
                        << " with " << level_cell_count << " cell(s), bucket_count="
                        << storage.meta.week_bucket_count << ", function_count="
                        << storage.profile_offsets.size();

            for (std::size_t chunk_begin = 0; chunk_begin < level_cell_count;
                 chunk_begin += PARALLEL_CELL_CHUNK_SIZE)
            {
                const auto chunk_end = std::min<std::size_t>(
                    level_cell_count, chunk_begin + PARALLEL_CELL_CHUNK_SIZE);
                std::vector<ComputedCellResult> computed_cells(chunk_end - chunk_begin);
                const auto chunk_compute_start = std::chrono::steady_clock::now();

                tbb::parallel_for(
                    tbb::blocked_range<std::size_t>(chunk_begin, chunk_end),
                    [&](const tbb::blocked_range<std::size_t> &range)
                    {
                        auto &worker = workers.local();
                        for (auto id = range.begin(); id != range.end(); ++id)
                        {
                            computed_cells[id - chunk_begin] =
                                ComputeCell(graph,
                                            worker.heap,
                                            cells,
                                            allowed_nodes,
                                            evaluator,
                                            metric,
                                            storage,
                                            static_cast<LevelID>(level),
                                             static_cast<CellID>(id));
                        }
                    });
                const auto chunk_compute_ms = static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - chunk_compute_start)
                        .count());

                std::uint64_t chunk_processed_source_rows = 0;
                ComputeDiagnostics chunk_compute_diagnostics;
                CommitDiagnostics chunk_commit_diagnostics;
                const auto chunk_commit_start = std::chrono::steady_clock::now();
                for (auto &computed : computed_cells)
                {
                    chunk_processed_source_rows += computed.processed_source_rows;
                    chunk_compute_diagnostics += computed.diagnostics;
                    chunk_commit_diagnostics +=
                        CommitCellResult(metric, storage, deduplicated_functions, std::move(computed));
                }
                const auto chunk_commit_ms = static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - chunk_commit_start)
                        .count());

                processed_source_rows += chunk_processed_source_rows;
                compute_time_ms_total += chunk_compute_ms;
                commit_time_ms_total += chunk_commit_ms;
                level_compute_diagnostics += chunk_compute_diagnostics;
                level_commit_diagnostics += chunk_commit_diagnostics;
                const auto completed_cells = chunk_end;
                const auto crossed_interval =
                    chunk_begin / log_interval != (completed_cells - 1) / log_interval;
                if (completed_cells == level_cell_count || crossed_interval)
                {
                    const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                                             std::chrono::steady_clock::now() - level_start)
                                             .count();
                    util::Log() << "Temporal overlay customization: level " << level
                                << " progress " << completed_cells << "/" << level_cell_count
                                << " cell(s), processed_source_rows=" << processed_source_rows
                                << ", function_count=" << storage.profile_offsets.size()
                                << ", elapsed=" << elapsed << "s"
                                << ", compute_ms=" << compute_time_ms_total
                                << ", commit_ms=" << commit_time_ms_total
                                << ", heap_pops=" << level_compute_diagnostics.heap_pops
                                << ", shortcut_decodes="
                                << level_compute_diagnostics.shortcut_duration_decodes
                                << ", base_edge_evals="
                                << level_compute_diagnostics.base_edge_duration_evaluations
                                << ", new_functions=" << level_commit_diagnostics.appended_functions
                                << ", reused_functions="
                                << level_commit_diagnostics.reused_functions;
                }
            }

            const auto level_elapsed_ms = static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - level_start)
                    .count());
            const auto compute_ms_per_source_row =
                processed_source_rows == 0
                    ? 0.0
                    : static_cast<double>(compute_time_ms_total) /
                          static_cast<double>(processed_source_rows);
            const auto commit_ms_per_source_row =
                processed_source_rows == 0
                    ? 0.0
                    : static_cast<double>(commit_time_ms_total) /
                          static_cast<double>(processed_source_rows);

            util::Log() << "Temporal overlay customization: level " << level
                        << " diagnostics elapsed_ms=" << level_elapsed_ms
                        << ", compute_ms=" << compute_time_ms_total
                        << ", commit_ms=" << commit_time_ms_total
                        << ", compute_ms_per_source_row=" << compute_ms_per_source_row
                        << ", commit_ms_per_source_row=" << commit_ms_per_source_row
                        << ", bucket_iterations=" << level_compute_diagnostics.bucket_iterations
                        << ", heap_pops=" << level_compute_diagnostics.heap_pops
                        << ", destination_hits=" << level_compute_diagnostics.destination_hits
                        << ", skipped_source_rows=" << level_compute_diagnostics.skipped_source_rows
                        << ", shortcut_row_lookups="
                        << level_compute_diagnostics.shortcut_row_lookups
                        << ", shortcut_candidates="
                        << level_compute_diagnostics.shortcut_candidates
                        << ", shortcut_decodes="
                        << level_compute_diagnostics.shortcut_duration_decodes
                        << ", shortcut_queue_updates="
                        << level_compute_diagnostics.shortcut_queue_updates
                        << ", base_edge_candidates="
                        << level_compute_diagnostics.base_edge_candidates
                        << ", base_edge_evals="
                        << level_compute_diagnostics.base_edge_duration_evaluations
                        << ", base_edge_queue_updates="
                        << level_compute_diagnostics.base_edge_queue_updates
                        << ", prepared_profiles="
                        << level_compute_diagnostics.prepared_destination_profiles
                        << ", invalid_profiles="
                        << level_compute_diagnostics.invalid_destination_profiles
                        << ", invalid_metric_entries="
                        << level_commit_diagnostics.invalid_metric_entries
                        << ", new_functions=" << level_commit_diagnostics.appended_functions
                        << ", reused_functions=" << level_commit_diagnostics.reused_functions
                        << ", appended_payload_units="
                        << level_commit_diagnostics.appended_payload_units;
        }
    }

    template <typename GraphT, typename EvaluatorT>
    void CustomizeSerial(const GraphT &graph,
                         const partitioner::CellStorage &cells,
                         const std::vector<bool> &allowed_nodes,
                         const EvaluatorT &evaluator,
                         TemporalCellMetric &metric,
                         TemporalFunctionStorage &storage,
                         const LevelID first_level,
                         const LevelID last_level) const
    {
        InitializeStorageMeta(storage, evaluator);

        if (partition.GetNumberOfLevels() <= 1)
        {
            return;
        }

        const auto begin_level =
            std::max<std::size_t>(1UL, static_cast<std::size_t>(first_level));
        const auto end_level =
            std::min<std::size_t>(partition.GetNumberOfLevels() - 1,
                                  static_cast<std::size_t>(last_level));

        if (begin_level > end_level)
        {
            return;
        }

        Heap heap(graph.GetNumberOfNodes());
        auto deduplicated_functions = BuildDeduplicatedFunctionIndex(storage);

        for (std::size_t level = begin_level; level <= end_level; ++level)
        {
            const auto level_cell_count =
                static_cast<std::size_t>(partition.GetNumberOfCells(static_cast<LevelID>(level)));
            for (std::size_t id = 0; id < level_cell_count; ++id)
            {
                auto computed = ComputeCell(graph,
                                            heap,
                                            cells,
                                            allowed_nodes,
                                            evaluator,
                                            metric,
                                            storage,
                                            static_cast<LevelID>(level),
                                            static_cast<CellID>(id));
                CommitCellResult(metric, storage, deduplicated_functions, std::move(computed));
            }
        }
    }

  private:
    const partitioner::MultiLevelPartition &partition;
    std::uint32_t preferred_coeff_count;
    float max_mean_abs_error;
    float max_bucket_abs_error;
};
} // namespace osrm::customizer

#endif
