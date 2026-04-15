#ifndef OSRM_CUSTOMIZER_TEMPORAL_CELL_CUSTOMIZER_HPP
#define OSRM_CUSTOMIZER_TEMPORAL_CELL_CUSTOMIZER_HPP

#include "customizer/temporal_cell_metric.hpp"
#include "engine/temporal/temporal_profile_decoder.hpp"
#include "partitioner/cell_storage.hpp"
#include "partitioner/multi_level_partition.hpp"
#include "util/query_heap.hpp"

#include <boost/assert.hpp>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <map>
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

    static DeduplicatedFunctionKey MakeFunctionKey(
        const TemporalFunctionStorage &storage,
        const EdgeDuration minimum_duration,
        const EdgeDuration freeflow_duration,
        const TemporalFunctionFlags flags,
        std::vector<TemporalFunctionBucketValue> values,
        std::vector<TemporalFunctionCoeffValue> coeffs)
    {
        return {storage.meta.encoding_version,
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
                MakeFunctionKey(storage,
                                storage.GetMinDuration(function_id),
                                storage.GetFreeFlowDuration(function_id),
                                storage.GetFlags(function_id),
                                std::move(values),
                                std::move(coeffs)),
                function_id);
        }

        return deduplicated_functions;
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

    TemporalFunctionID AppendFunction(TemporalFunctionStorage &storage,
                                      DeduplicatedFunctionIndex &deduplicated_functions,
                                      const std::vector<TemporalFunctionBucketValue> &profile) const
    {
        BOOST_ASSERT(!profile.empty());
        BOOST_ASSERT(storage.meta.week_bucket_count == profile.size());

        const auto minimum_duration = EdgeDuration{*std::min_element(profile.begin(), profile.end())};
        const auto freeflow_duration = minimum_duration;
        auto flags = TEMPORAL_FUNCTION_FLAG_NONE;
        if (engine::temporal::IsTemporalFunctionFIFO(profile, storage.meta.bucket_size_minutes))
        {
            flags |= TEMPORAL_FUNCTION_FLAG_FIFO_VALID;
        }

        if (storage.meta.encoding_version == TEMPORAL_FUNCTION_ENCODING_VERSION_DCT)
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
                flags |= TEMPORAL_FUNCTION_FLAG_USED_FALLBACK_COEFF_COUNT;
            }

            auto key = MakeFunctionKey(storage,
                                       minimum_duration,
                                       freeflow_duration,
                                       flags,
                                       {},
                                       compression.coefficients);
            const auto existing = deduplicated_functions.find(key);
            if (existing != deduplicated_functions.end())
            {
                return existing->second;
            }

            const auto function_id = static_cast<TemporalFunctionID>(storage.profile_offsets.size());
            EnsureFunctionMetadataAlignment(storage);
            storage.min_durations.push_back(from_alias<TemporalFunctionDurationValue>(minimum_duration));
            storage.freeflow_durations.push_back(
                from_alias<TemporalFunctionDurationValue>(freeflow_duration));
            storage.profile_flags.push_back(flags);

            storage.profile_offsets.push_back(storage.coeffs.size());
            storage.profile_sizes.push_back(
                static_cast<std::uint32_t>(compression.coefficients.size()));
            storage.coeffs.insert(storage.coeffs.end(),
                                  compression.coefficients.begin(),
                                  compression.coefficients.end());
            deduplicated_functions.emplace(std::move(key), function_id);
            return function_id;
        }

        auto key = MakeFunctionKey(
            storage, minimum_duration, freeflow_duration, flags, {profile.begin(), profile.end()}, {});
        const auto existing = deduplicated_functions.find(key);
        if (existing != deduplicated_functions.end())
        {
            return existing->second;
        }

        const auto function_id = static_cast<TemporalFunctionID>(storage.profile_offsets.size());
        EnsureFunctionMetadataAlignment(storage);
        storage.min_durations.push_back(from_alias<TemporalFunctionDurationValue>(minimum_duration));
        storage.freeflow_durations.push_back(from_alias<TemporalFunctionDurationValue>(freeflow_duration));
        storage.profile_flags.push_back(flags);
        storage.profile_offsets.push_back(storage.values.size());
        storage.profile_sizes.push_back(static_cast<std::uint32_t>(profile.size()));
        storage.values.insert(storage.values.end(), profile.begin(), profile.end());
        deduplicated_functions.emplace(std::move(key), function_id);
        return function_id;
    }

    template <typename GraphT, typename EvaluatorT>
    void RelaxNode(const GraphT &graph,
                   const partitioner::CellStorage &cells,
                   const std::vector<bool> &allowed_nodes,
                   const EvaluatorT &evaluator,
                   const TemporalCellMetric &metric,
                   const TemporalFunctionStorage &storage,
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
            const auto subcell_id = partition.GetCell(level - 1, node);
            const auto subcell = metric.GetCell(cells, level - 1, subcell_id);
            auto subcell_destination = subcell.GetDestinationNodes().begin();
            auto subcell_min_duration = subcell.GetOutMinDuration(node).begin();
            const auto evaluation_bucket =
                evaluator.GetBucketAfterDuration(departure_bucket, current_duration);

            for (const auto subcell_function_id : subcell.GetOutFunctionID(node))
            {
                const auto to = *subcell_destination;
                const auto lower_bound = *subcell_min_duration;

                if (allowed_nodes[to] &&
                    subcell_function_id != INVALID_TEMPORAL_FUNCTION_ID &&
                    lower_bound != INVALID_EDGE_DURATION)
                {
                    const auto shortcut_duration =
                        storage.GetDuration(subcell_function_id, evaluation_bucket);
                    const auto to_duration = SafeAddDuration(current_duration, shortcut_duration);
                    if (to_duration != INVALID_EDGE_DURATION)
                    {
                        if (!heap.WasInserted(to))
                        {
                            heap.Insert(to, to_duration, {true});
                        }
                        else if (to_duration < heap.GetKey(to))
                        {
                            heap.DecreaseKey(to, to_duration);
                            heap.GetData(to) = {true};
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
            }
            else if (to_duration < heap.GetKey(to))
            {
                heap.DecreaseKey(to, to_duration);
                heap.GetData(to) = {false};
            }
        }
    }

  public:
    using Heap =
        util::QueryHeap<NodeID, NodeID, EdgeDuration, HeapData, util::ArrayStorage<NodeID, int>>;

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
        auto deduplicated_functions = BuildDeduplicatedFunctionIndex(storage);
        CustomizeCell(graph,
                      heap,
                      cells,
                      allowed_nodes,
                      evaluator,
                      metric,
                      storage,
                      deduplicated_functions,
                      level,
                      id);
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

        const auto cell = cells.GetUnfilledCell(level, id);
        const auto &cell_data = cells.GetCellData(level, id);
        const auto destinations = cell.GetDestinationNodes();
        std::vector<NodeID> destination_nodes(destinations.begin(), destinations.end());
        std::unordered_map<NodeID, std::size_t> destination_lookup;
        destination_lookup.reserve(destination_nodes.size());
        for (std::size_t destination_index = 0; destination_index < destination_nodes.size();
             ++destination_index)
        {
            destination_lookup.emplace(destination_nodes[destination_index], destination_index);
        }

        const auto bucket_count = storage.meta.week_bucket_count;
        for (const auto source : cell.GetSourceNodes())
        {
            const auto row_offset = static_cast<std::size_t>(cell_data.value_offset) +
                                    static_cast<std::size_t>(
                                        std::distance(cell.GetSourceNodes().begin(),
                                                      std::find(cell.GetSourceNodes().begin(),
                                                                cell.GetSourceNodes().end(),
                                                                source))) *
                                        destination_nodes.size();

            if (!allowed_nodes[source])
            {
                for (std::size_t destination_index = 0; destination_index < destination_nodes.size();
                     ++destination_index)
                {
                    metric.function_ids[row_offset + destination_index] =
                        INVALID_TEMPORAL_FUNCTION_ID;
                    metric.min_durations[row_offset + destination_index] = INVALID_EDGE_DURATION;
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
                heap.Clear();
                heap.Insert(source, EdgeDuration{0}, {false});
                auto remaining = destination_lookup;

                while (!heap.Empty() && !remaining.empty())
                {
                    const auto node = heap.DeleteMin();
                    const auto current_duration = heap.GetKey(node);

                    const auto found = remaining.find(node);
                    if (found != remaining.end())
                    {
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
                    metric.function_ids[metric_index] = INVALID_TEMPORAL_FUNCTION_ID;
                    metric.min_durations[metric_index] = INVALID_EDGE_DURATION;
                    continue;
                }

                const auto function_id =
                    AppendFunction(storage, deduplicated_functions, profile);
                metric.function_ids[metric_index] = function_id;
                metric.min_durations[metric_index] = storage.GetMinDuration(function_id);
            }
        }
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
    void Customize(const GraphT &graph,
                   const partitioner::CellStorage &cells,
                   const std::vector<bool> &allowed_nodes,
                   const EvaluatorT &evaluator,
                   TemporalCellMetric &metric,
                   TemporalFunctionStorage &storage,
                   const LevelID first_level,
                   const LevelID last_level) const
    {
        Heap heap(graph.GetNumberOfNodes());
        auto deduplicated_functions = BuildDeduplicatedFunctionIndex(storage);

        const auto begin_level =
            std::max<std::size_t>(1UL, static_cast<std::size_t>(first_level));
        const auto end_level =
            std::min<std::size_t>(partition.GetNumberOfLevels() - 1,
                                  static_cast<std::size_t>(last_level));

        if (partition.GetNumberOfLevels() <= 1 || begin_level > end_level)
        {
            return;
        }

        for (std::size_t level = begin_level; level <= end_level; ++level)
        {
            for (std::size_t id = 0; id < partition.GetNumberOfCells(level); ++id)
            {
                CustomizeCell(graph,
                              heap,
                              cells,
                              allowed_nodes,
                              evaluator,
                              metric,
                              storage,
                              deduplicated_functions,
                              level,
                              id);
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
