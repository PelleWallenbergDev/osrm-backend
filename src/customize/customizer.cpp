#include "extractor/node_data_container.hpp"

#include "customizer/cell_customizer.hpp"
#include "customizer/customizer.hpp"
#include "customizer/edge_based_graph.hpp"
#include "customizer/files.hpp"
#include "customizer/temporal_cell_customizer.hpp"
#include "customizer/temporal_cell_files.hpp"
#include "customizer/temporal_files.hpp"

#include "engine/temporal_traffic.hpp"
#include "partitioner/cell_statistics.hpp"
#include "partitioner/cell_storage.hpp"
#include "partitioner/edge_based_graph_reader.hpp"
#include "partitioner/files.hpp"
#include "partitioner/multi_level_partition.hpp"

#include "storage/shared_memory_ownership.hpp"

#include "updater/updater.hpp"

#include "util/exclude_flag.hpp"
#include "util/log.hpp"
#include "util/timing_util.hpp"

#include <boost/assert.hpp>

#include <tbb/global_control.h>

#include <filesystem>
#include <limits>
#include <optional>
#include <unordered_map>

namespace osrm::customizer
{

namespace
{

template <typename Partition, typename CellStorage>
void printUnreachableStatistics(const Partition &partition,
                                const CellStorage &storage,
                                const CellMetric &metric)
{
    util::Log() << "Unreachable nodes statistics per level";

    for (std::size_t level = 1; level < partition.GetNumberOfLevels(); ++level)
    {
        auto num_cells = partition.GetNumberOfCells(level);
        std::size_t invalid_sources = 0;
        std::size_t invalid_destinations = 0;
        for (std::uint32_t cell_id = 0; cell_id < num_cells; ++cell_id)
        {
            const auto &cell = storage.GetCell(metric, level, cell_id);
            for (auto node : cell.GetSourceNodes())
            {
                const auto &weights = cell.GetOutWeight(node);
                invalid_sources +=
                    std::all_of(weights.begin(),
                                weights.end(),
                                [](auto weight) { return weight == INVALID_EDGE_WEIGHT; });
            }
            for (auto node : cell.GetDestinationNodes())
            {
                const auto &weights = cell.GetInWeight(node);
                invalid_destinations +=
                    std::all_of(weights.begin(),
                                weights.end(),
                                [](auto weight) { return weight == INVALID_EDGE_WEIGHT; });
            }
        }

        if (invalid_sources > 0 || invalid_destinations > 0)
        {
            util::Log(logWARNING) << "Level " << level << " unreachable boundary nodes per cell: "
                                  << (invalid_sources / (float)num_cells) << " sources, "
                                  << (invalid_destinations / (float)num_cells) << " destinations";
        }
    }
}

auto LoadAndUpdateEdgeExpandedGraph(const CustomizationConfig &config,
                                    const partitioner::MultiLevelPartition &mlp,
                                    std::vector<EdgeWeight> &node_weights,
                                    std::vector<EdgeDuration> &node_durations,
                                    std::vector<EdgeDistance> &node_distances,
                                    std::uint32_t &connectivity_checksum)
{
    updater::Updater updater(config.updater_config);

    std::vector<extractor::EdgeBasedEdge> edge_based_edge_list;
    EdgeID num_nodes = updater.LoadAndUpdateEdgeExpandedGraph(
        edge_based_edge_list, node_weights, node_durations, connectivity_checksum);

    extractor::files::readEdgeBasedNodeDistances(config.GetPath(".osrm.enw"), node_distances);

    auto directed = partitioner::splitBidirectionalEdges(edge_based_edge_list);

    auto tidied = partitioner::prepareEdgesForUsageInGraph<
        typename partitioner::MultiLevelEdgeBasedGraph::InputEdge>(std::move(directed));

    auto edge_based_graph = partitioner::MultiLevelEdgeBasedGraph(mlp, num_nodes, tidied);

    return edge_based_graph;
}

std::vector<CellMetric> customizeFilteredMetrics(const partitioner::MultiLevelEdgeBasedGraph &graph,
                                                 const partitioner::CellStorage &storage,
                                                 const CellCustomizer &customizer,
                                                 const std::vector<std::vector<bool>> &node_filters)
{
    std::vector<CellMetric> metrics;
    metrics.reserve(node_filters.size());

    for (const auto &filter : node_filters)
    {
        auto metric = storage.MakeMetric();
        customizer.Customize(graph, storage, filter, metric);
        metrics.push_back(std::move(metric));
    }

    return metrics;
}

std::vector<TemporalProfileID>
buildTemporalOverlayNodeProfileIDs(const extractor::EdgeBasedNodeDataContainer &node_data,
                                   const TemporalProfileIndex &profile_index)
{
    std::vector<TemporalProfileID> profile_ids(node_data.NumberOfNodes(),
                                               INVALID_TEMPORAL_PROFILE_ID);
    for (NodeID node = 0; node < profile_ids.size(); ++node)
    {
        const auto geometry_id = node_data.GetGeometryID(node);
        profile_ids[node] = geometry_id.forward
                                ? profile_index.GetForwardProfileID(geometry_id.id)
                                : profile_index.GetReverseProfileID(geometry_id.id);
    }

    return profile_ids;
}

template <typename GraphT>
std::vector<EdgeDuration>
buildTemporalOverlayEdgeTurnDurations(const GraphT &graph,
                                      const std::vector<EdgeDuration> &node_durations)
{
    std::vector<EdgeDuration> edge_turn_durations(graph.GetNumberOfEdges(),
                                                  INVALID_EDGE_DURATION);
    for (NodeID from = 0; from < graph.GetNumberOfNodes(); ++from)
    {
        const auto static_node_duration = node_durations[from];
        for (const auto edge : graph.GetAdjacentEdgeRange(from))
        {
            const auto static_edge_duration = EdgeDuration{graph.GetEdgeData(edge).duration};
            edge_turn_durations[edge] = engine::temporal::detail::SafeDurationSubFloorZero(
                static_edge_duration, static_node_duration);
        }
    }

    return edge_turn_durations;
}

struct TemporalOverlayEvaluator
{
    const extractor::EdgeBasedNodeDataContainer &node_data;
    const std::vector<EdgeDuration> &node_durations;
    const TemporalProfileIndex *profile_index = nullptr;
    const TemporalProfileStorage *profile_storage = nullptr;
    const DenseTemporalProfileCache *dense_profile_cache = nullptr;
    const std::vector<TemporalProfileID> *node_profile_ids = nullptr;
    const std::vector<EdgeDuration> *edge_turn_durations = nullptr;
    std::uint32_t bucket_size_minutes = 0;
    std::uint32_t week_bucket_count = 0;

    std::uint32_t GetBucketSizeMinutes() const { return bucket_size_minutes; }
    std::uint32_t GetWeekBucketCount() const { return week_bucket_count; }

    std::uint32_t GetBucketAfterDuration(const std::uint32_t departure_bucket,
                                         const EdgeDuration elapsed_duration) const
    {
        if (bucket_size_minutes == 0 || week_bucket_count == 0)
        {
            return 0;
        }

        const auto departure_timestamp =
            static_cast<std::time_t>(departure_bucket) * bucket_size_minutes * 60;
        const auto arrival_clock = engine::temporal::AdvanceTemporalClock(
            engine::temporal::ToTemporalClock(departure_timestamp), elapsed_duration);
        return std::min(engine::temporal::TimestampToWeekBucketFromClock(arrival_clock,
                                                                         bucket_size_minutes),
                        week_bucket_count - 1);
    }

    template <typename GraphT>
    EdgeDuration
    GetBaseEdgeDuration(const GraphT &graph,
                        const NodeID from,
                        const EdgeID edge,
                        const std::uint32_t current_bucket) const
    {
        const auto static_node_duration = node_durations[from];
        auto node_duration = static_node_duration;

        if (profile_storage != nullptr)
        {
            const auto profile_id =
                node_profile_ids != nullptr && from < node_profile_ids->size()
                    ? (*node_profile_ids)[from]
                    : (profile_index != nullptr
                           ? [&]() {
                                 const auto geometry_id = node_data.GetGeometryID(from);
                                 return geometry_id.forward
                                            ? profile_index->GetForwardProfileID(geometry_id.id)
                                            : profile_index->GetReverseProfileID(geometry_id.id);
                             }()
                           : INVALID_TEMPORAL_PROFILE_ID);
            if (profile_id != INVALID_TEMPORAL_PROFILE_ID)
            {
                const auto week_bucket =
                    week_bucket_count == 0 ? current_bucket
                                           : std::min(current_bucket, week_bucket_count - 1);
                const auto temporal_duration =
                    dense_profile_cache != nullptr && dense_profile_cache->HasProfile(profile_id)
                        ? dense_profile_cache->GetDuration(profile_id, week_bucket)
                        : profile_storage->GetDuration(profile_id, week_bucket);
                if (temporal_duration != INVALID_EDGE_DURATION &&
                    engine::temporal::detail::IsPlausibleTemporalDuration(temporal_duration,
                                                                          static_node_duration))
                {
                    node_duration = temporal_duration;
                }
            }
        }

        const auto turn_duration =
            edge_turn_durations != nullptr && edge < edge_turn_durations->size()
                ? (*edge_turn_durations)[edge]
                : engine::temporal::detail::SafeDurationSubFloorZero(
                      EdgeDuration{graph.GetEdgeData(edge).duration}, static_node_duration);
        return engine::temporal::detail::SafeDurationAdd(node_duration, turn_duration);
    }
};

std::vector<TemporalCellMetric>
customizeFilteredTemporalMetrics(const partitioner::MultiLevelEdgeBasedGraph &graph,
                                 const partitioner::CellStorage &storage,
                                 const TemporalCellCustomizer &customizer,
                                 const std::vector<std::vector<bool>> &node_filters,
                                 const TemporalOverlayEvaluator &evaluator,
                                 TemporalFunctionStorage &function_storage,
                                 const LevelID first_level,
                                 const LevelID last_level)
{
    std::vector<TemporalCellMetric> metrics;
    metrics.reserve(node_filters.size());

    for (const auto &filter : node_filters)
    {
        auto metric = MakeTemporalCellMetric(storage);
        customizer.Customize(
            graph, storage, filter, evaluator, metric, function_storage, first_level, last_level);
        metrics.push_back(std::move(metric));
    }

    return metrics;
}

struct TemporalOverlayWorkEstimate
{
    std::uint64_t total_cells = 0;
    std::uint64_t total_source_rows = 0;
    std::uint64_t total_destination_rows = 0;
    std::uint64_t total_boundary_pairs = 0;
};

TemporalOverlayWorkEstimate estimateTemporalOverlayWork(const partitioner::MultiLevelPartition &mlp,
                                                        const partitioner::CellStorage &storage,
                                                        const LevelID first_level,
                                                        const LevelID last_level)
{
    TemporalOverlayWorkEstimate estimate;
    const auto begin_level = std::max<std::size_t>(1UL, static_cast<std::size_t>(first_level));
    const auto end_level =
        std::min<std::size_t>(mlp.GetNumberOfLevels() > 0 ? mlp.GetNumberOfLevels() - 1 : 0,
                              static_cast<std::size_t>(last_level));

    if (mlp.GetNumberOfLevels() <= 1 || begin_level > end_level)
    {
        return estimate;
    }

    for (std::size_t level = begin_level; level <= end_level; ++level)
    {
        const auto cell_count = mlp.GetNumberOfCells(level);
        estimate.total_cells += cell_count;

        for (std::size_t id = 0; id < cell_count; ++id)
        {
            const auto cell = storage.GetUnfilledCell(level, id);
            const auto source_count = static_cast<std::uint64_t>(
                std::distance(cell.GetSourceNodes().begin(), cell.GetSourceNodes().end()));
            const auto destination_count = static_cast<std::uint64_t>(
                std::distance(cell.GetDestinationNodes().begin(), cell.GetDestinationNodes().end()));
            estimate.total_source_rows += source_count;
            estimate.total_destination_rows += destination_count;
            estimate.total_boundary_pairs += source_count * destination_count;
        }
    }

    return estimate;
}
} // namespace

int Customizer::Run(const CustomizationConfig &config)
{
    tbb::global_control gc(tbb::global_control::max_allowed_parallelism,
                           config.requested_num_threads);

    TIMER_START(loading_data);

    partitioner::MultiLevelPartition mlp;
    partitioner::files::readPartition(config.GetPath(".osrm.partition"), mlp);

    std::vector<EdgeWeight> node_weights;
    std::vector<EdgeDuration> node_durations; // TODO: remove when durations are optional
    std::vector<EdgeDistance> node_distances; // TODO: remove when distances are optional
    std::uint32_t connectivity_checksum = 0;
    auto graph = LoadAndUpdateEdgeExpandedGraph(
        config, mlp, node_weights, node_durations, node_distances, connectivity_checksum);
    BOOST_ASSERT(graph.GetNumberOfNodes() == node_weights.size());
    std::for_each(
        node_weights.begin(), node_weights.end(), [](auto &w) { w &= EdgeWeight{0x7fffffff}; });
    util::Log() << "Loaded edge based graph: " << graph.GetNumberOfEdges() << " edges, "
                << graph.GetNumberOfNodes() << " nodes";

    partitioner::CellStorage storage;
    partitioner::files::readCells(config.GetPath(".osrm.cells"), storage);
    TIMER_STOP(loading_data);

    extractor::EdgeBasedNodeDataContainer node_data;
    extractor::files::readNodeData(config.GetPath(".osrm.ebg_nodes"), node_data);

    extractor::ProfileProperties properties;
    extractor::files::readProfileProperties(config.GetPath(".osrm.properties"), properties);

    util::Log() << "Loading partition data took " << TIMER_SEC(loading_data) << " seconds";

    TIMER_START(cell_customize);
    auto filter = util::excludeFlagsToNodeFilter(graph.GetNumberOfNodes(), node_data, properties);
    auto metrics = customizeFilteredMetrics(graph, storage, CellCustomizer{mlp}, filter);
    TIMER_STOP(cell_customize);
    util::Log() << "Cells customization took " << TIMER_SEC(cell_customize) << " seconds";

    partitioner::printCellStatistics(mlp, storage);
    for (const auto &metric : metrics)
    {
        printUnreachableStatistics(mlp, storage, metric);
    }

    if (config.updater_config.write_temporal_overlay_sidecar)
    {
        TIMER_START(temporal_cell_customize);

        const auto temporal_index_path = config.updater_config.GetPath(".osrm.temporal_index");
        const auto temporal_profiles_path = config.updater_config.GetPath(".osrm.temporal_profiles");
        const auto temporal_meta_path = config.updater_config.GetPath(".osrm.temporal_meta");

        const auto has_temporal_index = std::filesystem::exists(temporal_index_path);
        const auto has_temporal_profiles = std::filesystem::exists(temporal_profiles_path);
        const auto has_temporal_meta = std::filesystem::exists(temporal_meta_path);
        const auto has_any_temporal_sidecar =
            has_temporal_index || has_temporal_profiles || has_temporal_meta;
        const auto has_complete_temporal_sidecar =
            has_temporal_index && has_temporal_profiles && has_temporal_meta;

        if (has_any_temporal_sidecar && !has_complete_temporal_sidecar)
        {
            throw util::exception(
                "Temporal overlay customization requires .osrm.temporal_index, "
                ".osrm.temporal_profiles, and .osrm.temporal_meta to either all exist or all be "
                "absent");
        }

        std::optional<TemporalProfileIndex> temporal_profile_index;
        std::optional<TemporalProfileStorage> temporal_profile_storage;
        std::optional<DenseTemporalProfileCache> dense_temporal_profile_cache;
        std::optional<std::vector<TemporalProfileID>> temporal_node_profile_ids;
        std::vector<EdgeDuration> temporal_edge_turn_durations =
            buildTemporalOverlayEdgeTurnDurations(graph, node_durations);
        if (has_complete_temporal_sidecar)
        {
            temporal_profile_index.emplace();
            temporal_profile_storage.emplace();
            files::readTemporalProfileIndex(temporal_index_path, *temporal_profile_index);
            files::readTemporalProfiles(temporal_profiles_path, *temporal_profile_storage);
            files::readTemporalMeta(temporal_meta_path, *temporal_profile_storage);
            dense_temporal_profile_cache.emplace(*temporal_profile_storage);
            temporal_node_profile_ids.emplace(
                buildTemporalOverlayNodeProfileIDs(node_data, *temporal_profile_index));
        }

        TemporalFunctionStorage temporal_overlay_storage;
        temporal_overlay_storage.meta.bucket_size_minutes =
            temporal_profile_storage ? temporal_profile_storage->GetBucketSizeMinutes()
                                     : config.updater_config.temporal_bucket_size_minutes;
        temporal_overlay_storage.meta.week_bucket_count =
            temporal_profile_storage ? temporal_profile_storage->GetWeekBucketCount()
                                     : config.updater_config.temporal_week_bucket_count;
        temporal_overlay_storage.meta.encoding_version =
            config.updater_config.write_temporal_overlay_debug_dense
                ? TEMPORAL_FUNCTION_ENCODING_VERSION_DENSE
                : TEMPORAL_FUNCTION_ENCODING_VERSION_DCT;

        const auto first_level = static_cast<LevelID>(
            std::min<std::uint32_t>(config.updater_config.temporal_overlay_min_level,
                                    std::numeric_limits<LevelID>::max()));
        const auto last_level = static_cast<LevelID>(
            std::min<std::uint32_t>(config.updater_config.temporal_overlay_max_level,
                                    std::numeric_limits<LevelID>::max()));

        const auto preferred_coeff_count =
            temporal_overlay_storage.meta.encoding_version ==
                    TEMPORAL_FUNCTION_ENCODING_VERSION_DCT
                ? config.updater_config.temporal_dct_coeff_count
                : 0U;
        const auto work_estimate =
            estimateTemporalOverlayWork(mlp, storage, first_level, last_level);

        util::Log() << "Starting temporal overlay customization with write_temporal_overlay_sidecar="
                    << config.updater_config.write_temporal_overlay_sidecar
                    << ", bucket_size_minutes="
                    << temporal_overlay_storage.meta.bucket_size_minutes
                    << ", week_bucket_count=" << temporal_overlay_storage.meta.week_bucket_count
                    << ", encoding_version=" << temporal_overlay_storage.meta.encoding_version
                    << ", preferred_coeff_count=" << preferred_coeff_count
                    << ", first_level=" << static_cast<unsigned>(first_level)
                    << ", last_level=" << static_cast<unsigned>(last_level)
                    << ", filter_count=" << filter.size()
                    << ", estimated_cells=" << work_estimate.total_cells
                    << ", estimated_source_rows=" << work_estimate.total_source_rows
                    << ", estimated_boundary_pairs=" << work_estimate.total_boundary_pairs;

        if (temporal_profile_storage)
        {
            util::Log() << "Loaded temporal base sidecar for overlay customization with "
                        << temporal_profile_storage->profile_offsets.size() << " profile(s)";
            util::Log() << "Prepared dense temporal base cache with "
                        << dense_temporal_profile_cache->profile_count << " profile(s), bytes="
                        << dense_temporal_profile_cache->GetStorageBytes();
            util::Log() << "Prepared temporal overlay base metadata cache with "
                        << temporal_node_profile_ids->size() << " node profile id(s) and "
                        << temporal_edge_turn_durations.size() << " edge turn duration(s)";
        }
        else
        {
            util::Log(logWARNING)
                << "Temporal overlay customization is running without a pre-existing temporal "
                   "base sidecar; base graph durations will stay static";
            util::Log() << "Prepared temporal overlay turn-duration cache with "
                        << temporal_edge_turn_durations.size() << " edge turn duration(s)";
        }

        TemporalOverlayEvaluator evaluator{node_data,
                                           node_durations,
                                           temporal_profile_index ? &*temporal_profile_index
                                                                  : nullptr,
                                           temporal_profile_storage ? &*temporal_profile_storage
                                                                    : nullptr,
                                           dense_temporal_profile_cache
                                               ? &*dense_temporal_profile_cache
                                               : nullptr,
                                           temporal_node_profile_ids
                                               ? &*temporal_node_profile_ids
                                               : nullptr,
                                           &temporal_edge_turn_durations,
                                           temporal_overlay_storage.meta.bucket_size_minutes,
                                           temporal_overlay_storage.meta.week_bucket_count};
        const auto temporal_metrics = customizeFilteredTemporalMetrics(
            graph,
            storage,
            TemporalCellCustomizer{mlp, preferred_coeff_count},
            filter,
            evaluator,
            temporal_overlay_storage,
            first_level,
            last_level);

        util::Log() << "Temporal overlay customization finished building "
                    << temporal_overlay_storage.profile_offsets.size()
                    << " temporal shortcut function(s) across "
                    << temporal_metrics.size() << " exclude-class metric set(s)";

        TIMER_STOP(temporal_cell_customize);
        util::Log() << "Temporal overlay customization took "
                    << TIMER_SEC(temporal_cell_customize) << " seconds";

        TIMER_START(writing_temporal_mld_data);
        std::unordered_map<std::string, std::vector<TemporalCellMetric>>
            temporal_metric_exclude_classes = {
                {properties.GetWeightName(), temporal_metrics},
            };
        files::writeTemporalCellMetrics(config.GetPath(".osrm.temporal_cell_metrics"),
                                        temporal_metric_exclude_classes);
        files::writeTemporalCellStorage(config.GetPath(".osrm.temporal_cell_profiles"),
                                        temporal_overlay_storage);
        files::writeTemporalCellMeta(config.GetPath(".osrm.temporal_cell_meta"),
                                     temporal_overlay_storage.meta);
        TIMER_STOP(writing_temporal_mld_data);
        util::Log() << "Temporal MLD customization writing took "
                    << TIMER_SEC(writing_temporal_mld_data) << " seconds";
    }

    TIMER_START(writing_mld_data);
    std::unordered_map<std::string, std::vector<CellMetric>> metric_exclude_classes = {
        {properties.GetWeightName(), std::move(metrics)},
    };
    files::writeCellMetrics(config.GetPath(".osrm.cell_metrics"), metric_exclude_classes);
    TIMER_STOP(writing_mld_data);
    util::Log() << "MLD customization writing took " << TIMER_SEC(writing_mld_data) << " seconds";

    TIMER_START(writing_graph);
    MultiLevelEdgeBasedGraph shaved_graph{std::move(graph),
                                          std::move(node_weights),
                                          std::move(node_durations),
                                          std::move(node_distances)};
    customizer::files::writeGraph(
        config.GetPath(".osrm.mldgr"), shaved_graph, connectivity_checksum);
    TIMER_STOP(writing_graph);
    util::Log() << "Graph writing took " << TIMER_SEC(writing_graph) << " seconds";

    return 0;
}

} // namespace osrm::customizer
