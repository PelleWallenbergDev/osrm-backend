#include "customizer/files.hpp"
#include "customizer/edge_based_graph.hpp"
#include "customizer/temporal_cell_files.hpp"
#include "engine/datafacade/contiguous_block_allocator.hpp"
#include "engine/datafacade/contiguous_internalmem_datafacade.hpp"
#include "engine/temporal/temporal_profile_decoder.hpp"
#include "partitioner/cell_storage.hpp"
#include "partitioner/edge_based_graph.hpp"
#include "partitioner/files.hpp"
#include "partitioner/multi_level_graph.hpp"
#include "partitioner/multi_level_partition.hpp"
#include "storage/storage.hpp"
#include "storage/view_factory.hpp"
#include "util/static_graph.hpp"

#include "../common/temporary_file.hpp"

#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
using namespace osrm;

struct MockEdge
{
    NodeID source;
    NodeID target;
    EdgeWeight weight;
};

auto makePartitionerGraph(const partitioner::MultiLevelPartition &partition,
                          const std::vector<MockEdge> &mock_edges)
{
    using PartitionerEdge =
        util::static_graph_details::SortableEdgeWithData<partitioner::EdgeBasedGraphEdgeData>;

    std::vector<PartitionerEdge> edges;
    std::size_t max_node_id = 0;
    for (const auto &mock_edge : mock_edges)
    {
        max_node_id =
            std::max<std::size_t>(max_node_id, std::max(mock_edge.source, mock_edge.target));
        edges.emplace_back(mock_edge.source,
                           mock_edge.target,
                           mock_edge.source,
                           mock_edge.weight,
                           EdgeDistance{1.0},
                           EdgeDuration{0},
                           true,
                           false);
        edges.emplace_back(mock_edge.target,
                           mock_edge.source,
                           mock_edge.target,
                           mock_edge.weight,
                           EdgeDistance{1.0},
                           EdgeDuration{0},
                           false,
                           true);
    }

    std::sort(edges.begin(), edges.end());
    return partitioner::MultiLevelEdgeBasedGraph(
        partition, static_cast<NodeID>(max_node_id + 1), edges);
}

auto makeCustomizerGraph(partitioner::MultiLevelEdgeBasedGraph partition_graph)
{
    const auto num_nodes = partition_graph.GetNumberOfNodes();
    std::vector<EdgeWeight> node_weights(num_nodes, EdgeWeight{0});
    std::vector<EdgeDuration> node_durations(num_nodes, EdgeDuration{0});
    std::vector<EdgeDistance> node_distances(num_nodes, EdgeDistance{0.0});

    return customizer::MultiLevelEdgeBasedGraph(std::move(partition_graph),
                                                std::move(node_weights),
                                                std::move(node_durations),
                                                std::move(node_distances));
}

class TestAllocator final : public engine::datafacade::ContiguousBlockAllocator
{
  public:
    TestAllocator(std::unique_ptr<char[]> memory_, storage::SharedDataIndex index_)
        : memory(std::move(memory_)), index(std::move(index_))
    {
    }

    const storage::SharedDataIndex &GetIndex() override { return index; }

  private:
    std::unique_ptr<char[]> memory;
    storage::SharedDataIndex index;
};

std::shared_ptr<TestAllocator> makeAllocator(const bool include_temporal_sidecar)
{
    TemporaryFile partition_file;
    TemporaryFile cells_file;
    TemporaryFile metrics_file;
    TemporaryFile graph_file;
    TemporaryFile temporal_metrics_file;
    TemporaryFile temporal_profiles_file;
    TemporaryFile temporal_meta_file;

    std::vector<CellID> level_one_partition{{0, 0, 1, 1}};
    partitioner::MultiLevelPartition partition{{level_one_partition}, {2}};

    const std::vector<MockEdge> edges = {
        {0, 1, EdgeWeight{1}},
        {0, 2, EdgeWeight{1}},
        {2, 3, EdgeWeight{1}},
        {3, 1, EdgeWeight{1}},
    };

    auto partition_graph = makePartitionerGraph(partition, edges);
    partitioner::CellStorage cell_storage(partition, partition_graph);
    auto graph = makeCustomizerGraph(std::move(partition_graph));
    auto scalar_metric = cell_storage.MakeMetric();
    std::unordered_map<std::string, std::vector<customizer::CellMetric>> scalar_metrics = {
        {"duration", {scalar_metric}}};

    partitioner::files::writePartition(partition_file.path, partition);
    partitioner::files::writeCells(cells_file.path, cell_storage);
    customizer::files::writeCellMetrics(metrics_file.path, scalar_metrics);
    customizer::files::writeGraph(graph_file.path, graph, 17U);

    if (include_temporal_sidecar)
    {
        auto temporal_metric = customizer::MakeTemporalCellMetric(cell_storage);
        const auto cell_data = cell_storage.GetCellData(1, 0);
        BOOST_REQUIRE_EQUAL(cell_data.num_source_nodes, 1U);
        BOOST_REQUIRE_EQUAL(cell_data.num_destination_nodes, 1U);

        temporal_metric.function_ids[cell_data.value_offset] = 0;
        temporal_metric.min_durations[cell_data.value_offset] = EdgeDuration{10};

        const std::vector<EdgeDuration::value_type> profile = {10, 12, 14, 16};
        customizer::TemporalFunctionStorage function_storage;
        function_storage.meta.bucket_size_minutes = 5;
        function_storage.meta.week_bucket_count = static_cast<std::uint32_t>(profile.size());
        function_storage.meta.encoding_version = customizer::TEMPORAL_FUNCTION_ENCODING_VERSION_DCT;
        function_storage.profile_offsets = {0};
        function_storage.profile_sizes = {static_cast<std::uint32_t>(profile.size())};
        function_storage.min_durations = {10};
        function_storage.freeflow_durations = {10};
        function_storage.profile_flags = {
            static_cast<customizer::TemporalFunctionFlags>(
                customizer::TEMPORAL_FUNCTION_FLAG_FIFO_VALID)};
        function_storage.coeffs =
            engine::temporal::CompressTemporalFunction(profile, function_storage.profile_sizes[0]);
        std::unordered_map<std::string, std::vector<customizer::TemporalCellMetric>>
            temporal_metrics = {{"duration", {temporal_metric}}};

        customizer::files::writeTemporalCellMetrics(temporal_metrics_file.path, temporal_metrics);
        customizer::files::writeTemporalCellStorage(temporal_profiles_file.path, function_storage);
        customizer::files::writeTemporalCellMeta(temporal_meta_file.path, function_storage.meta);
    }

    auto layout = std::make_unique<storage::ContiguousDataLayout>();
    storage::populateLayoutFromFile(partition_file.path, *layout);
    storage::populateLayoutFromFile(cells_file.path, *layout);
    storage::populateLayoutFromFile(metrics_file.path, *layout);
    storage::populateLayoutFromFile(graph_file.path, *layout);
    if (include_temporal_sidecar)
    {
        storage::populateLayoutFromFile(temporal_metrics_file.path, *layout);
        storage::populateLayoutFromFile(temporal_profiles_file.path, *layout);
        storage::populateLayoutFromFile(temporal_meta_file.path, *layout);
    }

    auto memory = std::make_unique<char[]>(layout->GetSizeOfLayout());
    std::vector<storage::SharedDataIndex::AllocatedRegion> regions;
    regions.push_back({memory.get(), std::move(layout)});
    storage::SharedDataIndex index(std::move(regions));

    auto partition_view = storage::make_partition_view(index, "/mld/multilevelpartition");
    partitioner::files::readPartition(partition_file.path, partition_view);

    auto cell_storage_view = storage::make_cell_storage_view(index, "/mld/cellstorage");
    partitioner::files::readCells(cells_file.path, cell_storage_view);

    auto metric_views = storage::make_cell_metric_view(index, "/mld/metrics/duration");
    std::unordered_map<std::string, std::vector<customizer::CellMetricView>> metric_targets = {
        {"duration", std::move(metric_views)}};
    customizer::files::readCellMetrics(metrics_file.path, metric_targets);

    auto graph_view = storage::make_multi_level_graph_view(index, "/mld/multilevelgraph");
    std::uint32_t connectivity_checksum = 0;
    customizer::files::readGraph(graph_file.path, graph_view, connectivity_checksum);

    if (include_temporal_sidecar)
    {
        auto temporal_metric_views =
            storage::make_temporal_cell_metric_view(index, "/mld/temporal_metrics/duration");
        std::unordered_map<std::string, std::vector<customizer::TemporalCellMetricView>>
            temporal_metric_targets = {{"duration", std::move(temporal_metric_views)}};
        customizer::files::readTemporalCellMetrics(temporal_metrics_file.path,
                                                   temporal_metric_targets);

        auto temporal_storage_view =
            storage::make_temporal_function_storage_view(index, "/mld/temporal_metric_storage");
        customizer::files::readTemporalCellStorage(temporal_profiles_file.path,
                                                   temporal_storage_view);
        customizer::files::readTemporalCellMeta(temporal_meta_file.path, temporal_storage_view);
    }

    return std::make_shared<TestAllocator>(std::move(memory), std::move(index));
}
} // namespace

BOOST_AUTO_TEST_SUITE(temporal_cell_facade)

BOOST_AUTO_TEST_CASE(mld_facade_reads_temporal_shortcuts_from_dct_sidecar)
{
    auto allocator = makeAllocator(true);
    engine::datafacade::ContiguousInternalMemoryAlgorithmDataFacade<engine::datafacade::MLD> facade(
        allocator, "duration", 0);

    BOOST_CHECK(facade.HasTemporalShortcut(1, 0, 0, 1));
    BOOST_CHECK(!facade.HasTemporalShortcut(1, 0, 1, 0));
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(facade.GetTemporalShortcutMinDuration(1, 0, 0, 1)),
                      10);
    BOOST_CHECK_SMALL(static_cast<double>(
                          std::abs(from_alias<std::int32_t>(
                                       facade.GetTemporalShortcutDuration(1, 0, 0, 1, 0)) -
                                   10)),
                      1.1);
    BOOST_CHECK_SMALL(static_cast<double>(
                          std::abs(from_alias<std::int32_t>(
                                       facade.GetTemporalShortcutDuration(1, 0, 0, 1, 2)) -
                                   14)),
                      1.1);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(facade.GetTemporalShortcutDuration(1, 0, 1, 0, 0)),
                      from_alias<std::int32_t>(INVALID_EDGE_DURATION));
}

BOOST_AUTO_TEST_CASE(mld_facade_handles_missing_temporal_shortcut_sidecar)
{
    auto allocator = makeAllocator(false);
    engine::datafacade::ContiguousInternalMemoryAlgorithmDataFacade<engine::datafacade::MLD> facade(
        allocator, "duration", 0);

    BOOST_CHECK(!facade.HasTemporalShortcut(1, 0, 0, 1));
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(facade.GetTemporalShortcutDuration(1, 0, 0, 1, 0)),
                      from_alias<std::int32_t>(INVALID_EDGE_DURATION));
    BOOST_CHECK_EQUAL(
        from_alias<std::int32_t>(facade.GetTemporalShortcutMinDuration(1, 0, 0, 1)),
        from_alias<std::int32_t>(INVALID_EDGE_DURATION));
}

BOOST_AUTO_TEST_SUITE_END()
