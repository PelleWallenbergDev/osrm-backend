#include "customizer/temporal_cell_customizer.hpp"
#include "partitioner/multi_level_graph.hpp"
#include "partitioner/multi_level_partition.hpp"
#include "util/static_graph.hpp"

#include <boost/test/unit_test.hpp>

#include <cstdint>
#include <map>
#include <utility>
#include <vector>

using namespace osrm;
using namespace osrm::customizer;
using namespace osrm::partitioner;
using namespace osrm::util;

namespace
{
struct MockEdge
{
    NodeID start;
    NodeID target;
    EdgeWeight weight;
};

auto makeGraph(const MultiLevelPartition &mlp, const std::vector<MockEdge> &mock_edges)
{
    struct EdgeData
    {
        EdgeWeight weight;
        EdgeDuration duration;
        EdgeDistance distance;
        bool forward;
        bool backward;
    };

    using Edge = static_graph_details::SortableEdgeWithData<EdgeData>;
    std::vector<Edge> edges;
    std::size_t max_id = 0;
    for (const auto &mock_edge : mock_edges)
    {
        max_id = std::max<std::size_t>(max_id, std::max(mock_edge.start, mock_edge.target));
        edges.push_back(Edge{mock_edge.start,
                             mock_edge.target,
                             mock_edge.weight,
                             EdgeDuration{0},
                             EdgeDistance{1.0},
                             true,
                             false});
        edges.push_back(Edge{mock_edge.target,
                             mock_edge.start,
                             mock_edge.weight,
                             EdgeDuration{0},
                             EdgeDistance{1.0},
                             false,
                             true});
    }

    std::sort(edges.begin(), edges.end());
    return partitioner::MultiLevelGraph<EdgeData, osrm::storage::Ownership::Container>(
        mlp, static_cast<NodeID>(max_id + 1), edges);
}

struct MockTemporalEvaluator
{
    std::uint32_t bucket_size_minutes = 5;
    std::uint32_t week_bucket_count = 4;
    std::map<std::pair<NodeID, NodeID>, std::vector<EdgeDuration::value_type>> durations;

    std::uint32_t GetBucketSizeMinutes() const { return bucket_size_minutes; }
    std::uint32_t GetWeekBucketCount() const { return week_bucket_count; }

    std::uint32_t GetBucketAfterDuration(const std::uint32_t departure_bucket,
                                         const EdgeDuration elapsed_duration) const
    {
        return (departure_bucket + static_cast<std::uint32_t>(from_alias<std::int32_t>(
                                       elapsed_duration))) %
               week_bucket_count;
    }

    template <typename GraphT>
    EdgeDuration GetBaseEdgeDuration(const GraphT &graph,
                                     const NodeID from,
                                     const EdgeID edge,
                                     const std::uint32_t current_bucket) const
    {
        const auto to = graph.GetTarget(edge);
        const auto iterator = durations.find({from, to});
        if (iterator == durations.end() || current_bucket >= iterator->second.size())
        {
            return INVALID_EDGE_DURATION;
        }

        return EdgeDuration{iterator->second[current_bucket]};
    }
};

template <typename CellT>
TemporalFunctionID FindFunctionID(const CellT &cell, const NodeID source, const NodeID destination)
{
    auto destination_iterator = cell.GetDestinationNodes().begin();
    auto function_iterator = cell.GetOutFunctionID(source).begin();
    for (; destination_iterator != cell.GetDestinationNodes().end();
         ++destination_iterator, ++function_iterator)
    {
        if (*destination_iterator == destination)
        {
            return *function_iterator;
        }
    }

    return INVALID_TEMPORAL_FUNCTION_ID;
}

void CheckTemporalStorageEquivalent(const TemporalFunctionStorage &lhs,
                                    const TemporalFunctionStorage &rhs)
{
    BOOST_CHECK_EQUAL(lhs.meta.bucket_size_minutes, rhs.meta.bucket_size_minutes);
    BOOST_CHECK_EQUAL(lhs.meta.week_bucket_count, rhs.meta.week_bucket_count);
    BOOST_CHECK_EQUAL(lhs.meta.encoding_version, rhs.meta.encoding_version);
    BOOST_REQUIRE_EQUAL(lhs.profile_offsets.size(), rhs.profile_offsets.size());
    BOOST_REQUIRE_EQUAL(lhs.profile_sizes.size(), rhs.profile_sizes.size());
    BOOST_REQUIRE_EQUAL(lhs.min_durations.size(), rhs.min_durations.size());
    BOOST_REQUIRE_EQUAL(lhs.freeflow_durations.size(), rhs.freeflow_durations.size());
    BOOST_REQUIRE_EQUAL(lhs.profile_flags.size(), rhs.profile_flags.size());

    for (std::size_t function_id = 0; function_id < lhs.profile_offsets.size(); ++function_id)
    {
        BOOST_CHECK_EQUAL(lhs.profile_sizes[function_id], rhs.profile_sizes[function_id]);
        BOOST_CHECK_EQUAL(lhs.min_durations[function_id], rhs.min_durations[function_id]);
        BOOST_CHECK_EQUAL(lhs.freeflow_durations[function_id], rhs.freeflow_durations[function_id]);
        BOOST_CHECK_EQUAL(lhs.profile_flags[function_id], rhs.profile_flags[function_id]);

        for (std::uint32_t bucket = 0; bucket < lhs.meta.week_bucket_count; ++bucket)
        {
            BOOST_CHECK_EQUAL(from_alias<std::int32_t>(
                                  lhs.GetDuration(static_cast<TemporalFunctionID>(function_id),
                                                  bucket)),
                              from_alias<std::int32_t>(
                                  rhs.GetDuration(static_cast<TemporalFunctionID>(function_id),
                                                  bucket)));
        }
    }
}

void CheckTemporalMetricEquivalent(const TemporalCellMetric &lhs, const TemporalCellMetric &rhs)
{
    BOOST_REQUIRE_EQUAL(lhs.function_ids.size(), rhs.function_ids.size());
    BOOST_REQUIRE_EQUAL(lhs.min_durations.size(), rhs.min_durations.size());

    for (std::size_t index = 0; index < lhs.function_ids.size(); ++index)
    {
        BOOST_CHECK_EQUAL(lhs.function_ids[index], rhs.function_ids[index]);
        BOOST_CHECK_EQUAL(from_alias<std::int32_t>(lhs.min_durations[index]),
                          from_alias<std::int32_t>(rhs.min_durations[index]));
    }
}
} // namespace

BOOST_AUTO_TEST_SUITE(temporal_cell_customization)

BOOST_AUTO_TEST_CASE(level_one_temporal_cell_customization_builds_dense_profile)
{
    std::vector<CellID> l1{{0, 0, 1, 1}};
    MultiLevelPartition mlp{{l1}, {2}};

    const std::vector<MockEdge> edges = {
        {0, 1, EdgeWeight{1}},
        {0, 2, EdgeWeight{1}},
        {2, 3, EdgeWeight{1}},
        {3, 1, EdgeWeight{1}},
    };

    auto graph = makeGraph(mlp, edges);
    std::vector<bool> allowed_nodes(graph.GetNumberOfNodes(), true);

    CellStorage cell_storage(mlp, graph);
    auto temporal_metric = MakeTemporalCellMetric(cell_storage);
    TemporalFunctionStorage function_storage;
    function_storage.meta.bucket_size_minutes = 5;
    function_storage.meta.week_bucket_count = 4;
    function_storage.meta.encoding_version = TEMPORAL_FUNCTION_ENCODING_VERSION_DENSE;

    MockTemporalEvaluator evaluator;
    evaluator.durations[{0, 1}] = {1, 2, 3, 4};
    evaluator.durations[{1, 0}] = {1, 2, 3, 4};
    evaluator.durations[{0, 2}] = {50, 50, 50, 50};
    evaluator.durations[{2, 0}] = {50, 50, 50, 50};
    evaluator.durations[{2, 3}] = {1, 1, 1, 1};
    evaluator.durations[{3, 2}] = {1, 1, 1, 1};
    evaluator.durations[{3, 1}] = {1, 1, 1, 1};
    evaluator.durations[{1, 3}] = {1, 1, 1, 1};

    TemporalCellCustomizer customizer(mlp);
    TemporalCellCustomizer::Heap heap(graph.GetNumberOfNodes());
    customizer.Customize(
        graph, heap, cell_storage, allowed_nodes, evaluator, temporal_metric, function_storage, 1, 0);

    const auto metric_cell = temporal_metric.GetCell(cell_storage, 1, 0);
    const auto function_id = FindFunctionID(metric_cell, 0, 1);
    BOOST_REQUIRE_NE(function_id, INVALID_TEMPORAL_FUNCTION_ID);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(temporal_metric.min_durations[0]), 1);

    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(function_storage.GetDuration(function_id, 0)), 1);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(function_storage.GetDuration(function_id, 1)), 2);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(function_storage.GetDuration(function_id, 2)), 3);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(function_storage.GetDuration(function_id, 3)), 4);
}

BOOST_AUTO_TEST_CASE(level_one_temporal_cell_customization_reuses_existing_identical_function)
{
    std::vector<CellID> l1{{0, 0, 1, 1}};
    MultiLevelPartition mlp{{l1}, {2}};

    const std::vector<MockEdge> edges = {
        {0, 1, EdgeWeight{1}},
        {0, 2, EdgeWeight{1}},
        {2, 3, EdgeWeight{1}},
        {3, 1, EdgeWeight{1}},
    };

    auto graph = makeGraph(mlp, edges);
    std::vector<bool> allowed_nodes(graph.GetNumberOfNodes(), true);

    CellStorage cell_storage(mlp, graph);
    auto temporal_metric = MakeTemporalCellMetric(cell_storage);
    TemporalFunctionStorage function_storage;
    function_storage.meta.bucket_size_minutes = 5;
    function_storage.meta.week_bucket_count = 4;
    function_storage.meta.encoding_version = TEMPORAL_FUNCTION_ENCODING_VERSION_DENSE;

    MockTemporalEvaluator evaluator;
    evaluator.durations[{0, 1}] = {1, 2, 3, 4};
    evaluator.durations[{1, 0}] = {1, 2, 3, 4};
    evaluator.durations[{0, 2}] = {50, 50, 50, 50};
    evaluator.durations[{2, 0}] = {50, 50, 50, 50};
    evaluator.durations[{2, 3}] = {1, 1, 1, 1};
    evaluator.durations[{3, 2}] = {1, 1, 1, 1};
    evaluator.durations[{3, 1}] = {1, 1, 1, 1};
    evaluator.durations[{1, 3}] = {1, 1, 1, 1};

    TemporalCellCustomizer customizer(mlp);
    TemporalCellCustomizer::Heap heap(graph.GetNumberOfNodes());
    customizer.Customize(
        graph, heap, cell_storage, allowed_nodes, evaluator, temporal_metric, function_storage, 1, 0);

    const auto metric_cell = temporal_metric.GetCell(cell_storage, 1, 0);
    const auto function_id = FindFunctionID(metric_cell, 0, 1);
    BOOST_REQUIRE_NE(function_id, INVALID_TEMPORAL_FUNCTION_ID);
    BOOST_REQUIRE_EQUAL(function_storage.profile_offsets.size(), 1U);
    BOOST_REQUIRE_EQUAL(function_storage.values.size(), 4U);

    customizer.Customize(
        graph, heap, cell_storage, allowed_nodes, evaluator, temporal_metric, function_storage, 1, 0);

    const auto metric_cell_after = temporal_metric.GetCell(cell_storage, 1, 0);
    const auto repeated_function_id = FindFunctionID(metric_cell_after, 0, 1);
    BOOST_CHECK_EQUAL(repeated_function_id, function_id);
    BOOST_CHECK_EQUAL(function_storage.profile_offsets.size(), 1U);
    BOOST_CHECK_EQUAL(function_storage.values.size(), 4U);
}

BOOST_AUTO_TEST_CASE(level_one_temporal_cell_customization_marks_non_fifo_profiles)
{
    std::vector<CellID> l1{{0, 0, 1, 1}};
    MultiLevelPartition mlp{{l1}, {2}};

    const std::vector<MockEdge> edges = {
        {0, 1, EdgeWeight{1}},
        {0, 2, EdgeWeight{1}},
        {2, 3, EdgeWeight{1}},
        {3, 1, EdgeWeight{1}},
    };

    auto graph = makeGraph(mlp, edges);
    std::vector<bool> allowed_nodes(graph.GetNumberOfNodes(), true);

    CellStorage cell_storage(mlp, graph);
    auto temporal_metric = MakeTemporalCellMetric(cell_storage);
    TemporalFunctionStorage function_storage;
    function_storage.meta.bucket_size_minutes = 5;
    function_storage.meta.week_bucket_count = 4;
    function_storage.meta.encoding_version = TEMPORAL_FUNCTION_ENCODING_VERSION_DENSE;

    MockTemporalEvaluator evaluator;
    evaluator.durations[{0, 1}] = {5000, 1, 5000, 1};
    evaluator.durations[{1, 0}] = {5000, 1, 5000, 1};
    evaluator.durations[{0, 2}] = {50000, 50000, 50000, 50000};
    evaluator.durations[{2, 0}] = {50000, 50000, 50000, 50000};
    evaluator.durations[{2, 3}] = {1, 1, 1, 1};
    evaluator.durations[{3, 2}] = {1, 1, 1, 1};
    evaluator.durations[{3, 1}] = {1, 1, 1, 1};
    evaluator.durations[{1, 3}] = {1, 1, 1, 1};

    TemporalCellCustomizer customizer(mlp);
    TemporalCellCustomizer::Heap heap(graph.GetNumberOfNodes());
    customizer.Customize(
        graph, heap, cell_storage, allowed_nodes, evaluator, temporal_metric, function_storage, 1, 0);

    const auto metric_cell = temporal_metric.GetCell(cell_storage, 1, 0);
    const auto function_id = FindFunctionID(metric_cell, 0, 1);
    BOOST_REQUIRE_NE(function_id, INVALID_TEMPORAL_FUNCTION_ID);
    BOOST_CHECK(!function_storage.IsFIFOValid(function_id));
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(function_storage.GetDuration(function_id, 0)), 5000);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(function_storage.GetDuration(function_id, 1)), 1);
}

BOOST_AUTO_TEST_CASE(higher_level_temporal_cell_customization_uses_lower_level_shortcuts)
{
    std::vector<CellID> l1{{0, 0, 1, 1, 2, 2}};
    std::vector<CellID> l2{{0, 0, 0, 0, 1, 1}};
    std::vector<CellID> l3{{0, 0, 0, 0, 0, 0}};
    MultiLevelPartition mlp{{l1, l2, l3}, {3, 2, 1}};

    const std::vector<MockEdge> edges = {
        {0, 1, EdgeWeight{1}},
        {1, 2, EdgeWeight{1}},
        {2, 3, EdgeWeight{1}},
        {4, 5, EdgeWeight{1}},
        {0, 4, EdgeWeight{1}},
        {3, 4, EdgeWeight{1}},
    };

    auto graph = makeGraph(mlp, edges);
    std::vector<bool> allowed_nodes(graph.GetNumberOfNodes(), true);

    CellStorage cell_storage(mlp, graph);
    auto temporal_metric = MakeTemporalCellMetric(cell_storage);
    TemporalFunctionStorage function_storage;
    function_storage.meta.bucket_size_minutes = 5;
    function_storage.meta.week_bucket_count = 4;
    function_storage.meta.encoding_version = TEMPORAL_FUNCTION_ENCODING_VERSION_DCT;

    MockTemporalEvaluator evaluator;
    evaluator.durations[{0, 1}] = {1, 2, 3, 4};
    evaluator.durations[{1, 0}] = {1, 2, 3, 4};
    evaluator.durations[{1, 2}] = {10, 10, 10, 10};
    evaluator.durations[{2, 1}] = {10, 10, 10, 10};
    evaluator.durations[{2, 3}] = {2, 2, 2, 2};
    evaluator.durations[{3, 2}] = {2, 2, 2, 2};
    evaluator.durations[{4, 5}] = {1, 1, 1, 1};
    evaluator.durations[{5, 4}] = {1, 1, 1, 1};
    evaluator.durations[{0, 4}] = {100, 100, 100, 100};
    evaluator.durations[{4, 0}] = {100, 100, 100, 100};
    evaluator.durations[{3, 4}] = {100, 100, 100, 100};
    evaluator.durations[{4, 3}] = {100, 100, 100, 100};

    TemporalCellCustomizer customizer(mlp, 4);
    customizer.Customize(
        graph, cell_storage, allowed_nodes, evaluator, temporal_metric, function_storage);

    const auto metric_cell = temporal_metric.GetCell(cell_storage, 2, 0);
    const auto function_id = FindFunctionID(metric_cell, 0, 3);
    BOOST_REQUIRE_NE(function_id, INVALID_TEMPORAL_FUNCTION_ID);

    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(function_storage.GetDuration(function_id, 0)), 13);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(function_storage.GetDuration(function_id, 1)), 14);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(function_storage.GetDuration(function_id, 2)), 15);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(function_storage.GetDuration(function_id, 3)), 16);
}

BOOST_AUTO_TEST_CASE(level_one_temporal_cell_customization_marks_fallback_dct_profiles)
{
    std::vector<CellID> l1{{0, 0, 1, 1}};
    MultiLevelPartition mlp{{l1}, {2}};

    const std::vector<MockEdge> edges = {
        {0, 1, EdgeWeight{1}},
        {0, 2, EdgeWeight{1}},
        {2, 3, EdgeWeight{1}},
        {3, 1, EdgeWeight{1}},
    };

    auto graph = makeGraph(mlp, edges);
    std::vector<bool> allowed_nodes(graph.GetNumberOfNodes(), true);

    CellStorage cell_storage(mlp, graph);
    auto temporal_metric = MakeTemporalCellMetric(cell_storage);
    TemporalFunctionStorage function_storage;
    function_storage.meta.bucket_size_minutes = 5;
    function_storage.meta.week_bucket_count = 16;
    function_storage.meta.encoding_version = TEMPORAL_FUNCTION_ENCODING_VERSION_DCT;

    MockTemporalEvaluator evaluator;
    evaluator.week_bucket_count = 16;
    for (std::uint32_t bucket = 0; bucket < evaluator.week_bucket_count; ++bucket)
    {
        const auto sharp_value = ((bucket / 2) % 2 == 0) ? 45 : 240;
        evaluator.durations[{0, 1}].push_back(sharp_value);
        evaluator.durations[{1, 0}].push_back(sharp_value);
        evaluator.durations[{0, 2}].push_back(500);
        evaluator.durations[{2, 0}].push_back(500);
        evaluator.durations[{2, 3}].push_back(1);
        evaluator.durations[{3, 2}].push_back(1);
        evaluator.durations[{3, 1}].push_back(1);
        evaluator.durations[{1, 3}].push_back(1);
    }

    TemporalCellCustomizer customizer(mlp, 2);
    TemporalCellCustomizer::Heap heap(graph.GetNumberOfNodes());
    customizer.Customize(
        graph, heap, cell_storage, allowed_nodes, evaluator, temporal_metric, function_storage, 1, 0);

    const auto metric_cell = temporal_metric.GetCell(cell_storage, 1, 0);
    const auto function_id = FindFunctionID(metric_cell, 0, 1);
    BOOST_REQUIRE_NE(function_id, INVALID_TEMPORAL_FUNCTION_ID);
    BOOST_CHECK(function_storage.IsFIFOValid(function_id));
    BOOST_CHECK(function_storage.UsedFallbackCoeffCount(function_id));
    BOOST_CHECK_EQUAL(function_storage.profile_sizes[function_id], evaluator.week_bucket_count);
    BOOST_CHECK_SMALL(static_cast<double>(
                          std::abs(from_alias<std::int32_t>(function_storage.GetDuration(function_id, 0)) -
                                   evaluator.durations[{0, 1}][0])),
                      2.1);
    BOOST_CHECK_SMALL(static_cast<double>(
                          std::abs(from_alias<std::int32_t>(function_storage.GetDuration(function_id, 3)) -
                                   evaluator.durations[{0, 1}][3])),
                      2.1);
}

BOOST_AUTO_TEST_CASE(dense_temporal_function_cache_matches_dct_storage)
{
    std::vector<CellID> l1{{0, 0, 1, 1, 2, 2}};
    std::vector<CellID> l2{{0, 0, 0, 0, 1, 1}};
    std::vector<CellID> l3{{0, 0, 0, 0, 0, 0}};
    MultiLevelPartition mlp{{l1, l2, l3}, {3, 2, 1}};

    const std::vector<MockEdge> edges = {
        {0, 1, EdgeWeight{1}},
        {1, 2, EdgeWeight{1}},
        {2, 3, EdgeWeight{1}},
        {4, 5, EdgeWeight{1}},
        {0, 4, EdgeWeight{1}},
        {3, 4, EdgeWeight{1}},
    };

    auto graph = makeGraph(mlp, edges);
    std::vector<bool> allowed_nodes(graph.GetNumberOfNodes(), true);

    CellStorage cell_storage(mlp, graph);
    auto temporal_metric = MakeTemporalCellMetric(cell_storage);
    TemporalFunctionStorage function_storage;
    function_storage.meta.bucket_size_minutes = 5;
    function_storage.meta.week_bucket_count = 4;
    function_storage.meta.encoding_version = TEMPORAL_FUNCTION_ENCODING_VERSION_DCT;

    MockTemporalEvaluator evaluator;
    evaluator.durations[{0, 1}] = {1, 2, 3, 4};
    evaluator.durations[{1, 0}] = {1, 2, 3, 4};
    evaluator.durations[{1, 2}] = {10, 10, 10, 10};
    evaluator.durations[{2, 1}] = {10, 10, 10, 10};
    evaluator.durations[{2, 3}] = {2, 2, 2, 2};
    evaluator.durations[{3, 2}] = {2, 2, 2, 2};
    evaluator.durations[{4, 5}] = {1, 1, 1, 1};
    evaluator.durations[{5, 4}] = {1, 1, 1, 1};
    evaluator.durations[{0, 4}] = {100, 100, 100, 100};
    evaluator.durations[{4, 0}] = {100, 100, 100, 100};
    evaluator.durations[{3, 4}] = {100, 100, 100, 100};
    evaluator.durations[{4, 3}] = {100, 100, 100, 100};

    TemporalCellCustomizer customizer(mlp, 4);
    customizer.Customize(
        graph, cell_storage, allowed_nodes, evaluator, temporal_metric, function_storage);

    DenseTemporalFunctionCache dense_cache(function_storage);
    BOOST_REQUIRE_EQUAL(dense_cache.profile_count, function_storage.profile_offsets.size());
    BOOST_REQUIRE_EQUAL(dense_cache.meta.week_bucket_count, function_storage.meta.week_bucket_count);

    for (TemporalFunctionID function_id = 0; function_id < function_storage.profile_offsets.size();
         ++function_id)
    {
        for (std::uint32_t bucket = 0; bucket < function_storage.meta.week_bucket_count; ++bucket)
        {
            BOOST_CHECK_EQUAL(
                from_alias<std::int32_t>(dense_cache.GetDuration(function_id, bucket)),
                from_alias<std::int32_t>(function_storage.GetDuration(function_id, bucket)));
        }
    }
}

BOOST_AUTO_TEST_CASE(temporal_cell_customization_can_limit_selected_levels)
{
    std::vector<CellID> l1{{0, 0, 1, 1, 2, 2}};
    std::vector<CellID> l2{{0, 0, 0, 0, 1, 1}};
    std::vector<CellID> l3{{0, 0, 0, 0, 0, 0}};
    MultiLevelPartition mlp{{l1, l2, l3}, {3, 2, 1}};

    const std::vector<MockEdge> edges = {
        {0, 1, EdgeWeight{1}},
        {1, 2, EdgeWeight{1}},
        {2, 3, EdgeWeight{1}},
        {4, 5, EdgeWeight{1}},
        {0, 4, EdgeWeight{1}},
        {3, 4, EdgeWeight{1}},
    };

    auto graph = makeGraph(mlp, edges);
    std::vector<bool> allowed_nodes(graph.GetNumberOfNodes(), true);

    CellStorage cell_storage(mlp, graph);
    auto temporal_metric = MakeTemporalCellMetric(cell_storage);
    TemporalFunctionStorage function_storage;
    function_storage.meta.bucket_size_minutes = 5;
    function_storage.meta.week_bucket_count = 4;
    function_storage.meta.encoding_version = TEMPORAL_FUNCTION_ENCODING_VERSION_DENSE;

    MockTemporalEvaluator evaluator;
    evaluator.durations[{0, 1}] = {1, 2, 3, 4};
    evaluator.durations[{1, 0}] = {1, 2, 3, 4};
    evaluator.durations[{1, 2}] = {10, 10, 10, 10};
    evaluator.durations[{2, 1}] = {10, 10, 10, 10};
    evaluator.durations[{2, 3}] = {2, 2, 2, 2};
    evaluator.durations[{3, 2}] = {2, 2, 2, 2};
    evaluator.durations[{4, 5}] = {1, 1, 1, 1};
    evaluator.durations[{5, 4}] = {1, 1, 1, 1};
    evaluator.durations[{0, 4}] = {100, 100, 100, 100};
    evaluator.durations[{4, 0}] = {100, 100, 100, 100};
    evaluator.durations[{3, 4}] = {100, 100, 100, 100};
    evaluator.durations[{4, 3}] = {100, 100, 100, 100};

    TemporalCellCustomizer customizer(mlp, 4);
    customizer.Customize(graph,
                         cell_storage,
                         allowed_nodes,
                         evaluator,
                         temporal_metric,
                         function_storage,
                         LevelID{1},
                         LevelID{1});

    BOOST_CHECK_GT(function_storage.profile_offsets.size(), 0U);

    const auto level_two_cell = temporal_metric.GetCell(cell_storage, 2, 0);
    BOOST_CHECK_EQUAL(FindFunctionID(level_two_cell, 0, 3), INVALID_TEMPORAL_FUNCTION_ID);
}

BOOST_AUTO_TEST_CASE(parallel_temporal_cell_customization_matches_serial_reference_across_chunks)
{
    std::vector<CellID> l1;
    l1.reserve(80);
    std::vector<MockEdge> edges;
    for (CellID cell = 0; cell < 40; ++cell)
    {
        l1.push_back(cell);
        l1.push_back(cell);

        const auto a = static_cast<NodeID>(cell * 2);
        const auto b = static_cast<NodeID>(cell * 2 + 1);
        edges.push_back({a, b, EdgeWeight{1}});
        if (cell + 1 < 40)
        {
            edges.push_back({b, static_cast<NodeID>((cell + 1) * 2), EdgeWeight{1}});
        }
    }
    MultiLevelPartition mlp{{l1}, {40}};

    auto graph = makeGraph(mlp, edges);
    std::vector<bool> allowed_nodes(graph.GetNumberOfNodes(), true);

    CellStorage cell_storage(mlp, graph);
    auto serial_metric = MakeTemporalCellMetric(cell_storage);
    auto parallel_metric = MakeTemporalCellMetric(cell_storage);
    TemporalFunctionStorage serial_storage;
    TemporalFunctionStorage parallel_storage;
    serial_storage.meta.bucket_size_minutes = 5;
    serial_storage.meta.week_bucket_count = 4;
    serial_storage.meta.encoding_version = TEMPORAL_FUNCTION_ENCODING_VERSION_DENSE;
    parallel_storage.meta = serial_storage.meta;

    MockTemporalEvaluator evaluator;
    for (CellID cell = 0; cell < 40; ++cell)
    {
        const auto a = static_cast<NodeID>(cell * 2);
        const auto b = static_cast<NodeID>(cell * 2 + 1);
        const auto base = static_cast<EdgeDuration::value_type>(1 + (cell % 4));
        evaluator.durations[{a, b}] = {base, static_cast<EdgeDuration::value_type>(base + 1),
                                       static_cast<EdgeDuration::value_type>(base + 2),
                                       static_cast<EdgeDuration::value_type>(base + 3)};
        evaluator.durations[{b, a}] = evaluator.durations[{a, b}];

        if (cell + 1 < 40)
        {
            const auto next = static_cast<NodeID>((cell + 1) * 2);
            const auto jump = static_cast<EdgeDuration::value_type>(5 + (cell % 3));
            evaluator.durations[{b, next}] = {jump, jump, jump, jump};
            evaluator.durations[{next, b}] = evaluator.durations[{b, next}];
        }
    }

    TemporalCellCustomizer customizer(mlp);
    customizer.CustomizeSerial(
        graph, cell_storage, allowed_nodes, evaluator, serial_metric, serial_storage);
    customizer.Customize(
        graph, cell_storage, allowed_nodes, evaluator, parallel_metric, parallel_storage);

    CheckTemporalMetricEquivalent(serial_metric, parallel_metric);
    CheckTemporalStorageEquivalent(serial_storage, parallel_storage);
}

BOOST_AUTO_TEST_CASE(parallel_temporal_cell_customization_matches_serial_reference_with_dct_shortcuts)
{
    std::vector<CellID> l1{{0, 0, 1, 1, 2, 2}};
    std::vector<CellID> l2{{0, 0, 0, 0, 1, 1}};
    std::vector<CellID> l3{{0, 0, 0, 0, 0, 0}};
    MultiLevelPartition mlp{{l1, l2, l3}, {3, 2, 1}};

    const std::vector<MockEdge> edges = {
        {0, 1, EdgeWeight{1}},
        {1, 2, EdgeWeight{1}},
        {2, 3, EdgeWeight{1}},
        {4, 5, EdgeWeight{1}},
        {0, 4, EdgeWeight{1}},
        {3, 4, EdgeWeight{1}},
    };

    auto graph = makeGraph(mlp, edges);
    std::vector<bool> allowed_nodes(graph.GetNumberOfNodes(), true);

    CellStorage cell_storage(mlp, graph);
    auto serial_metric = MakeTemporalCellMetric(cell_storage);
    auto parallel_metric = MakeTemporalCellMetric(cell_storage);
    TemporalFunctionStorage serial_storage;
    TemporalFunctionStorage parallel_storage;
    serial_storage.meta.bucket_size_minutes = 5;
    serial_storage.meta.week_bucket_count = 4;
    serial_storage.meta.encoding_version = TEMPORAL_FUNCTION_ENCODING_VERSION_DCT;
    parallel_storage.meta = serial_storage.meta;

    MockTemporalEvaluator evaluator;
    evaluator.durations[{0, 1}] = {1, 2, 3, 4};
    evaluator.durations[{1, 0}] = {1, 2, 3, 4};
    evaluator.durations[{1, 2}] = {10, 10, 10, 10};
    evaluator.durations[{2, 1}] = {10, 10, 10, 10};
    evaluator.durations[{2, 3}] = {2, 2, 2, 2};
    evaluator.durations[{3, 2}] = {2, 2, 2, 2};
    evaluator.durations[{4, 5}] = {1, 1, 1, 1};
    evaluator.durations[{5, 4}] = {1, 1, 1, 1};
    evaluator.durations[{0, 4}] = {100, 100, 100, 100};
    evaluator.durations[{4, 0}] = {100, 100, 100, 100};
    evaluator.durations[{3, 4}] = {100, 100, 100, 100};
    evaluator.durations[{4, 3}] = {100, 100, 100, 100};

    TemporalCellCustomizer customizer(mlp, 4);
    customizer.CustomizeSerial(
        graph, cell_storage, allowed_nodes, evaluator, serial_metric, serial_storage);
    customizer.Customize(
        graph, cell_storage, allowed_nodes, evaluator, parallel_metric, parallel_storage);

    CheckTemporalMetricEquivalent(serial_metric, parallel_metric);
    CheckTemporalStorageEquivalent(serial_storage, parallel_storage);
}

BOOST_AUTO_TEST_SUITE_END()
