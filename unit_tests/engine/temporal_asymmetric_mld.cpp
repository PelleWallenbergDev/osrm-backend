#include "engine/routing_algorithms/temporal_asymmetric_mld.hpp"
#include "util/integer_range.hpp"

#include <boost/test/unit_test.hpp>

#include <cstdint>
#include <ctime>
#include <map>
#include <vector>

BOOST_AUTO_TEST_SUITE(temporal_asymmetric_mld)

namespace
{
using namespace osrm;

struct SearchEdgeData
{
    NodeID turn_id = 0;
};

struct TemporalSearchEdge
{
    NodeID target = SPECIAL_NODEID;
    SearchEdgeData data;
    bool forward = true;
};

struct TemporalSearchFacade
{
    std::vector<std::vector<SegmentDuration>> static_forward_durations;
    std::map<std::pair<PackedGeometryID, std::uint32_t>, EdgeDuration> forward_temporal;
    std::vector<EdgeDuration> forward_temporal_min;
    std::vector<TurnPenalty> turn_penalties;
    std::vector<TemporalSearchEdge> edges;
    std::vector<EdgeID> edge_offsets;

    unsigned GetNumberOfNodes() const { return static_cast<unsigned>(static_forward_durations.size()); }

    auto GetAdjacentEdgeRange(const NodeID node) const
    {
        return util::range<EdgeID>(edge_offsets[node], edge_offsets[node + 1]);
    }

    bool IsForwardEdge(const EdgeID edge) const { return edges[edge].forward; }

    NodeID GetTarget(const EdgeID edge) const { return edges[edge].target; }

    const SearchEdgeData &GetEdgeData(const EdgeID edge) const { return edges[edge].data; }

    TurnPenalty GetDurationPenaltyForEdgeID(const NodeID turn_id) const
    {
        return turn_penalties[turn_id];
    }

    bool ExcludeNode(const NodeID) const { return false; }

    GeometryID GetGeometryIndex(const NodeID edge_based_node_id) const
    {
        return GeometryID{static_cast<PackedGeometryID>(edge_based_node_id), true};
    }

    const std::vector<SegmentDuration> &
    GetUncompressedForwardDurations(const PackedGeometryID id) const
    {
        return static_forward_durations[id];
    }

    const std::vector<SegmentDuration> &
    GetUncompressedReverseDurations(const PackedGeometryID id) const
    {
        return static_forward_durations[id];
    }

    std::uint32_t GetTemporalBucketSizeMinutes() const { return 15; }

    std::uint32_t GetTemporalWeekBucketCount() const { return 672; }

    bool HasTemporalForwardProfile(const PackedGeometryID id) const
    {
        return id < forward_temporal_min.size() &&
               forward_temporal_min[id] != INVALID_EDGE_DURATION;
    }

    bool HasTemporalReverseProfile(const PackedGeometryID) const { return false; }

    EdgeDuration GetTemporalForwardDuration(const PackedGeometryID id,
                                            const std::uint32_t week_bucket) const
    {
        const auto it = forward_temporal.find({id, week_bucket});
        return it == forward_temporal.end() ? INVALID_EDGE_DURATION : it->second;
    }

    EdgeDuration GetTemporalReverseDuration(const PackedGeometryID,
                                            const std::uint32_t) const
    {
        return INVALID_EDGE_DURATION;
    }

    EdgeDuration GetTemporalForwardMinDuration(const PackedGeometryID id) const
    {
        return forward_temporal_min[id];
    }

    EdgeDuration GetTemporalReverseMinDuration(const PackedGeometryID) const
    {
        return INVALID_EDGE_DURATION;
    }
};

engine::PhantomNode MakeTemporalPhantom(const NodeID node, const EdgeDuration duration)
{
    engine::PhantomNode phantom;
    phantom.forward_segment_id = SegmentID{node, true};
    phantom.forward_duration = duration;
    phantom.forward_weight = alias_cast<EdgeWeight>(duration);
    phantom.component = ComponentID{1, false};
    return phantom;
}

} // namespace

BOOST_AUTO_TEST_CASE(search_prefers_temporally_faster_path_over_statically_shorter_path)
{
    TemporalSearchFacade facade;
    facade.static_forward_durations = {{SegmentDuration{10}},
                                       {SegmentDuration{10}},
                                       {SegmentDuration{20}},
                                       {SegmentDuration{10}},
                                       {SegmentDuration{10}}};
    facade.forward_temporal = {{{1, 192}, EdgeDuration{100}},
                               {{2, 192}, EdgeDuration{20}}};
    facade.forward_temporal_min = {EdgeDuration{10},
                                   EdgeDuration{10},
                                   EdgeDuration{20},
                                   EdgeDuration{10},
                                   EdgeDuration{10}};
    facade.turn_penalties = {TurnPenalty{0}, TurnPenalty{0}, TurnPenalty{0}, TurnPenalty{0}};
    facade.edges = {{1, {0}, true}, {2, {1}, true}, {4, {2}, true}, {4, {3}, true}};
    facade.edge_offsets = {0, 2, 3, 4, 4, 4};

    const auto source_phantom = MakeTemporalPhantom(0, EdgeDuration{0});
    const auto target_phantom = MakeTemporalPhantom(4, EdgeDuration{10});

    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 4, false};

    const auto result = engine::routing_algorithms::mld::temporal::Search(
        facade, source, target, std::time_t{1735689600});

    BOOST_REQUIRE(result.is_valid());
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(result.total_duration), 40);
    BOOST_REQUIRE_EQUAL(result.nodes.size(), 3UL);
    BOOST_CHECK_EQUAL(result.nodes[0], 0);
    BOOST_CHECK_EQUAL(result.nodes[1], 2);
    BOOST_CHECK_EQUAL(result.nodes[2], 4);
}

BOOST_AUTO_TEST_CASE(search_handles_same_geometry_local_paths)
{
    TemporalSearchFacade facade;
    facade.static_forward_durations = {{SegmentDuration{100}}};
    facade.forward_temporal = {{{0, 192}, EdgeDuration{100}}};
    facade.forward_temporal_min = {EdgeDuration{100}};
    facade.turn_penalties = {};
    facade.edge_offsets = {0, 0};

    const auto source_phantom = MakeTemporalPhantom(0, EdgeDuration{20});
    const auto target_phantom = MakeTemporalPhantom(0, EdgeDuration{70});

    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 0, false};

    const auto result = engine::routing_algorithms::mld::temporal::Search(
        facade, source, target, std::time_t{1735689600});

    BOOST_REQUIRE(result.is_valid());
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(result.total_duration), 50);
    BOOST_REQUIRE_EQUAL(result.nodes.size(), 1UL);
    BOOST_CHECK_EQUAL(result.nodes[0], 0);
}

BOOST_AUTO_TEST_CASE(optimized_search_matches_plain_search_on_temporal_choice)
{
    TemporalSearchFacade facade;
    facade.static_forward_durations = {{SegmentDuration{10}},
                                       {SegmentDuration{10}},
                                       {SegmentDuration{20}},
                                       {SegmentDuration{10}},
                                       {SegmentDuration{10}}};
    facade.forward_temporal = {{{1, 192}, EdgeDuration{100}},
                               {{2, 192}, EdgeDuration{20}}};
    facade.forward_temporal_min = {EdgeDuration{10},
                                   EdgeDuration{10},
                                   EdgeDuration{20},
                                   EdgeDuration{10},
                                   EdgeDuration{10}};
    facade.turn_penalties = {TurnPenalty{0}, TurnPenalty{0}, TurnPenalty{0}, TurnPenalty{0}};
    facade.edges = {{1, {0}, true}, {2, {1}, true}, {4, {2}, true}, {4, {3}, true}};
    facade.edge_offsets = {0, 2, 3, 4, 4, 4};

    const auto source_phantom = MakeTemporalPhantom(0, EdgeDuration{0});
    const auto target_phantom = MakeTemporalPhantom(4, EdgeDuration{10});

    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 4, false};

    const auto plain = engine::routing_algorithms::mld::temporal::Search(
        facade, source, target, std::time_t{1735689600});
    const auto optimized = engine::routing_algorithms::mld::temporal::SearchOptimized(
        facade, source, target, std::time_t{1735689600});

    BOOST_REQUIRE(plain.is_valid());
    BOOST_REQUIRE(optimized.is_valid());
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(plain.total_duration),
                      from_alias<std::int32_t>(optimized.total_duration));
    BOOST_CHECK_EQUAL_COLLECTIONS(
        plain.nodes.begin(), plain.nodes.end(), optimized.nodes.begin(), optimized.nodes.end());
}

BOOST_AUTO_TEST_CASE(optimized_search_matches_plain_search_on_local_path)
{
    TemporalSearchFacade facade;
    facade.static_forward_durations = {{SegmentDuration{100}}};
    facade.forward_temporal = {{{0, 192}, EdgeDuration{100}}};
    facade.forward_temporal_min = {EdgeDuration{100}};
    facade.turn_penalties = {};
    facade.edge_offsets = {0, 0};

    const auto source_phantom = MakeTemporalPhantom(0, EdgeDuration{20});
    const auto target_phantom = MakeTemporalPhantom(0, EdgeDuration{70});

    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 0, false};

    const auto plain = engine::routing_algorithms::mld::temporal::Search(
        facade, source, target, std::time_t{1735689600});
    const auto optimized = engine::routing_algorithms::mld::temporal::SearchOptimized(
        facade, source, target, std::time_t{1735689600});

    BOOST_REQUIRE(plain.is_valid());
    BOOST_REQUIRE(optimized.is_valid());
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(plain.total_duration),
                      from_alias<std::int32_t>(optimized.total_duration));
    BOOST_CHECK_EQUAL_COLLECTIONS(
        plain.nodes.begin(), plain.nodes.end(), optimized.nodes.begin(), optimized.nodes.end());
}

BOOST_AUTO_TEST_CASE(search_keeps_equal_initial_upper_bound_as_valid_solution)
{
    TemporalSearchFacade facade;
    facade.static_forward_durations = {
        {SegmentDuration{10}}, {SegmentDuration{10}}, {SegmentDuration{10}}};
    facade.forward_temporal = {};
    facade.forward_temporal_min = {EdgeDuration{10}, EdgeDuration{10}, EdgeDuration{10}};
    facade.turn_penalties = {TurnPenalty{0}, TurnPenalty{0}};
    facade.edges = {{1, {0}, true}, {2, {1}, true}};
    facade.edge_offsets = {0, 1, 2, 2};

    const auto source_phantom = MakeTemporalPhantom(0, EdgeDuration{0});
    const auto target_phantom = MakeTemporalPhantom(2, EdgeDuration{10});

    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 2, false};

    const auto result = engine::routing_algorithms::mld::temporal::Search(
        facade, source, target, std::time_t{1735689600}, EdgeDuration{30});

    BOOST_REQUIRE(result.is_valid());
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(result.total_duration), 30);
    BOOST_REQUIRE_EQUAL(result.nodes.size(), 3UL);
    BOOST_CHECK_EQUAL(result.nodes[0], 0);
    BOOST_CHECK_EQUAL(result.nodes[1], 1);
    BOOST_CHECK_EQUAL(result.nodes[2], 2);
}

BOOST_AUTO_TEST_CASE(search_does_not_prune_against_initial_upper_bound_before_exact_candidate)
{
    TemporalSearchFacade facade;
    facade.static_forward_durations = {
        {SegmentDuration{10}}, {SegmentDuration{10}}, {SegmentDuration{10}}};
    facade.forward_temporal = {};
    facade.forward_temporal_min = {EdgeDuration{10}, EdgeDuration{10}, EdgeDuration{10}};
    facade.turn_penalties = {TurnPenalty{0}, TurnPenalty{0}};
    facade.edges = {{1, {0}, true}, {2, {1}, true}};
    facade.edge_offsets = {0, 1, 2, 2};

    const auto source_phantom = MakeTemporalPhantom(0, EdgeDuration{0});
    const auto target_phantom = MakeTemporalPhantom(2, EdgeDuration{10});

    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 2, false};

    const auto result = engine::routing_algorithms::mld::temporal::Search(
        facade, source, target, std::time_t{1735689600}, EdgeDuration{29});

    BOOST_REQUIRE(result.is_valid());
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(result.total_duration), 30);
    BOOST_REQUIRE_EQUAL(result.nodes.size(), 3UL);
    BOOST_CHECK_EQUAL(result.nodes[0], 0);
    BOOST_CHECK_EQUAL(result.nodes[1], 1);
    BOOST_CHECK_EQUAL(result.nodes[2], 2);
}

BOOST_AUTO_TEST_CASE(search_ignores_negative_initial_upper_bound)
{
    TemporalSearchFacade facade;
    facade.static_forward_durations = {
        {SegmentDuration{10}}, {SegmentDuration{10}}, {SegmentDuration{10}}};
    facade.forward_temporal = {};
    facade.forward_temporal_min = {EdgeDuration{10}, EdgeDuration{10}, EdgeDuration{10}};
    facade.turn_penalties = {TurnPenalty{0}, TurnPenalty{0}};
    facade.edges = {{1, {0}, true}, {2, {1}, true}};
    facade.edge_offsets = {0, 1, 2, 2};

    const auto source_phantom = MakeTemporalPhantom(0, EdgeDuration{0});
    const auto target_phantom = MakeTemporalPhantom(2, EdgeDuration{10});

    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 2, false};

    const auto result = engine::routing_algorithms::mld::temporal::Search(
        facade, source, target, std::time_t{1735689600}, EdgeDuration{-1});

    BOOST_REQUIRE(result.is_valid());
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(result.total_duration), 30);
    BOOST_REQUIRE_EQUAL(result.nodes.size(), 3UL);
    BOOST_CHECK_EQUAL(result.nodes[0], 0);
    BOOST_CHECK_EQUAL(result.nodes[1], 1);
    BOOST_CHECK_EQUAL(result.nodes[2], 2);
}

BOOST_AUTO_TEST_CASE(search_keeps_reverse_lower_bound_no_greater_than_static)
{
    TemporalSearchFacade facade;
    facade.static_forward_durations = {
        {SegmentDuration{10}}, {SegmentDuration{10}}, {SegmentDuration{10}}};
    facade.forward_temporal = {};
    facade.forward_temporal_min = {EdgeDuration{1000}, EdgeDuration{1000}, EdgeDuration{1000}};
    facade.turn_penalties = {TurnPenalty{0}, TurnPenalty{0}};
    facade.edges = {{1, {0}, true}, {2, {1}, true}};
    facade.edge_offsets = {0, 1, 2, 2};

    const auto source_phantom = MakeTemporalPhantom(0, EdgeDuration{0});
    const auto target_phantom = MakeTemporalPhantom(2, EdgeDuration{10});

    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 2, false};

    const auto result = engine::routing_algorithms::mld::temporal::Search(
        facade, source, target, std::time_t{1735689600}, EdgeDuration{30});

    BOOST_REQUIRE(result.is_valid());
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(result.total_duration), 30);
    BOOST_REQUIRE_EQUAL(result.nodes.size(), 3UL);
    BOOST_CHECK_EQUAL(result.nodes[0], 0);
    BOOST_CHECK_EQUAL(result.nodes[1], 1);
    BOOST_CHECK_EQUAL(result.nodes[2], 2);
}

BOOST_AUTO_TEST_SUITE_END()
