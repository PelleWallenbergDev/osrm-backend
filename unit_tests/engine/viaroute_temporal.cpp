#include "engine/temporal_traffic.hpp"

#include <boost/test/unit_test.hpp>

#include <cstdint>
#include <ctime>
#include <map>
#include <vector>

BOOST_AUTO_TEST_SUITE(viaroute_temporal)

namespace
{
using namespace osrm;

struct FakeTemporalFacade
{
    std::map<PackedGeometryID, std::vector<SegmentDuration>> static_forward_durations = {
        {7, {SegmentDuration{60}, SegmentDuration{40}}},
        {8, {SegmentDuration{60}, SegmentDuration{40}}},
        {9, {SegmentDuration{60}, SegmentDuration{40}}},
    };
    std::map<std::pair<PackedGeometryID, std::uint32_t>, EdgeDuration> forward_temporal = {
        {{7, 192}, EdgeDuration{300}},
        {{8, 192}, EdgeDuration{200}},
    };

    auto GetUncompressedForwardDurations(const PackedGeometryID id) const
    {
        return static_forward_durations.at(id);
    }

    auto GetUncompressedReverseDurations(const PackedGeometryID id) const
    {
        return static_forward_durations.at(id);
    }

    GeometryID GetGeometryIndex(const NodeID edge_based_node_id) const
    {
        return GeometryID{static_cast<PackedGeometryID>(edge_based_node_id), true};
    }

    std::uint32_t GetTemporalBucketSizeMinutes() const { return 15; }

    std::uint32_t GetTemporalWeekBucketCount() const { return 672; }

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
};

engine::InternalRouteResult MakeRoute(const NodeID edge_based_node_id)
{
    engine::InternalRouteResult route;
    route.shortest_path_weight = EdgeWeight{100};
    route.source_traversed_in_reverse = {false};
    route.target_traversed_in_reverse = {false};

    engine::PhantomEndpoints endpoints;
    endpoints.source_phantom.forward_segment_id = SegmentID{edge_based_node_id, true};
    endpoints.source_phantom.forward_duration = EdgeDuration{0};
    endpoints.source_phantom.component = ComponentID{1, 0};
    endpoints.target_phantom.forward_segment_id = SegmentID{edge_based_node_id, true};
    endpoints.target_phantom.forward_duration = EdgeDuration{40};
    endpoints.target_phantom.component = ComponentID{1, 0};
    route.leg_endpoints = {endpoints};

    route.unpacked_path_segments = {{engine::PathData{edge_based_node_id,
                                                      edge_based_node_id + 100,
                                                      EdgeWeight{60},
                                                      EdgeWeight{0},
                                                      EdgeDuration{60},
                                                      EdgeDuration{0},
                                                      DatasourceID{0},
                                                      std::nullopt}}};
    return route;
}

NodeID GetRouteGeometryNode(const engine::InternalRouteResult &route)
{
    return route.leg_endpoints.front().source_phantom.forward_segment_id.id;
}
} // namespace

BOOST_AUTO_TEST_CASE(rerank_routes_prefers_temporally_faster_candidate)
{
    FakeTemporalFacade facade;
    engine::InternalManyRoutesResult routes{
        std::vector<engine::InternalRouteResult>{MakeRoute(7), MakeRoute(8)}};

    const auto reordered =
        engine::temporal::RerankRoutesByTemporalDuration(facade, routes, std::time_t{1735689600});

    BOOST_CHECK(reordered);
    BOOST_REQUIRE_EQUAL(routes.routes.size(), 2UL);
    BOOST_CHECK_EQUAL(GetRouteGeometryNode(routes.routes[0]), 8);
    BOOST_CHECK_EQUAL(GetRouteGeometryNode(routes.routes[1]), 7);
}

BOOST_AUTO_TEST_CASE(rerank_routes_keeps_static_order_when_temporal_data_is_missing)
{
    FakeTemporalFacade facade;
    facade.forward_temporal.clear();
    engine::InternalManyRoutesResult routes{
        std::vector<engine::InternalRouteResult>{MakeRoute(7), MakeRoute(8)}};

    const auto reordered =
        engine::temporal::RerankRoutesByTemporalDuration(facade, routes, std::time_t{1735689600});

    BOOST_CHECK(!reordered);
    BOOST_REQUIRE_EQUAL(routes.routes.size(), 2UL);
    BOOST_CHECK_EQUAL(GetRouteGeometryNode(routes.routes[0]), 7);
    BOOST_CHECK_EQUAL(GetRouteGeometryNode(routes.routes[1]), 8);
}

BOOST_AUTO_TEST_CASE(rerank_routes_keeps_invalid_candidates_after_valid_ones)
{
    FakeTemporalFacade facade;
    auto invalid_route = MakeRoute(9);
    invalid_route.shortest_path_weight = INVALID_EDGE_WEIGHT;

    engine::InternalManyRoutesResult routes{
        std::vector<engine::InternalRouteResult>{invalid_route, MakeRoute(8), MakeRoute(7)}};

    const auto reordered =
        engine::temporal::RerankRoutesByTemporalDuration(facade, routes, std::time_t{1735689600});

    BOOST_CHECK(reordered);
    BOOST_REQUIRE_EQUAL(routes.routes.size(), 3UL);
    BOOST_CHECK_EQUAL(GetRouteGeometryNode(routes.routes[0]), 8);
    BOOST_CHECK_EQUAL(GetRouteGeometryNode(routes.routes[1]), 7);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(routes.routes[2].shortest_path_weight),
                      from_alias<std::int32_t>(INVALID_EDGE_WEIGHT));
}

BOOST_AUTO_TEST_SUITE_END()
