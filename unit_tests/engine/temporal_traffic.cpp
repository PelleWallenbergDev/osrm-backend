#include "engine/temporal_traffic.hpp"

#include <boost/test/unit_test.hpp>

#include <cstdint>
#include <map>
#include <vector>

BOOST_AUTO_TEST_SUITE(temporal_traffic)

namespace
{
using namespace osrm;

struct FakeTemporalFacade
{
    std::map<PackedGeometryID, std::vector<SegmentDuration>> static_forward_durations = {
        {7, {SegmentDuration{40}}},
        {9, {SegmentDuration{20}, SegmentDuration{30}}},
    };
    std::map<std::pair<PackedGeometryID, std::uint32_t>, EdgeDuration> forward_temporal = {
        {{7, 0}, EdgeDuration{50}},
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

engine::InternalRouteResult MakeRoute(const NodeID edge_based_node_id,
                                      const NodeID via_node,
                                      const EdgeDuration target_duration)
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
    endpoints.target_phantom.forward_duration = target_duration;
    endpoints.target_phantom.component = ComponentID{1, 0};
    route.leg_endpoints = {endpoints};

    route.unpacked_path_segments = {{engine::PathData{edge_based_node_id,
                                                      via_node,
                                                      EdgeWeight{60},
                                                      EdgeWeight{0},
                                                      EdgeDuration{60},
                                                      EdgeDuration{0},
                                                      DatasourceID{0},
                                                      std::nullopt}}};
    return route;
}
} // namespace

BOOST_AUTO_TEST_CASE(evaluate_geometry_path_falls_back_to_static_duration)
{
    FakeTemporalFacade facade;

    const std::vector<osrm::engine::temporal::TemporalGeometryStep> steps = {
        {7, true},
        {9, true},
    };

    const auto evaluation =
        osrm::engine::temporal::EvaluateGeometryPath(facade, steps, std::time_t{345600});

    BOOST_REQUIRE_EQUAL(evaluation.steps.size(), 2);
    BOOST_CHECK(evaluation.steps[0].used_temporal);
    BOOST_CHECK(!evaluation.steps[1].used_temporal);
    BOOST_CHECK_EQUAL(evaluation.steps[0].week_bucket, 0);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(evaluation.steps[0].duration), 50);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(evaluation.steps[1].duration), 50);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(evaluation.total_duration), 100);
    BOOST_CHECK_EQUAL(evaluation.arrival_timestamp, std::time_t{345610});
    BOOST_CHECK_EQUAL(evaluation.arrival_timestamp_ds,
                      osrm::engine::temporal::ToTemporalClock(std::time_t{345600}) + 100);
}

BOOST_AUTO_TEST_CASE(timestamp_to_week_bucket_advances_on_bucket_boundary)
{
    BOOST_CHECK_EQUAL(osrm::engine::temporal::TimestampToWeekBucket(std::time_t{345600}, 15), 0U);
    BOOST_CHECK_EQUAL(osrm::engine::temporal::TimestampToWeekBucket(std::time_t{346499}, 15), 0U);
    BOOST_CHECK_EQUAL(osrm::engine::temporal::TimestampToWeekBucket(std::time_t{346500}, 15), 1U);
}

BOOST_AUTO_TEST_CASE(evaluate_route_accumulates_temporal_leg_durations)
{
    FakeTemporalFacade facade;
    facade.static_forward_durations = {{7, {SegmentDuration{60}, SegmentDuration{40}}},
                                       {9, {SegmentDuration{60}, SegmentDuration{40}}}};
    facade.forward_temporal = {{{7, 0}, EdgeDuration{200}}, {{9, 0}, EdgeDuration{300}}};

    auto route = MakeRoute(7, 11, EdgeDuration{40});
    route.leg_endpoints.push_back(route.leg_endpoints.front());
    route.leg_endpoints.back().source_phantom.forward_segment_id = SegmentID{9, true};
    route.leg_endpoints.back().target_phantom.forward_segment_id = SegmentID{9, true};
    route.unpacked_path_segments.push_back(
        {engine::PathData{9,
                          21,
                          EdgeWeight{60},
                          EdgeWeight{0},
                          EdgeDuration{60},
                          EdgeDuration{0},
                          DatasourceID{0},
                          std::nullopt}});
    route.source_traversed_in_reverse.push_back(false);
    route.target_traversed_in_reverse.push_back(false);

    const auto evaluation = osrm::engine::temporal::EvaluateRoute(facade, route, std::time_t{345600});

    BOOST_CHECK(evaluation.used_temporal);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(evaluation.total_duration), 500);
    BOOST_CHECK_EQUAL(evaluation.arrival_timestamp, std::time_t{345650});
    BOOST_CHECK_EQUAL(evaluation.arrival_timestamp_ds,
                      osrm::engine::temporal::ToTemporalClock(std::time_t{345600}) + 500);
}

BOOST_AUTO_TEST_SUITE_END()
