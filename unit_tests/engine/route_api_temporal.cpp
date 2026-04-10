#include "engine/api/route_api.hpp"

#include "mocks/mock_datafacade.hpp"

#include <boost/test/unit_test.hpp>

#include <cstdint>
#include <ctime>
#include <string_view>
#include <vector>

BOOST_AUTO_TEST_SUITE(route_api_temporal)

namespace
{
using namespace osrm;

class TemporalRouteFacade final : public test::MockBaseDataFacade
{
  public:
    explicit TemporalRouteFacade(const bool has_temporal_profile_) : has_temporal_profile(has_temporal_profile_) {}

    util::Coordinate GetCoordinateOfNode(const NodeID node_based_node_id) const override
    {
        switch (node_based_node_id)
        {
        case 10:
            return {util::FloatLongitude{0.0}, util::FloatLatitude{0.0}};
        case 11:
            return {util::FloatLongitude{0.001}, util::FloatLatitude{0.0}};
        case 12:
            return {util::FloatLongitude{0.002}, util::FloatLatitude{0.0}};
        default:
            return {util::FloatLongitude{0.0}, util::FloatLatitude{0.0}};
        }
    }

    GeometryID GetGeometryIndex(const NodeID edge_based_node_id) const override
    {
        return edge_based_node_id == 1 ? GeometryID{7, true} : GeometryID{};
    }

    ComponentID GetComponentID(const NodeID) const override { return ComponentID{1, 0}; }

    NodeForwardRange GetUncompressedForwardGeometry(const PackedGeometryID) const override
    {
        return NodeForwardRange(geometry_nodes.cbegin(), geometry_nodes.cend());
    }

    NodeReverseRange GetUncompressedReverseGeometry(const PackedGeometryID id) const override
    {
        return NodeReverseRange(GetUncompressedForwardGeometry(id));
    }

    WeightForwardRange GetUncompressedForwardWeights(const PackedGeometryID) const override
    {
        return WeightForwardRange(forward_weights.cbegin(), forward_weights.cend());
    }

    WeightReverseRange GetUncompressedReverseWeights(const PackedGeometryID id) const override
    {
        return WeightReverseRange(GetUncompressedForwardWeights(id));
    }

    DurationForwardRange GetUncompressedForwardDurations(const PackedGeometryID) const override
    {
        return DurationForwardRange(forward_durations.cbegin(), forward_durations.cend());
    }

    DurationReverseRange GetUncompressedReverseDurations(const PackedGeometryID id) const override
    {
        return DurationReverseRange(GetUncompressedForwardDurations(id));
    }

    DatasourceForwardRange GetUncompressedForwardDatasources(const PackedGeometryID) const override
    {
        return DatasourceForwardRange(datasources.cbegin(), datasources.cend());
    }

    DatasourceReverseRange GetUncompressedReverseDatasources(const PackedGeometryID id) const override
    {
        return DatasourceReverseRange(GetUncompressedForwardDatasources(id));
    }

    extractor::TravelMode GetTravelMode(const NodeID) const override
    {
        return extractor::TRAVEL_MODE_DRIVING;
    }

    bool HasTemporalForwardProfile(const PackedGeometryID) const override
    {
        return has_temporal_profile;
    }

    bool HasTemporalReverseProfile(const PackedGeometryID) const override { return false; }

    std::uint32_t GetTemporalBucketSizeMinutes() const override { return 15; }

    std::uint32_t GetTemporalWeekBucketCount() const override { return 672; }

    EdgeDuration GetTemporalForwardDuration(const PackedGeometryID geometry_id,
                                            const std::uint32_t week_bucket) const override
    {
        if (has_temporal_profile && geometry_id == 7)
        {
            if (week_bucket == 192)
            {
                return EdgeDuration{200};
            }

            if (week_bucket == 193)
            {
                return EdgeDuration{300};
            }
        }

        return INVALID_EDGE_DURATION;
    }

    EdgeDuration GetTemporalReverseDuration(const PackedGeometryID,
                                            const std::uint32_t) const override
    {
        return INVALID_EDGE_DURATION;
    }

  private:
    static constexpr std::uint64_t PackFirstTwo22BitValues(const std::uint32_t first,
                                                           const std::uint32_t second)
    {
        return static_cast<std::uint64_t>(first) |
               (static_cast<std::uint64_t>(second) << SEGMENT_DURATION_BITS);
    }

    bool has_temporal_profile = false;

    const extractor::SegmentDataView::SegmentNodeVector geometry_nodes = []()
    {
        static NodeID geometry_node_data[] = {10, 11, 12};
        return extractor::SegmentDataView::SegmentNodeVector(geometry_node_data, 3);
    }();

    const extractor::SegmentDataView::SegmentWeightVector forward_weights = []()
    {
        static std::uint64_t forward_weight_words[] = {PackFirstTwo22BitValues(60, 40), 0};
        return extractor::SegmentDataView::SegmentWeightVector(
            util::vector_view<std::uint64_t>(forward_weight_words, 2), 2);
    }();

    const extractor::SegmentDataView::SegmentDurationVector forward_durations = []()
    {
        static std::uint64_t forward_duration_words[] = {PackFirstTwo22BitValues(60, 40), 0};
        return extractor::SegmentDataView::SegmentDurationVector(
            util::vector_view<std::uint64_t>(forward_duration_words, 2), 2);
    }();

    const util::vector_view<DatasourceID> datasources = []()
    {
        static DatasourceID datasource_data[] = {0, 0};
        return util::vector_view<DatasourceID>(datasource_data, 2);
    }();
};

class SubsecondTemporalRouteFacade final : public test::MockBaseDataFacade
{
  public:
    util::Coordinate GetCoordinateOfNode(const NodeID node_based_node_id) const override
    {
        switch (node_based_node_id)
        {
        case 10:
            return {util::FloatLongitude{0.0}, util::FloatLatitude{0.0}};
        case 11:
            return {util::FloatLongitude{0.001}, util::FloatLatitude{0.0}};
        case 12:
            return {util::FloatLongitude{0.002}, util::FloatLatitude{0.0}};
        default:
            return {util::FloatLongitude{0.0}, util::FloatLatitude{0.0}};
        }
    }

    GeometryID GetGeometryIndex(const NodeID edge_based_node_id) const override
    {
        return edge_based_node_id == 1 ? GeometryID{7, true} : GeometryID{};
    }

    ComponentID GetComponentID(const NodeID) const override { return ComponentID{1, 0}; }

    NodeForwardRange GetUncompressedForwardGeometry(const PackedGeometryID) const override
    {
        return NodeForwardRange(geometry_nodes.cbegin(), geometry_nodes.cend());
    }

    NodeReverseRange GetUncompressedReverseGeometry(const PackedGeometryID id) const override
    {
        return NodeReverseRange(GetUncompressedForwardGeometry(id));
    }

    WeightForwardRange GetUncompressedForwardWeights(const PackedGeometryID) const override
    {
        return WeightForwardRange(forward_weights.cbegin(), forward_weights.cend());
    }

    WeightReverseRange GetUncompressedReverseWeights(const PackedGeometryID id) const override
    {
        return WeightReverseRange(GetUncompressedForwardWeights(id));
    }

    DurationForwardRange GetUncompressedForwardDurations(const PackedGeometryID) const override
    {
        return DurationForwardRange(forward_durations.cbegin(), forward_durations.cend());
    }

    DurationReverseRange GetUncompressedReverseDurations(const PackedGeometryID id) const override
    {
        return DurationReverseRange(GetUncompressedForwardDurations(id));
    }

    DatasourceForwardRange GetUncompressedForwardDatasources(const PackedGeometryID) const override
    {
        return DatasourceForwardRange(datasources.cbegin(), datasources.cend());
    }

    DatasourceReverseRange GetUncompressedReverseDatasources(const PackedGeometryID id) const override
    {
        return DatasourceReverseRange(GetUncompressedForwardDatasources(id));
    }

    extractor::TravelMode GetTravelMode(const NodeID) const override
    {
        return extractor::TRAVEL_MODE_DRIVING;
    }

    bool HasTemporalForwardProfile(const PackedGeometryID id) const override { return id == 7; }

    bool HasTemporalReverseProfile(const PackedGeometryID) const override { return false; }

    std::uint32_t GetTemporalBucketSizeMinutes() const override { return 15; }

    std::uint32_t GetTemporalWeekBucketCount() const override { return 672; }

    EdgeDuration GetTemporalForwardDuration(const PackedGeometryID geometry_id,
                                            const std::uint32_t week_bucket) const override
    {
        if (geometry_id != 7)
        {
            return INVALID_EDGE_DURATION;
        }

        if (week_bucket == 192)
        {
            return EdgeDuration{6};
        }

        if (week_bucket == 193)
        {
            return EdgeDuration{50};
        }

        return INVALID_EDGE_DURATION;
    }

    EdgeDuration GetTemporalReverseDuration(const PackedGeometryID,
                                            const std::uint32_t) const override
    {
        return INVALID_EDGE_DURATION;
    }

  private:
    static constexpr std::uint64_t PackFirstTwo22BitValues(const std::uint32_t first,
                                                           const std::uint32_t second)
    {
        return static_cast<std::uint64_t>(first) |
               (static_cast<std::uint64_t>(second) << SEGMENT_DURATION_BITS);
    }

    const extractor::SegmentDataView::SegmentNodeVector geometry_nodes = []()
    {
        static NodeID geometry_node_data[] = {10, 11, 12};
        return extractor::SegmentDataView::SegmentNodeVector(geometry_node_data, 3);
    }();

    const extractor::SegmentDataView::SegmentWeightVector forward_weights = []()
    {
        static std::uint64_t forward_weight_words[] = {PackFirstTwo22BitValues(60, 40), 0};
        return extractor::SegmentDataView::SegmentWeightVector(
            util::vector_view<std::uint64_t>(forward_weight_words, 2), 2);
    }();

    const extractor::SegmentDataView::SegmentDurationVector forward_durations = []()
    {
        static std::uint64_t forward_duration_words[] = {PackFirstTwo22BitValues(60, 40), 0};
        return extractor::SegmentDataView::SegmentDurationVector(
            util::vector_view<std::uint64_t>(forward_duration_words, 2), 2);
    }();

    const util::vector_view<DatasourceID> datasources = []()
    {
        static DatasourceID datasource_data[] = {0, 0};
        return util::vector_view<DatasourceID>(datasource_data, 2);
    }();
};

class InspectableRouteAPI final : public engine::api::RouteAPI
{
  public:
    using engine::api::RouteAPI::MakeLegs;
    using engine::api::RouteAPI::MakeRoute;
    using engine::api::RouteAPI::RouteAPI;
};

engine::PhantomEndpoints MakePhantoms()
{
    engine::PhantomEndpoints endpoints;

    endpoints.source_phantom.forward_segment_id = SegmentID{1, true};
    endpoints.source_phantom.forward_weight = EdgeWeight{0};
    endpoints.source_phantom.forward_duration = EdgeDuration{0};
    endpoints.source_phantom.forward_distance = EdgeDistance{0};
    endpoints.source_phantom.location = {util::FloatLongitude{0.0}, util::FloatLatitude{0.0}};
    endpoints.source_phantom.input_location = endpoints.source_phantom.location;
    endpoints.source_phantom.component = ComponentID{1, 0};
    endpoints.source_phantom.fwd_segment_position = 0;

    endpoints.target_phantom.forward_segment_id = SegmentID{1, true};
    endpoints.target_phantom.forward_weight = EdgeWeight{40};
    endpoints.target_phantom.forward_duration = EdgeDuration{40};
    endpoints.target_phantom.forward_distance = EdgeDistance{0};
    endpoints.target_phantom.location = {util::FloatLongitude{0.002}, util::FloatLatitude{0.0}};
    endpoints.target_phantom.input_location = endpoints.target_phantom.location;
    endpoints.target_phantom.component = ComponentID{1, 0};
    endpoints.target_phantom.fwd_segment_position = 1;

    return endpoints;
}

engine::InternalManyRoutesResult MakeRouteResult()
{
    engine::InternalRouteResult route;
    route.shortest_path_weight = EdgeWeight{100};
    route.leg_endpoints = {MakePhantoms()};
    route.source_traversed_in_reverse = {false};
    route.target_traversed_in_reverse = {false};
    route.unpacked_path_segments = {{engine::PathData{1,
                                                      11,
                                                      EdgeWeight{60},
                                                      EdgeWeight{0},
                                                      EdgeDuration{60},
                                                      EdgeDuration{0},
                                                      DatasourceID{0},
                                                      std::nullopt}}};
    return engine::InternalManyRoutesResult{std::move(route)};
}

engine::InternalManyRoutesResult MakeTwoLegRouteResult()
{
    engine::InternalRouteResult route;
    route.shortest_path_weight = EdgeWeight{200};
    route.leg_endpoints = {MakePhantoms(), MakePhantoms()};
    route.source_traversed_in_reverse = {false, false};
    route.target_traversed_in_reverse = {false, false};
    route.unpacked_path_segments = {
        {engine::PathData{1,
                          11,
                          EdgeWeight{60},
                          EdgeWeight{0},
                          EdgeDuration{60},
                          EdgeDuration{0},
                          DatasourceID{0},
                          std::nullopt}},
        {engine::PathData{1,
                          11,
                          EdgeWeight{60},
                          EdgeWeight{0},
                          EdgeDuration{60},
                          EdgeDuration{0},
                          DatasourceID{0},
                          std::nullopt}}};
    return engine::InternalManyRoutesResult{std::move(route)};
}

engine::InternalManyRoutesResult MakeThreeLegRouteResult()
{
    engine::InternalRouteResult route;
    route.shortest_path_weight = EdgeWeight{300};
    route.leg_endpoints = {MakePhantoms(), MakePhantoms(), MakePhantoms()};
    route.source_traversed_in_reverse = {false, false, false};
    route.target_traversed_in_reverse = {false, false, false};
    route.unpacked_path_segments = {
        {engine::PathData{1,
                          11,
                          EdgeWeight{60},
                          EdgeWeight{0},
                          EdgeDuration{60},
                          EdgeDuration{0},
                          DatasourceID{0},
                          std::nullopt}},
        {engine::PathData{1,
                          11,
                          EdgeWeight{60},
                          EdgeWeight{0},
                          EdgeDuration{60},
                          EdgeDuration{0},
                          DatasourceID{0},
                          std::nullopt}},
        {engine::PathData{1,
                          11,
                          EdgeWeight{60},
                          EdgeWeight{0},
                          EdgeDuration{60},
                          EdgeDuration{0},
                          DatasourceID{0},
                          std::nullopt}}};
    return engine::InternalManyRoutesResult{std::move(route)};
}

double ExtractRouteDuration(const util::json::Object &response)
{
    const auto &routes = std::get<util::json::Array>(response.values.at("routes"));
    const auto &route = std::get<util::json::Object>(routes.values.front());
    return std::get<util::json::Number>(route.values.at("duration")).value;
}

double ExtractLegDuration(const util::json::Object &response, const std::size_t leg_index = 0)
{
    const auto &routes = std::get<util::json::Array>(response.values.at("routes"));
    const auto &route = std::get<util::json::Object>(routes.values.front());
    const auto &legs = std::get<util::json::Array>(route.values.at("legs"));
    const auto &leg = std::get<util::json::Object>(legs.values.at(leg_index));
    return std::get<util::json::Number>(leg.values.at("duration")).value;
}

std::vector<double> ExtractAnnotationDurations(const util::json::Object &response)
{
    const auto &routes = std::get<util::json::Array>(response.values.at("routes"));
    const auto &route = std::get<util::json::Object>(routes.values.front());
    const auto &legs = std::get<util::json::Array>(route.values.at("legs"));
    const auto &leg = std::get<util::json::Object>(legs.values.front());
    const auto &annotation = std::get<util::json::Object>(leg.values.at("annotation"));
    const auto &durations = std::get<util::json::Array>(annotation.values.at("duration"));

    std::vector<double> result;
    result.reserve(durations.values.size());

    for (const auto &duration : durations.values)
    {
        result.push_back(std::get<util::json::Number>(duration).value);
    }

    return result;
}

double ExtractTemporalDebugNumber(const util::json::Object &response, const std::string_view key)
{
    const auto &debug = std::get<util::json::Object>(response.values.at("temporal_debug"));
    return std::get<util::json::Number>(debug.values.at(key)).value;
}

engine::api::RouteParameters MakeParameters()
{
    engine::api::RouteParameters parameters;
    parameters.coordinates = {{util::FloatLongitude{0.0}, util::FloatLatitude{0.0}},
                              {util::FloatLongitude{0.002}, util::FloatLatitude{0.0}}};
    parameters.overview = engine::api::RouteParameters::OverviewType::False;
    parameters.skip_waypoints = true;
    return parameters;
}
} // namespace

BOOST_AUTO_TEST_CASE(make_response_uses_temporal_duration_when_departure_time_is_set)
{
    TemporalRouteFacade facade{true};
    auto parameters = MakeParameters();
    parameters.departure_timestamp = std::time_t{1735689600};

    InspectableRouteAPI route_api{facade, parameters};
    const auto route_result = MakeRouteResult();

    const auto temporal_leg =
        engine::temporal::EvaluateRouteLeg(facade,
                                           route_result.routes.front().unpacked_path_segments.front(),
                                           route_result.routes.front().leg_endpoints.front().source_phantom,
                                           route_result.routes.front().leg_endpoints.front().target_phantom,
                                           route_result.routes.front().source_traversed_in_reverse.front(),
                                           route_result.routes.front().target_traversed_in_reverse.front(),
                                           *parameters.departure_timestamp);

    BOOST_REQUIRE_EQUAL(temporal_leg.path_data.size(), 1UL);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(temporal_leg.path_data.front().duration_until_turn),
                      120);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(temporal_leg.target_phantom.forward_duration), 80);
    BOOST_CHECK_EQUAL(temporal_leg.arrival_timestamp, std::time_t{1735689620});

    const auto legs_info = route_api.MakeLegs(route_result.routes.front().leg_endpoints,
                                              route_result.routes.front().unpacked_path_segments,
                                              route_result.routes.front().source_traversed_in_reverse,
                                              route_result.routes.front().target_traversed_in_reverse);
    BOOST_REQUIRE_EQUAL(legs_info.first.size(), 1UL);
    BOOST_CHECK_EQUAL(legs_info.first.front().duration, 20.0);

    engine::api::ResultT response = util::json::Object{};
    route_api.MakeResponse(route_result, {}, response);

    const auto &json = std::get<util::json::Object>(response);
    BOOST_CHECK_EQUAL(ExtractRouteDuration(json), 20.0);
    BOOST_CHECK_EQUAL(ExtractLegDuration(json), 20.0);
}

BOOST_AUTO_TEST_CASE(make_response_temporalizes_leg_annotations_when_requested)
{
    TemporalRouteFacade facade{true};
    auto parameters = MakeParameters();
    parameters.annotations_type = engine::api::RouteParameters::AnnotationsType::Duration;
    parameters.annotations = true;
    parameters.departure_timestamp = std::time_t{1735689600};

    engine::api::RouteAPI route_api{facade, parameters};
    engine::api::ResultT response = util::json::Object{};
    route_api.MakeResponse(MakeRouteResult(), {}, response);

    const auto annotation_durations =
        ExtractAnnotationDurations(std::get<util::json::Object>(response));
    BOOST_REQUIRE_EQUAL(annotation_durations.size(), 2UL);
    BOOST_CHECK_EQUAL(annotation_durations[0], 12.0);
    BOOST_CHECK_EQUAL(annotation_durations[1], 8.0);
}

BOOST_AUTO_TEST_CASE(make_response_advances_departure_time_between_legs)
{
    TemporalRouteFacade facade{true};
    auto parameters = MakeParameters();
    parameters.departure_timestamp = std::time_t{1735690499};

    InspectableRouteAPI route_api{facade, parameters};
    const auto route_result = MakeTwoLegRouteResult();
    const auto legs_info = route_api.MakeLegs(route_result.routes.front().leg_endpoints,
                                              route_result.routes.front().unpacked_path_segments,
                                              route_result.routes.front().source_traversed_in_reverse,
                                              route_result.routes.front().target_traversed_in_reverse);
    BOOST_REQUIRE_EQUAL(legs_info.first.size(), 2UL);
    BOOST_CHECK_EQUAL(legs_info.first[0].duration, 20.0);
    BOOST_CHECK_EQUAL(legs_info.first[1].duration, 30.0);

    engine::api::ResultT response = util::json::Object{};
    route_api.MakeResponse(route_result, {}, response);

    const auto &json = std::get<util::json::Object>(response);
    BOOST_CHECK_EQUAL(ExtractRouteDuration(json), 50.0);
    BOOST_CHECK_EQUAL(ExtractLegDuration(json, 0), 20.0);
    BOOST_CHECK_EQUAL(ExtractLegDuration(json, 1), 30.0);
}

BOOST_AUTO_TEST_CASE(make_response_keeps_decisecond_precision_across_multiple_legs)
{
    SubsecondTemporalRouteFacade facade;
    auto parameters = MakeParameters();
    parameters.departure_timestamp = std::time_t{1735690499};

    InspectableRouteAPI route_api{facade, parameters};
    const auto route_result = MakeThreeLegRouteResult();
    const auto legs_info = route_api.MakeLegs(route_result.routes.front().leg_endpoints,
                                              route_result.routes.front().unpacked_path_segments,
                                              route_result.routes.front().source_traversed_in_reverse,
                                              route_result.routes.front().target_traversed_in_reverse);
    BOOST_REQUIRE_EQUAL(legs_info.first.size(), 3UL);
    BOOST_CHECK_EQUAL(legs_info.first[0].duration, 0.6);
    BOOST_CHECK_EQUAL(legs_info.first[1].duration, 0.6);
    BOOST_CHECK_EQUAL(legs_info.first[2].duration, 5.0);

    engine::api::ResultT response = util::json::Object{};
    route_api.MakeResponse(route_result, {}, response);

    const auto &json = std::get<util::json::Object>(response);
    BOOST_CHECK_EQUAL(ExtractRouteDuration(json), 6.2);
    BOOST_CHECK_EQUAL(ExtractLegDuration(json, 0), 0.6);
    BOOST_CHECK_EQUAL(ExtractLegDuration(json, 1), 0.6);
    BOOST_CHECK_EQUAL(ExtractLegDuration(json, 2), 5.0);
}

BOOST_AUTO_TEST_CASE(make_response_falls_back_to_static_duration_without_temporal_profile)
{
    TemporalRouteFacade facade{false};
    auto parameters = MakeParameters();
    parameters.departure_timestamp = std::time_t{1735689600};

    engine::api::RouteAPI route_api{facade, parameters};
    engine::api::ResultT response = util::json::Object{};
    route_api.MakeResponse(MakeRouteResult(), {}, response);

    const auto &json = std::get<util::json::Object>(response);
    BOOST_CHECK_EQUAL(ExtractRouteDuration(json), 10.0);
    BOOST_CHECK_EQUAL(ExtractLegDuration(json), 10.0);
}

BOOST_AUTO_TEST_CASE(make_response_stays_static_without_departure_time)
{
    TemporalRouteFacade facade{true};
    auto parameters = MakeParameters();

    engine::api::RouteAPI route_api{facade, parameters};
    engine::api::ResultT response = util::json::Object{};
    route_api.MakeResponse(MakeRouteResult(), {}, response);

    const auto &json = std::get<util::json::Object>(response);
    BOOST_CHECK_EQUAL(ExtractRouteDuration(json), 10.0);
    BOOST_CHECK_EQUAL(ExtractLegDuration(json), 10.0);
}

BOOST_AUTO_TEST_CASE(make_response_includes_temporal_debug_when_requested)
{
    TemporalRouteFacade facade{false};
    auto parameters = MakeParameters();
    parameters.temporal_debug = true;

    auto route_result = MakeRouteResult();
    engine::TemporalAsymmetricSearchDiagnostics diagnostics;
    diagnostics.endpoint_pairs_tried = 4;
    diagnostics.endpoint_pairs_with_static_upper_bound = 3;
    diagnostics.static_upper_bound_pair_match_count = 2;
    diagnostics.static_upper_bound_source_mismatch_count = 1;
    diagnostics.static_upper_bound_target_mismatch_count = 1;
    diagnostics.static_upper_bound_direction_mismatch_count = 1;
    diagnostics.static_upper_bound_route_endpoint_unavailable_count = 0;
    diagnostics.first_static_upper_bound_mismatch_directed_source_node = 101;
    diagnostics.first_static_upper_bound_mismatch_directed_target_node = 202;
    diagnostics.first_static_upper_bound_mismatch_route_source_node = 303;
    diagnostics.first_static_upper_bound_mismatch_route_target_node = 404;
    diagnostics.reverse_lower_bound_source_invalid = 2;
    diagnostics.queue_exhausted_without_target = 1;
    diagnostics.expanded_nodes = 11;
    diagnostics.relaxation_attempts = 22;
    diagnostics.relaxation_improvements = 7;
    diagnostics.source_first_pop_pruned_by_initial_upper_bound = 5;
    diagnostics.RecordInitialUpperBoundPruneMargin(3);
    diagnostics.RecordInitialUpperBoundPruneMargin(9);
    route_result.routes.front().temporal_asymmetric_debug = diagnostics;

    engine::api::RouteAPI route_api{facade, parameters};
    engine::api::ResultT response = util::json::Object{};
    route_api.MakeResponse(route_result, {}, response);

    const auto &json = std::get<util::json::Object>(response);
    BOOST_CHECK_EQUAL(ExtractTemporalDebugNumber(json, "endpoint_pairs_tried"), 4.0);
    BOOST_CHECK_EQUAL(ExtractTemporalDebugNumber(json, "endpoint_pairs_with_static_upper_bound"),
                      3.0);
    BOOST_CHECK_EQUAL(ExtractTemporalDebugNumber(json, "static_upper_bound_pair_match_count"), 2.0);
    BOOST_CHECK_EQUAL(ExtractTemporalDebugNumber(json, "static_upper_bound_source_mismatch_count"),
                      1.0);
    BOOST_CHECK_EQUAL(ExtractTemporalDebugNumber(json, "static_upper_bound_target_mismatch_count"),
                      1.0);
    BOOST_CHECK_EQUAL(
        ExtractTemporalDebugNumber(json, "static_upper_bound_direction_mismatch_count"), 1.0);
    BOOST_CHECK_EQUAL(
        ExtractTemporalDebugNumber(json, "static_upper_bound_route_endpoint_unavailable_count"),
        0.0);
    BOOST_CHECK_EQUAL(
        ExtractTemporalDebugNumber(json, "first_static_upper_bound_mismatch_directed_source_node"),
        101.0);
    BOOST_CHECK_EQUAL(
        ExtractTemporalDebugNumber(json, "first_static_upper_bound_mismatch_directed_target_node"),
        202.0);
    BOOST_CHECK_EQUAL(
        ExtractTemporalDebugNumber(json, "first_static_upper_bound_mismatch_route_source_node"),
        303.0);
    BOOST_CHECK_EQUAL(
        ExtractTemporalDebugNumber(json, "first_static_upper_bound_mismatch_route_target_node"),
        404.0);
    BOOST_CHECK_EQUAL(ExtractTemporalDebugNumber(json, "reverse_lower_bound_source_invalid"), 2.0);
    BOOST_CHECK_EQUAL(ExtractTemporalDebugNumber(json, "queue_exhausted_without_target"), 1.0);
    BOOST_CHECK_EQUAL(ExtractTemporalDebugNumber(json, "expanded_nodes"), 11.0);
    BOOST_CHECK_EQUAL(ExtractTemporalDebugNumber(json, "relaxation_attempts"), 22.0);
    BOOST_CHECK_EQUAL(ExtractTemporalDebugNumber(json, "relaxation_improvements"), 7.0);
    BOOST_CHECK_EQUAL(
        ExtractTemporalDebugNumber(json, "source_first_pop_pruned_by_initial_upper_bound"), 5.0);
    BOOST_CHECK_EQUAL(ExtractTemporalDebugNumber(json, "min_initial_upper_bound_prune_margin"),
                      3.0);
    BOOST_CHECK_EQUAL(ExtractTemporalDebugNumber(json, "max_initial_upper_bound_prune_margin"),
                      9.0);
}

BOOST_AUTO_TEST_SUITE_END()
