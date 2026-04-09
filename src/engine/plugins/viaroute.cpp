#include "engine/plugins/viaroute.hpp"
#include "engine/api/route_api.hpp"
#include "engine/routing_algorithms.hpp"
#include "engine/status.hpp"
#include "engine/temporal_traffic.hpp"

#include "util/for_each_pair.hpp"
#include "util/integer_range.hpp"

#include <cstdlib>

#include <algorithm>
#include <string>
#include <variant>
#include <vector>

namespace osrm::engine::plugins
{
namespace
{
util::json::Object
MakeTemporalAsymmetricDebug(const TemporalAsymmetricSearchDiagnostics &diagnostics)
{
    util::json::Object debug;
    debug.values.emplace("endpoint_pairs_tried",
                         util::json::Number{
                             static_cast<double>(diagnostics.endpoint_pairs_tried)});
    debug.values.emplace("endpoint_pairs_with_static_upper_bound",
                         util::json::Number{static_cast<double>(
                             diagnostics.endpoint_pairs_with_static_upper_bound)});
    debug.values.emplace("reverse_lower_bound_source_invalid",
                         util::json::Number{static_cast<double>(
                             diagnostics.reverse_lower_bound_source_invalid)});
    debug.values.emplace("queue_exhausted_without_target",
                         util::json::Number{static_cast<double>(
                             diagnostics.queue_exhausted_without_target)});
    debug.values.emplace("pruned_by_initial_upper_bound",
                         util::json::Number{static_cast<double>(
                             diagnostics.pruned_by_initial_upper_bound)});
    debug.values.emplace("pruned_by_best_upper_bound",
                         util::json::Number{static_cast<double>(
                             diagnostics.pruned_by_best_upper_bound)});
    debug.values.emplace("invalid_duration_relaxations",
                         util::json::Number{static_cast<double>(
                             diagnostics.invalid_duration_relaxations)});
    debug.values.emplace("target_reached",
                         util::json::Number{static_cast<double>(diagnostics.target_reached)});
    debug.values.emplace("best_candidate_rejected",
                         util::json::Number{
                             static_cast<double>(diagnostics.best_candidate_rejected)});
    debug.values.emplace("expanded_nodes",
                         util::json::Number{static_cast<double>(diagnostics.expanded_nodes)});
    debug.values.emplace("relaxation_attempts",
                         util::json::Number{
                             static_cast<double>(diagnostics.relaxation_attempts)});
    debug.values.emplace("relaxation_improvements",
                         util::json::Number{
                             static_cast<double>(diagnostics.relaxation_improvements)});
    debug.values.emplace("source_first_pop_pruned_by_initial_upper_bound",
                         util::json::Number{static_cast<double>(
                             diagnostics.source_first_pop_pruned_by_initial_upper_bound)});
    debug.values.emplace("min_initial_upper_bound_prune_margin",
                         util::json::Number{static_cast<double>(
                             diagnostics.MinInitialUpperBoundPruneMarginOrSentinel())});
    debug.values.emplace("max_initial_upper_bound_prune_margin",
                         util::json::Number{static_cast<double>(
                             diagnostics.MaxInitialUpperBoundPruneMarginOrSentinel())});
    return debug;
}

void AddTemporalAsymmetricDebugIfRequested(const api::RouteParameters &route_parameters,
                                           const InternalManyRoutesResult &routes,
                                           osrm::engine::api::ResultT &result)
{
    if (!route_parameters.temporal_debug)
    {
        return;
    }

    TemporalAsymmetricSearchDiagnostics merged;
    bool has_diagnostics = false;
    for (const auto &route : routes.routes)
    {
        if (!route.temporal_asymmetric_debug)
        {
            continue;
        }

        merged.Merge(*route.temporal_asymmetric_debug);
        has_diagnostics = true;
    }

    if (!has_diagnostics)
    {
        return;
    }

    if (auto *json_result = std::get_if<util::json::Object>(&result))
    {
        json_result->values.emplace("temporal_debug", MakeTemporalAsymmetricDebug(merged));
    }
}
} // namespace

ViaRoutePlugin::ViaRoutePlugin(int max_locations_viaroute,
                               int max_alternatives,
                               std::optional<double> default_radius)
    : BasePlugin(default_radius), max_locations_viaroute(max_locations_viaroute),
      max_alternatives(max_alternatives)
{
}

Status ViaRoutePlugin::HandleRequest(const RoutingAlgorithmsInterface &algorithms,
                                     const api::RouteParameters &route_parameters,
                                     osrm::engine::api::ResultT &result) const
{
    BOOST_ASSERT(route_parameters.IsValid());
    const auto use_temporal_asymmetric_routing =
        route_parameters.temporal_routing_mode ==
        api::RouteParameters::TemporalRoutingMode::Asymmetric;

    if (!use_temporal_asymmetric_routing && !algorithms.HasShortestPathSearch() &&
        route_parameters.coordinates.size() > 2)
    {
        return Error("NotImplemented",
                     "Shortest path search is not implemented for the chosen search algorithm. "
                     "Only two coordinates supported.",
                     result);
    }

    if (!use_temporal_asymmetric_routing && !algorithms.HasDirectShortestPathSearch() &&
        !algorithms.HasShortestPathSearch())
    {
        return Error(
            "NotImplemented",
            "Direct shortest path search is not implemented for the chosen search algorithm.",
            result);
    }

    if (max_locations_viaroute > 0 &&
        (static_cast<int>(route_parameters.coordinates.size()) > max_locations_viaroute))
    {
        return Error("TooBig",
                     "Number of entries " + std::to_string(route_parameters.coordinates.size()) +
                         " is higher than current maximum (" +
                         std::to_string(max_locations_viaroute) + ")",
                     result);
    }

    // Takes care of alternatives=n and alternatives=true
    if ((route_parameters.number_of_alternatives > static_cast<unsigned>(max_alternatives)) ||
        (route_parameters.alternatives && max_alternatives == 0))
    {
        return Error("TooBig",
                     "Requested number of alternatives is higher than current maximum (" +
                         std::to_string(max_alternatives) + ")",
                     result);
    }

    if (!CheckAllCoordinates(route_parameters.coordinates))
    {
        return Error("InvalidValue", "Invalid coordinate value.", result);
    }

    // Error: first and last points should be waypoints
    if (!route_parameters.waypoints.empty() &&
        (route_parameters.waypoints[0] != 0 ||
         route_parameters.waypoints.back() != (route_parameters.coordinates.size() - 1)))
    {
        return Error(
            "InvalidValue", "First and last coordinates must be specified as waypoints.", result);
    }

    if (!CheckAlgorithms(route_parameters, algorithms, result))
        return Status::Error;

    const auto &facade = algorithms.GetFacade();
    auto phantom_node_pairs = GetPhantomNodes(facade, route_parameters);
    if (phantom_node_pairs.size() != route_parameters.coordinates.size())
    {
        return Error("NoSegment",
                     MissingPhantomErrorMessage(phantom_node_pairs, route_parameters.coordinates),
                     result);
    }
    BOOST_ASSERT(phantom_node_pairs.size() == route_parameters.coordinates.size());

    auto snapped_phantoms = SnapPhantomNodes(std::move(phantom_node_pairs));

    api::RouteAPI route_api{facade, route_parameters};

    // TODO: in v6 we should remove the boolean and only keep the number parameter.
    // For now just force them to be in sync. and keep backwards compatibility.
    const auto wants_alternatives =
        (max_alternatives > 0) &&
        (route_parameters.alternatives || route_parameters.number_of_alternatives > 0);
    const auto number_of_alternatives = std::max(1u, route_parameters.number_of_alternatives);

    InternalManyRoutesResult routes;
    const auto use_temporal_candidate_reranking =
        !use_temporal_asymmetric_routing && route_parameters.departure_timestamp &&
        2 == snapped_phantoms.size() &&
        algorithms.HasAlternativePathSearch() && max_alternatives > 0;

    // Alternatives do not support vias, only direct s,t queries supported
    // See the implementation notes and high-level outline.
    // https://github.com/Project-OSRM/osrm-backend/issues/3905
    if (use_temporal_asymmetric_routing)
    {
        if (!route_parameters.departure_timestamp)
        {
            return Error("InvalidValue",
                         "temporal_mode=asymmetric requires depart_at.",
                         result);
        }

        if (2 != snapped_phantoms.size())
        {
            return Error("NotImplemented",
                         "temporal_mode=asymmetric currently supports only two coordinates.",
                         result);
        }

        if (wants_alternatives)
        {
            return Error("NotImplemented",
                         "temporal_mode=asymmetric does not support alternatives.",
                         result);
        }

        if (!algorithms.HasTemporalAsymmetricDirectShortestPathSearch())
        {
            return Error(
                "NotImplemented",
                "Temporal asymmetric routing is not implemented for the chosen search algorithm.",
                result);
        }

        routes = algorithms.TemporalAsymmetricDirectShortestPathSearch(
            {snapped_phantoms[0], snapped_phantoms[1]}, *route_parameters.departure_timestamp);
    }
    else if (use_temporal_candidate_reranking)
    {
        const auto temporal_candidate_count = wants_alternatives ? number_of_alternatives : 1u;
        routes = algorithms.AlternativePathSearch({snapped_phantoms[0], snapped_phantoms[1]},
                                                  temporal_candidate_count);
        temporal::RerankRoutesByTemporalDuration(facade,
                                                 routes,
                                                 *route_parameters.departure_timestamp);

        if (!wants_alternatives && routes.routes.size() > 1)
        {
            routes.routes.resize(1);
        }
    }
    else if (2 == snapped_phantoms.size() && algorithms.HasAlternativePathSearch() &&
             wants_alternatives)
    {
        routes = algorithms.AlternativePathSearch({snapped_phantoms[0], snapped_phantoms[1]},
                                                  number_of_alternatives);
    }
    else if (2 == snapped_phantoms.size() && algorithms.HasDirectShortestPathSearch())
    {
        routes = algorithms.DirectShortestPathSearch({snapped_phantoms[0], snapped_phantoms[1]});
    }
    else
    {
        routes =
            algorithms.ShortestPathSearch(snapped_phantoms, route_parameters.continue_straight);
    }

    // The post condition for all path searches is we have at least one route in our result.
    // This route might be invalid by means of INVALID_EDGE_WEIGHT as shortest path weight.
    BOOST_ASSERT(!routes.routes.empty());

    // we can only know this after the fact, different SCC ids still
    // allow for connection in one direction.

    if (routes.routes[0].is_valid())
    {
        auto collapse_legs = !route_parameters.waypoints.empty();
        if (collapse_legs)
        {
            std::vector<bool> waypoint_legs(route_parameters.coordinates.size(), false);
            std::for_each(route_parameters.waypoints.begin(),
                          route_parameters.waypoints.end(),
                          [&](const std::size_t waypoint_index)
                          {
                              BOOST_ASSERT(waypoint_index < waypoint_legs.size());
                              waypoint_legs[waypoint_index] = true;
                          });
            // First and last coordinates should always be waypoints
            // This gets validated earlier, but double-checking here, jic
            BOOST_ASSERT(waypoint_legs.front());
            BOOST_ASSERT(waypoint_legs.back());
            for (std::size_t i = 0; i < routes.routes.size(); i++)
            {
                routes.routes[i] = CollapseInternalRouteResult(routes.routes[i], waypoint_legs);
            }
        }

        route_api.MakeResponse(routes, snapped_phantoms, result);
    }
    else
    {
        const auto all_in_same_component =
            [](const std::vector<PhantomNodeCandidates> &waypoint_candidates)
        {
            return std::any_of(waypoint_candidates.front().begin(),
                               waypoint_candidates.front().end(),
                               // For each of the first possible phantoms, check if all other
                               // positions in the list have a phantom from the same component.
                               [&](const PhantomNode &phantom)
                               {
                                   const auto component_id = phantom.component.id;
                                   return std::all_of(
                                       std::next(waypoint_candidates.begin()),
                                       std::end(waypoint_candidates),
                                       [component_id](const PhantomNodeCandidates &candidates) {
                                           return candidatesHaveComponent(candidates, component_id);
                                       });
                               });
        };

        if (!all_in_same_component(snapped_phantoms))
        {
            const auto status = Error("NoRoute", "Impossible route between points", result);
            AddTemporalAsymmetricDebugIfRequested(route_parameters, routes, result);
            return status;
        }
        else
        {
            const auto status = Error("NoRoute", "No route found between points", result);
            AddTemporalAsymmetricDebugIfRequested(route_parameters, routes, result);
            return status;
        }
    }

    return Status::Ok;
}
} // namespace osrm::engine::plugins
