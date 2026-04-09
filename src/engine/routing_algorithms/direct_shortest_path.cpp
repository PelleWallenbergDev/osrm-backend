#include "engine/routing_algorithms/direct_shortest_path.hpp"
#include "engine/routing_algorithms/routing_base.hpp"
#include "engine/routing_algorithms/routing_base_ch.hpp"
#include "engine/routing_algorithms/routing_base_mld.hpp"
#include "engine/routing_algorithms/temporal_asymmetric_mld.hpp"

#include "util/for_each_pair.hpp"

namespace osrm::engine::routing_algorithms
{

/// This is a stripped down version of the general shortest path algorithm.
/// The general algorithm always computes two queries for each leg. This is only
/// necessary in case of vias, where the directions of the start node is constrained
/// by the previous route.
/// This variation is only an optimization for graphs with slow queries, for example
/// not fully contracted graphs.
template <>
InternalRouteResult directShortestPathSearch(SearchEngineData<ch::Algorithm> &engine_working_data,
                                             const DataFacade<ch::Algorithm> &facade,
                                             const PhantomEndpointCandidates &endpoint_candidates)
{
    engine_working_data.InitializeOrClearFirstThreadLocalStorage(facade.GetNumberOfNodes());
    auto &forward_heap = *engine_working_data.forward_heap_1;
    auto &reverse_heap = *engine_working_data.reverse_heap_1;
    forward_heap.Clear();
    reverse_heap.Clear();

    EdgeWeight weight = INVALID_EDGE_WEIGHT;
    std::vector<NodeID> packed_leg;
    insertNodesInHeaps(forward_heap, reverse_heap, endpoint_candidates);

    search(engine_working_data,
           facade,
           forward_heap,
           reverse_heap,
           weight,
           packed_leg,
           {},
           endpoint_candidates);

    std::vector<NodeID> unpacked_nodes;
    std::vector<EdgeID> unpacked_edges;

    if (!packed_leg.empty())
    {
        unpacked_nodes.reserve(packed_leg.size());
        unpacked_edges.reserve(packed_leg.size());
        unpacked_nodes.push_back(packed_leg.front());
        ch::unpackPath(facade,
                       packed_leg.begin(),
                       packed_leg.end(),
                       [&unpacked_nodes, &unpacked_edges](
                           [[maybe_unused]] NodeID first, NodeID second, const auto &edge_id)
                       {
                           BOOST_ASSERT(first == unpacked_nodes.back());
                           unpacked_nodes.push_back(second);
                           unpacked_edges.push_back(edge_id);
                       });
    }

    return extractRoute(facade, weight, endpoint_candidates, unpacked_nodes, unpacked_edges);
}

template <>
InternalRouteResult directShortestPathSearch(SearchEngineData<mld::Algorithm> &engine_working_data,
                                             const DataFacade<mld::Algorithm> &facade,
                                             const PhantomEndpointCandidates &endpoint_candidates)
{
    engine_working_data.InitializeOrClearFirstThreadLocalStorage(facade.GetNumberOfNodes(),
                                                                 facade.GetMaxBorderNodeID() + 1);
    auto &forward_heap = *engine_working_data.forward_heap_1;
    auto &reverse_heap = *engine_working_data.reverse_heap_1;
    insertNodesInHeaps(forward_heap, reverse_heap, endpoint_candidates);

    auto unpacked_path = mld::search(engine_working_data,
                                     facade,
                                     forward_heap,
                                     reverse_heap,
                                     {},
                                     INVALID_EDGE_WEIGHT,
                                     endpoint_candidates);

    return extractRoute(facade,
                        unpacked_path.weight,
                        endpoint_candidates,
                        unpacked_path.nodes,
                        unpacked_path.edges);
}

template <>
InternalRouteResult temporalAsymmetricDirectShortestPathSearch(
    SearchEngineData<mld::Algorithm> &engine_working_data,
    const DataFacade<mld::Algorithm> &facade,
    const PhantomEndpointCandidates &endpoint_candidates,
    std::time_t departure_timestamp)
{
    const auto source_endpoints =
        mld::temporal::EnumerateSourceEndpoints(endpoint_candidates.source_phantoms);
    const auto target_endpoints =
        mld::temporal::EnumerateTargetEndpoints(endpoint_candidates.target_phantoms);

    TemporalAsymmetricSearchDiagnostics diagnostics;

    if (source_endpoints.empty() || target_endpoints.empty())
    {
        InternalRouteResult route;
        route.temporal_asymmetric_debug = diagnostics;
        return route;
    }

    const auto incoming_edges = mld::temporal::BuildIncomingEdgeIndex(facade);

    mld::temporal::TemporalAsymmetricPath best_path;
    const PhantomNode *best_source_phantom = nullptr;
    const PhantomNode *best_target_phantom = nullptr;

    for (const auto &source : source_endpoints)
    {
        for (const auto &target : target_endpoints)
        {
            ++diagnostics.endpoint_pairs_tried;

            PhantomNodeCandidates source_candidates{*source.phantom};
            PhantomNodeCandidates target_candidates{*target.phantom};
            const PhantomEndpointCandidates specific_candidates{source_candidates, target_candidates};

            std::optional<EdgeDuration> initial_upper_bound;
            auto static_route =
                routing_algorithms::directShortestPathSearch(engine_working_data,
                                                             facade,
                                                             specific_candidates);
            if (static_route.is_valid())
            {
                const auto evaluation =
                    engine::temporal::EvaluateRoute(facade, static_route, departure_timestamp);
                const auto static_duration = static_route.duration();
                if (engine::temporal::detail::IsFiniteNonNegativeDuration(
                        evaluation.total_duration) &&
                    (!evaluation.used_temporal ||
                     engine::temporal::detail::IsPlausibleTemporalDuration(
                         evaluation.total_duration, static_duration)))
                {
                    initial_upper_bound = evaluation.total_duration;
                    ++diagnostics.endpoint_pairs_with_static_upper_bound;
                }
            }

            const auto candidate = mld::temporal::Search(facade,
                                                         source,
                                                         target,
                                                         departure_timestamp,
                                                         incoming_edges,
                                                         initial_upper_bound,
                                                         &diagnostics);

            if (!candidate.is_valid() ||
                (best_path.is_valid() && candidate.total_duration >= best_path.total_duration))
            {
                ++diagnostics.best_candidate_rejected;
                continue;
            }

            best_path = candidate;
            best_source_phantom = source.phantom;
            best_target_phantom = target.phantom;
        }
    }

    if (!best_path.is_valid() || best_source_phantom == nullptr || best_target_phantom == nullptr)
    {
        InternalRouteResult route;
        route.temporal_asymmetric_debug = diagnostics;
        return route;
    }

    std::vector<EdgeID> unpacked_edges;
    if (best_path.nodes.size() > 1)
    {
        unpacked_edges.reserve(best_path.nodes.size() - 1);
        util::for_each_pair(best_path.nodes.begin(),
                            best_path.nodes.end(),
                            [&facade, &unpacked_edges](const NodeID from, const NodeID to)
                            { unpacked_edges.push_back(facade.FindEdge(from, to)); });
    }

    PhantomNodeCandidates source_candidates{*best_source_phantom};
    PhantomNodeCandidates target_candidates{*best_target_phantom};
    const PhantomEndpointCandidates best_candidates{source_candidates, target_candidates};

    auto route = extractRoute(facade,
                              alias_cast<EdgeWeight>(best_path.total_duration),
                              best_candidates,
                              best_path.nodes,
                              unpacked_edges);
    route.temporal_asymmetric_debug = diagnostics;
    return route;
}

} // namespace osrm::engine::routing_algorithms
