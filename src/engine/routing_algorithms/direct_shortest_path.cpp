#include "engine/routing_algorithms/direct_shortest_path.hpp"
#include "engine/routing_algorithms/routing_base.hpp"
#include "engine/routing_algorithms/routing_base_ch.hpp"
#include "engine/routing_algorithms/routing_base_mld.hpp"
#include "engine/routing_algorithms/routing_base_td_mld.hpp"
#include "engine/routing_algorithms/temporal_asymmetric_mld.hpp"

#include "util/for_each_pair.hpp"
#include "util/log.hpp"

#include <chrono>
#include <optional>

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

    if (source_endpoints.empty() || target_endpoints.empty())
    {
        return {};
    }

    const auto incoming_edges = mld::temporal::BuildIncomingEdgeIndex(facade);

    mld::temporal::TemporalAsymmetricPath best_path;
    const PhantomNode *best_source_phantom = nullptr;
    const PhantomNode *best_target_phantom = nullptr;

    for (const auto &source : source_endpoints)
    {
        for (const auto &target : target_endpoints)
        {
            PhantomNodeCandidates source_candidates{*source.phantom};
            PhantomNodeCandidates target_candidates{*target.phantom};
            const PhantomEndpointCandidates specific_candidates{source_candidates, target_candidates};

            const auto candidate = mld::temporal::Search(facade,
                                                         source,
                                                         target,
                                                         departure_timestamp,
                                                         incoming_edges);

            if (!candidate.is_valid() ||
                (best_path.is_valid() && candidate.total_duration >= best_path.total_duration))
            {
                continue;
            }

            best_path = candidate;
            best_source_phantom = source.phantom;
            best_target_phantom = target.phantom;
        }
    }

    if (!best_path.is_valid() || best_source_phantom == nullptr || best_target_phantom == nullptr)
    {
        return {};
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

    return extractRoute(facade,
                        alias_cast<EdgeWeight>(best_path.total_duration),
                        best_candidates,
                        best_path.nodes,
                        unpacked_edges);
}

template <>
InternalRouteResult temporalOverlayDirectShortestPathSearch(
    SearchEngineData<mld::Algorithm> &engine_working_data,
    const DataFacade<mld::Algorithm> &facade,
    const PhantomEndpointCandidates &endpoint_candidates,
    std::time_t departure_timestamp)
{
    (void)engine_working_data;
    const auto overlay_timing_enabled = [&]()
    {
        const auto &policy = util::LogPolicy::GetInstance();
        return !policy.IsMute() && policy.GetLevel() >= logDEBUG;
    }();
    const auto query_start = std::chrono::steady_clock::now();

    const auto endpoint_enumeration_start = std::chrono::steady_clock::now();
    const auto source_endpoints =
        mld::temporal::EnumerateSourceEndpoints(endpoint_candidates.source_phantoms);
    const auto target_endpoints =
        mld::temporal::EnumerateTargetEndpoints(endpoint_candidates.target_phantoms);
    const auto endpoint_enumeration_stop = std::chrono::steady_clock::now();

    if (source_endpoints.empty() || target_endpoints.empty())
    {
        if (overlay_timing_enabled)
        {
            util::Log(logDEBUG)
                << "[overlay query] direct_shortest_path ms="
                << mld::temporal::overlay::detail::OverlayDurationMs(
                       std::chrono::steady_clock::now() - query_start)
                << " endpoint_ms="
                << mld::temporal::overlay::detail::OverlayDurationMs(
                       endpoint_enumeration_stop - endpoint_enumeration_start)
                << " shared_corridor_ms=0 shared_reverse_ms=0 pair_search_ms=0 unpack_ms=0"
                << " source_endpoints=" << source_endpoints.size()
                << " target_endpoints=" << target_endpoints.size()
                << " candidate_pairs=0 valid_pairs=0 best_updates=0 result=empty_endpoints";
        }
        return {};
    }

    const auto shared_corridor_start = std::chrono::steady_clock::now();
    const auto shared_query_level_policy =
        mld::temporal::overlay::detail::MakeSharedEndpointQueryLevelPolicy(
            facade, source_endpoints, target_endpoints);
    const auto shared_corridor_stop = std::chrono::steady_clock::now();
    const auto shared_reverse_lower_bounds_start = std::chrono::steady_clock::now();
    mld::temporal::overlay::detail::ReverseLowerBoundWorkspace shared_reverse_lower_bound_workspace{
        facade.GetNumberOfNodes()};
    mld::temporal::overlay::detail::ReverseLowerBoundSearchStats shared_reverse_lower_bound_stats;
    const auto &shared_reverse_lower_bounds =
        mld::temporal::overlay::detail::ComputeReverseLowerBoundsForTargets(
            facade,
            target_endpoints,
            {},
            shared_query_level_policy,
            shared_reverse_lower_bound_workspace,
            &shared_reverse_lower_bound_stats);
    const auto shared_reverse_lower_bounds_stop = std::chrono::steady_clock::now();

    if (overlay_timing_enabled)
    {
        util::Log(logDEBUG)
            << "[overlay query] shared_reverse_lower_bounds ms="
            << mld::temporal::overlay::detail::OverlayDurationMs(
                   shared_reverse_lower_bounds_stop - shared_reverse_lower_bounds_start)
            << " target_endpoints=" << target_endpoints.size()
            << " queue_pops=" << shared_reverse_lower_bound_stats.queue_pops
            << " clique_candidates=" << shared_reverse_lower_bound_stats.clique_candidates
            << " border_edge_candidates="
            << shared_reverse_lower_bound_stats.border_edge_candidates
            << " lower_bound_updates=" << shared_reverse_lower_bound_stats.lower_bound_updates;
    }

    mld::temporal::overlay::TemporalOverlayPath best_path;
    const mld::temporal::DirectedPhantomEndpoint *best_source = nullptr;
    const mld::temporal::DirectedPhantomEndpoint *best_target = nullptr;
    std::size_t candidate_pairs = 0;
    std::size_t valid_pairs = 0;
    std::size_t best_updates = 0;
    std::chrono::steady_clock::duration pair_search_duration{0};

    for (const auto &source : source_endpoints)
    {
        for (const auto &target : target_endpoints)
        {
            ++candidate_pairs;
            const auto pair_search_start = std::chrono::steady_clock::now();
            const auto candidate = mld::temporal::overlay::Search(
                facade,
                source,
                target,
                departure_timestamp,
                shared_query_level_policy,
                shared_reverse_lower_bounds);
            const auto pair_search_stop = std::chrono::steady_clock::now();
            pair_search_duration += pair_search_stop - pair_search_start;

            if (overlay_timing_enabled)
            {
                util::Log(logDEBUG)
                    << "[overlay query] pair_search source_node=" << source.node
                    << " target_node=" << target.node
                    << " ms="
                    << mld::temporal::overlay::detail::OverlayDurationMs(
                           pair_search_stop - pair_search_start)
                    << " valid=" << candidate.is_valid()
                    << " duration="
                    << (candidate.is_valid() ? from_alias<std::int32_t>(candidate.total_duration)
                                             : -1);
            }

            if (candidate.is_valid())
            {
                ++valid_pairs;
            }

            if (!candidate.is_valid() ||
                (best_path.is_valid() && candidate.total_duration >= best_path.total_duration))
            {
                continue;
            }

            best_path = candidate;
            best_source = &source;
            best_target = &target;
            ++best_updates;
        }
    }

    if (!best_path.is_valid() || best_source == nullptr || best_target == nullptr)
    {
        if (overlay_timing_enabled)
        {
            util::Log(logDEBUG)
                << "[overlay query] direct_shortest_path ms="
                << mld::temporal::overlay::detail::OverlayDurationMs(
                       std::chrono::steady_clock::now() - query_start)
                << " endpoint_ms="
                << mld::temporal::overlay::detail::OverlayDurationMs(
                       endpoint_enumeration_stop - endpoint_enumeration_start)
                << " shared_corridor_ms="
                << mld::temporal::overlay::detail::OverlayDurationMs(
                       shared_corridor_stop - shared_corridor_start)
                << " shared_reverse_ms="
                << mld::temporal::overlay::detail::OverlayDurationMs(
                       shared_reverse_lower_bounds_stop - shared_reverse_lower_bounds_start)
                << " pair_search_ms="
                << mld::temporal::overlay::detail::OverlayDurationMs(pair_search_duration)
                << " unpack_ms=0"
                << " source_endpoints=" << source_endpoints.size()
                << " target_endpoints=" << target_endpoints.size()
                << " candidate_pairs=" << candidate_pairs
                << " valid_pairs=" << valid_pairs
                << " best_updates=" << best_updates
                << " result=no_best_path";
        }
        return {};
    }

    const auto unpack_start = std::chrono::steady_clock::now();
    const auto unpacked = mld::temporal::overlay::UnpackPath(
        facade, *best_source, *best_target, departure_timestamp, best_path);
    const auto unpack_stop = std::chrono::steady_clock::now();
    if (!unpacked.is_valid())
    {
        if (overlay_timing_enabled)
        {
            util::Log(logDEBUG)
                << "[overlay query] direct_shortest_path ms="
                << mld::temporal::overlay::detail::OverlayDurationMs(
                       std::chrono::steady_clock::now() - query_start)
                << " endpoint_ms="
                << mld::temporal::overlay::detail::OverlayDurationMs(
                       endpoint_enumeration_stop - endpoint_enumeration_start)
                << " shared_corridor_ms="
                << mld::temporal::overlay::detail::OverlayDurationMs(
                       shared_corridor_stop - shared_corridor_start)
                << " shared_reverse_ms="
                << mld::temporal::overlay::detail::OverlayDurationMs(
                       shared_reverse_lower_bounds_stop - shared_reverse_lower_bounds_start)
                << " pair_search_ms="
                << mld::temporal::overlay::detail::OverlayDurationMs(pair_search_duration)
                << " unpack_ms="
                << mld::temporal::overlay::detail::OverlayDurationMs(unpack_stop - unpack_start)
                << " source_endpoints=" << source_endpoints.size()
                << " target_endpoints=" << target_endpoints.size()
                << " candidate_pairs=" << candidate_pairs
                << " valid_pairs=" << valid_pairs
                << " best_updates=" << best_updates
                << " result=unpack_failed";
        }
        return {};
    }

    if (overlay_timing_enabled)
    {
        util::Log(logDEBUG)
            << "[overlay query] direct_shortest_path ms="
            << mld::temporal::overlay::detail::OverlayDurationMs(
                   std::chrono::steady_clock::now() - query_start)
            << " endpoint_ms="
            << mld::temporal::overlay::detail::OverlayDurationMs(
                   endpoint_enumeration_stop - endpoint_enumeration_start)
            << " shared_corridor_ms="
            << mld::temporal::overlay::detail::OverlayDurationMs(
                   shared_corridor_stop - shared_corridor_start)
            << " shared_reverse_ms="
            << mld::temporal::overlay::detail::OverlayDurationMs(
                   shared_reverse_lower_bounds_stop - shared_reverse_lower_bounds_start)
            << " pair_search_ms="
            << mld::temporal::overlay::detail::OverlayDurationMs(pair_search_duration)
            << " unpack_ms="
            << mld::temporal::overlay::detail::OverlayDurationMs(unpack_stop - unpack_start)
            << " source_endpoints=" << source_endpoints.size()
            << " target_endpoints=" << target_endpoints.size()
            << " candidate_pairs=" << candidate_pairs
            << " valid_pairs=" << valid_pairs
            << " best_updates=" << best_updates
            << " best_source_node=" << best_source->node
            << " best_target_node=" << best_target->node
            << " best_duration=" << from_alias<std::int32_t>(unpacked.total_duration)
            << " result=ok";
    }

    PhantomNodeCandidates source_candidates{*best_source->phantom};
    PhantomNodeCandidates target_candidates{*best_target->phantom};
    const PhantomEndpointCandidates best_candidates{source_candidates, target_candidates};

    return extractRoute(facade,
                        alias_cast<EdgeWeight>(unpacked.total_duration),
                        best_candidates,
                        unpacked.nodes,
                        unpacked.edges);
}

} // namespace osrm::engine::routing_algorithms
