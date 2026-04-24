#ifndef OSRM_ENGINE_ROUTING_ALGORITHMS_TEMPORAL_ASYMMETRIC_MLD_HPP
#define OSRM_ENGINE_ROUTING_ALGORITHMS_TEMPORAL_ASYMMETRIC_MLD_HPP

#include "engine/phantom_node.hpp"
#include "engine/internal_route_result.hpp"
#include "engine/temporal_traffic.hpp"

#include "util/typedefs.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

namespace osrm::engine::routing_algorithms::mld::temporal
{

struct DirectedPhantomEndpoint
{
    const PhantomNode *phantom = nullptr;
    NodeID node = SPECIAL_NODEID;
    bool traversed_in_reverse = false;
};

struct TemporalAsymmetricPath
{
    EdgeDuration total_duration = INVALID_EDGE_DURATION;
    std::vector<NodeID> nodes;

    bool is_valid() const { return total_duration != INVALID_EDGE_DURATION && !nodes.empty(); }
};

namespace detail
{
struct IncomingEdge
{
    NodeID from = SPECIAL_NODEID;
    EdgeID edge = SPECIAL_EDGEID;
};

struct QueueEntry
{
    EdgeDuration cost = INVALID_EDGE_DURATION;
    NodeID node = SPECIAL_NODEID;
};

struct QueueCompare
{
    bool operator()(const QueueEntry &lhs, const QueueEntry &rhs) const
    {
        return from_alias<std::int32_t>(lhs.cost) > from_alias<std::int32_t>(rhs.cost);
    }
};

inline bool IsInvalidDuration(const EdgeDuration duration)
{
    return !engine::temporal::detail::IsFiniteNonNegativeDuration(duration);
}

inline EdgeDuration SafeDurationAdd(const EdgeDuration lhs, const EdgeDuration rhs)
{
    return engine::temporal::detail::SafeDurationAdd(lhs, rhs);
}

inline EdgeDuration TurnPenaltyToDuration(const TurnPenalty penalty)
{
    return to_alias<EdgeDuration>(from_alias<std::int64_t>(penalty));
}

template <typename FacadeT>
EdgeDuration GetNodeLowerBoundDuration(const FacadeT &facade, const NodeID node)
{
    const auto geometry_index = facade.GetGeometryIndex(node);
    const auto static_duration = engine::temporal::GetStaticGeometryDuration(
        facade, geometry_index.id, geometry_index.forward);
    const auto temporal_min = geometry_index.forward
                                  ? facade.GetTemporalForwardMinDuration(geometry_index.id)
                                  : facade.GetTemporalReverseMinDuration(geometry_index.id);

    if (engine::temporal::detail::IsPlausibleTemporalDuration(temporal_min, static_duration))
    {
        return std::min(temporal_min, static_duration);
    }

    return static_duration;
}

template <typename FacadeT>
EdgeDuration GetNodeDurationAtClock(const FacadeT &facade,
                                    const NodeID node,
                                    const engine::temporal::TemporalClock clock)
{
    const auto geometry_index = facade.GetGeometryIndex(node);
    return engine::temporal::GetGeometryDurationAtClock(
               facade, geometry_index.id, geometry_index.forward, clock)
        .duration;
}

template <typename FacadeT>
EdgeDuration GetPhantomTraversalDurationAtClock(const FacadeT &facade,
                                                const DirectedPhantomEndpoint &endpoint,
                                                const engine::temporal::TemporalClock clock)
{
    const auto geometry_index = facade.GetGeometryIndex(endpoint.node);
    const auto full_duration = engine::temporal::GetGeometryDurationAtClock(
                                   facade, geometry_index.id, geometry_index.forward, clock)
                                   .duration;
    const auto static_total = engine::temporal::GetStaticGeometryDuration(
        facade, geometry_index.id, geometry_index.forward);
    const auto partial_duration =
        engine::temporal::detail::GetTraversalDuration(*endpoint.phantom,
                                                       endpoint.traversed_in_reverse);
    return engine::temporal::detail::ScaleDurationProportionally(
        partial_duration, static_total, full_duration);
}

template <typename FacadeT>
EdgeDuration GetPhantomTraversalLowerBound(const FacadeT &facade,
                                           const DirectedPhantomEndpoint &endpoint)
{
    const auto geometry_index = facade.GetGeometryIndex(endpoint.node);
    const auto static_total = engine::temporal::GetStaticGeometryDuration(
        facade, geometry_index.id, geometry_index.forward);
    auto full_duration = geometry_index.forward
                             ? facade.GetTemporalForwardMinDuration(geometry_index.id)
                             : facade.GetTemporalReverseMinDuration(geometry_index.id);

    if (engine::temporal::detail::IsPlausibleTemporalDuration(full_duration, static_total))
    {
        full_duration = std::min(full_duration, static_total);
    }
    else
    {
        full_duration = static_total;
    }

    const auto partial_duration =
        engine::temporal::detail::GetTraversalDuration(*endpoint.phantom,
                                                       endpoint.traversed_in_reverse);
    return engine::temporal::detail::ScaleDurationProportionally(
        partial_duration, static_total, full_duration);
}

template <typename FacadeT>
EdgeDuration GetSourceRemainingDurationAtClock(const FacadeT &facade,
                                               const DirectedPhantomEndpoint &source,
                                               const engine::temporal::TemporalClock clock)
{
    const auto node_duration = GetNodeDurationAtClock(facade, source.node, clock);
    const auto source_offset = GetPhantomTraversalDurationAtClock(facade, source, clock);
    return engine::temporal::detail::SafeDurationSubFloorZero(node_duration, source_offset);
}

template <typename FacadeT>
EdgeDuration GetSourceRemainingLowerBound(const FacadeT &facade,
                                         const DirectedPhantomEndpoint &source)
{
    const auto node_duration = GetNodeLowerBoundDuration(facade, source.node);
    const auto source_offset = GetPhantomTraversalLowerBound(facade, source);
    return engine::temporal::detail::SafeDurationSubFloorZero(node_duration, source_offset);
}

template <typename FacadeT>
EdgeDuration EvaluateLocalPathDuration(const FacadeT &facade,
                                       const DirectedPhantomEndpoint &source,
                                       const DirectedPhantomEndpoint &target,
                                       const engine::temporal::TemporalClock departure_clock)
{
    const auto source_offset = GetPhantomTraversalDurationAtClock(facade, source, departure_clock);
    const auto target_offset = GetPhantomTraversalDurationAtClock(facade, target, departure_clock);
    return engine::temporal::detail::SafeDurationSubFloorZero(target_offset, source_offset);
}

template <typename FacadeT>
std::vector<std::vector<IncomingEdge>> BuildIncomingEdgeIndex(const FacadeT &facade)
{
    std::vector<std::vector<IncomingEdge>> incoming_edges(facade.GetNumberOfNodes());

    for (NodeID node = 0; node < facade.GetNumberOfNodes(); ++node)
    {
        for (const auto edge : facade.GetAdjacentEdgeRange(node))
        {
            if (!facade.IsForwardEdge(edge))
            {
                continue;
            }

            const auto target = facade.GetTarget(edge);
            if (target < incoming_edges.size())
            {
                incoming_edges[target].push_back({node, edge});
            }
        }
    }

    return incoming_edges;
}

template <typename FacadeT>
std::vector<EdgeDuration>
ComputeReverseLowerBounds(const FacadeT &facade,
                          const DirectedPhantomEndpoint &target,
                          const std::vector<std::vector<IncomingEdge>> &incoming_edges)
{
    std::vector<EdgeDuration> lower_bounds(facade.GetNumberOfNodes(), INVALID_EDGE_DURATION);
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> queue;

    const auto target_lower_bound = GetPhantomTraversalLowerBound(facade, target);
    lower_bounds[target.node] = target_lower_bound;
    queue.push({target_lower_bound, target.node});

    while (!queue.empty())
    {
        const auto current = queue.top();
        queue.pop();

        if (current.cost != lower_bounds[current.node])
        {
            continue;
        }

        for (const auto &incoming : incoming_edges[current.node])
        {
            const auto &edge_data = facade.GetEdgeData(incoming.edge);
            const auto turn_penalty =
                TurnPenaltyToDuration(facade.GetDurationPenaltyForEdgeID(edge_data.turn_id));
            const auto predecessor_duration = GetNodeLowerBoundDuration(facade, incoming.from);
            const auto edge_cost = SafeDurationAdd(predecessor_duration, turn_penalty);
            const auto candidate = SafeDurationAdd(current.cost, edge_cost);

            if (candidate == INVALID_EDGE_DURATION)
            {
                continue;
            }

            if (lower_bounds[incoming.from] == INVALID_EDGE_DURATION ||
                candidate < lower_bounds[incoming.from])
            {
                lower_bounds[incoming.from] = candidate;
                queue.push({candidate, incoming.from});
            }
        }
    }

    return lower_bounds;
}

template <typename FacadeT>
TemporalAsymmetricPath SearchForwardDijkstra(
    const FacadeT &facade,
    const DirectedPhantomEndpoint &source,
    const DirectedPhantomEndpoint &target,
    const std::time_t departure_timestamp)
{
    TemporalAsymmetricPath best_path;
    const auto departure_clock = engine::temporal::ToTemporalClock(departure_timestamp);

    if (source.node == target.node)
    {
        const auto local_duration =
            EvaluateLocalPathDuration(facade, source, target, departure_clock);
        best_path.total_duration = local_duration;
        best_path.nodes = {source.node};
    }

    std::vector<EdgeDuration> settled_costs(facade.GetNumberOfNodes(), INVALID_EDGE_DURATION);
    std::vector<NodeID> parents(facade.GetNumberOfNodes(), SPECIAL_NODEID);
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> queue;

    settled_costs[source.node] = EdgeDuration{0};
    parents[source.node] = source.node;
    queue.push({EdgeDuration{0}, source.node});

    while (!queue.empty())
    {
        const auto current = queue.top();
        queue.pop();

        if (current.cost != settled_costs[current.node])
        {
            continue;
        }

        if (current.node == target.node &&
            (current.cost > EdgeDuration{0} || source.node != target.node))
        {
            const auto arrival_clock =
                engine::temporal::AdvanceTemporalClock(departure_clock, current.cost);
            const auto target_duration =
                GetPhantomTraversalDurationAtClock(facade, target, arrival_clock);
            const auto candidate_total = SafeDurationAdd(current.cost, target_duration);

            if (candidate_total != INVALID_EDGE_DURATION &&
                (!best_path.is_valid() || candidate_total < best_path.total_duration))
            {
                std::vector<NodeID> nodes;
                auto trace = current.node;
                while (trace != SPECIAL_NODEID)
                {
                    nodes.push_back(trace);
                    if (trace == parents[trace])
                    {
                        break;
                    }
                    trace = parents[trace];
                }
                std::reverse(nodes.begin(), nodes.end());
                best_path.total_duration = candidate_total;
                best_path.nodes = std::move(nodes);
            }
        }

        const auto current_clock =
            engine::temporal::AdvanceTemporalClock(departure_clock, current.cost);
        const auto node_duration =
            current.node == source.node && current.cost == EdgeDuration{0}
                ? GetSourceRemainingDurationAtClock(facade, source, current_clock)
                : GetNodeDurationAtClock(facade, current.node, current_clock);

        for (const auto edge : facade.GetAdjacentEdgeRange(current.node))
        {
            if (!facade.IsForwardEdge(edge))
            {
                continue;
            }

            const auto target_node = facade.GetTarget(edge);
            if (facade.ExcludeNode(target_node))
            {
                continue;
            }

            const auto &edge_data = facade.GetEdgeData(edge);
            const auto turn_penalty =
                TurnPenaltyToDuration(facade.GetDurationPenaltyForEdgeID(edge_data.turn_id));
            const auto edge_cost = SafeDurationAdd(node_duration, turn_penalty);
            const auto candidate_cost = SafeDurationAdd(current.cost, edge_cost);

            if (edge_cost == INVALID_EDGE_DURATION || candidate_cost == INVALID_EDGE_DURATION)
            {
                continue;
            }

            if (settled_costs[target_node] == INVALID_EDGE_DURATION ||
                candidate_cost < settled_costs[target_node])
            {
                settled_costs[target_node] = candidate_cost;
                parents[target_node] = current.node;
                queue.push({candidate_cost, target_node});
            }
        }
    }

    return best_path;
}

template <typename FacadeT>
TemporalAsymmetricPath SearchWithIncomingEdges(
    const FacadeT &facade,
    const DirectedPhantomEndpoint &source,
    const DirectedPhantomEndpoint &target,
    const std::time_t departure_timestamp,
    const std::vector<std::vector<IncomingEdge>> &incoming_edges)
{
    (void)incoming_edges;
    return SearchForwardDijkstra(facade, source, target, departure_timestamp);
}
} // namespace detail

inline std::vector<DirectedPhantomEndpoint>
EnumerateSourceEndpoints(const PhantomNodeCandidates &source_candidates)
{
    std::vector<DirectedPhantomEndpoint> endpoints;
    endpoints.reserve(source_candidates.size() * 2);

    for (const auto &phantom : source_candidates)
    {
        if (phantom.IsValidForwardSource())
        {
            endpoints.push_back({&phantom, phantom.forward_segment_id.id, false});
        }
        if (phantom.IsValidReverseSource())
        {
            endpoints.push_back({&phantom, phantom.reverse_segment_id.id, true});
        }
    }

    return endpoints;
}

inline std::vector<DirectedPhantomEndpoint>
EnumerateTargetEndpoints(const PhantomNodeCandidates &target_candidates)
{
    std::vector<DirectedPhantomEndpoint> endpoints;
    endpoints.reserve(target_candidates.size() * 2);

    for (const auto &phantom : target_candidates)
    {
        if (phantom.IsValidForwardTarget())
        {
            endpoints.push_back({&phantom, phantom.forward_segment_id.id, false});
        }
        if (phantom.IsValidReverseTarget())
        {
            endpoints.push_back({&phantom, phantom.reverse_segment_id.id, true});
        }
    }

    return endpoints;
}

template <typename FacadeT>
std::vector<std::vector<detail::IncomingEdge>> BuildIncomingEdgeIndex(const FacadeT &facade)
{
    return detail::BuildIncomingEdgeIndex(facade);
}

template <typename FacadeT>
TemporalAsymmetricPath Search(const FacadeT &facade,
                              const DirectedPhantomEndpoint &source,
                              const DirectedPhantomEndpoint &target,
                              const std::time_t departure_timestamp)
{
    return detail::SearchForwardDijkstra(facade, source, target, departure_timestamp);
}

template <typename FacadeT>
TemporalAsymmetricPath Search(const FacadeT &facade,
                              const DirectedPhantomEndpoint &source,
                              const DirectedPhantomEndpoint &target,
                              const std::time_t departure_timestamp,
                              const EdgeDuration initial_upper_bound)
{
    (void)initial_upper_bound;
    return detail::SearchForwardDijkstra(facade, source, target, departure_timestamp);
}

template <typename FacadeT>
TemporalAsymmetricPath Search(const FacadeT &facade,
                              const DirectedPhantomEndpoint &source,
                              const DirectedPhantomEndpoint &target,
                              const std::time_t departure_timestamp,
                              const std::vector<std::vector<detail::IncomingEdge>> &incoming_edges)
{
    return detail::SearchWithIncomingEdges(
        facade, source, target, departure_timestamp, incoming_edges);
}

} // namespace osrm::engine::routing_algorithms::mld::temporal

#endif
