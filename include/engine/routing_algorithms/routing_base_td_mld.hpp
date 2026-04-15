#ifndef OSRM_ENGINE_ROUTING_BASE_TD_MLD_HPP
#define OSRM_ENGINE_ROUTING_BASE_TD_MLD_HPP

#include "engine/algorithm.hpp"
#include "engine/datafacade.hpp"
#include "engine/phantom_node.hpp"
#include "engine/routing_algorithms/temporal_asymmetric_mld.hpp"
#include "engine/temporal_traffic.hpp"

#include "util/typedefs.hpp"

#include <algorithm>
#include <cstdint>
#include <ctime>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

namespace osrm::engine::routing_algorithms::mld::temporal::overlay
{

struct PackedEdge
{
    NodeID from = SPECIAL_NODEID;
    NodeID to = SPECIAL_NODEID;
    bool is_overlay = false;
    LevelID overlay_level = INVALID_LEVEL_ID;
};
using PackedPath = std::vector<PackedEdge>;

struct TemporalOverlayPath
{
    EdgeDuration total_duration = INVALID_EDGE_DURATION;
    PackedPath packed_path;

    bool is_valid() const { return total_duration != INVALID_EDGE_DURATION; }
};

struct TemporalOverlayUnpackedPath
{
    EdgeDuration total_duration = INVALID_EDGE_DURATION;
    std::vector<NodeID> nodes;
    std::vector<EdgeID> edges;

    bool is_valid() const { return total_duration != INVALID_EDGE_DURATION && !nodes.empty(); }
};

namespace detail
{
struct SearchRestriction
{
    bool restricted = false;
    LevelID level = INVALID_LEVEL_ID;
    CellID parent_cell = INVALID_CELL_ID;
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

template <typename FnT> inline void ForEachDescendingLevel(const LevelID start_level, FnT &&fn)
{
    for (auto level_int = static_cast<std::int32_t>(start_level); level_int >= 0; --level_int)
    {
        fn(static_cast<LevelID>(level_int));
    }
}

template <typename MultiLevelPartition>
inline LevelID GetNodeQueryLevel(const MultiLevelPartition &partition,
                                 const NodeID node,
                                 const DirectedPhantomEndpoint &source,
                                 const DirectedPhantomEndpoint &target,
                                 const SearchRestriction &restriction)
{
    if (restriction.restricted)
    {
        return restriction.level;
    }

    return partition.GetQueryLevel(source.node, target.node, node);
}

template <typename MultiLevelPartition>
inline bool CheckParentCellRestriction(const MultiLevelPartition &partition,
                                       const LevelID level,
                                       const NodeID node,
                                       const SearchRestriction &restriction)
{
    if (!restriction.restricted)
    {
        return true;
    }

    if (level + 1 >= partition.GetNumberOfLevels())
    {
        return true;
    }

    return partition.GetCell(level + 1, node) == restriction.parent_cell;
}

template <typename FacadeT>
inline std::uint32_t GetWeekBucketForClock(const FacadeT &facade,
                                           const engine::temporal::TemporalClock clock)
{
    const auto bucket_size_minutes = facade.GetTemporalBucketSizeMinutes();
    const auto week_bucket_count = facade.GetTemporalWeekBucketCount();
    if (bucket_size_minutes == 0 || week_bucket_count == 0)
    {
        return 0;
    }

    return std::min(engine::temporal::TimestampToWeekBucketFromClock(clock, bucket_size_minutes),
                    week_bucket_count - 1);
}

template <typename FacadeT>
inline EdgeDuration GetExactBaseEdgeDurationAtClock(const FacadeT &facade,
                                                    const DirectedPhantomEndpoint &source,
                                                    const NodeID from_node,
                                                    const EdgeID edge,
                                                    const bool is_first_edge,
                                                    const engine::temporal::TemporalClock clock)
{
    const auto node_duration =
        is_first_edge && from_node == source.node
            ? temporal::detail::GetSourceRemainingDurationAtClock(facade, source, clock)
            : temporal::detail::GetNodeDurationAtClock(facade, from_node, clock);
    const auto turn_penalty = temporal::detail::TurnPenaltyToDuration(
        facade.GetDurationPenaltyForEdgeID(facade.GetEdgeData(edge).turn_id));
    return temporal::detail::SafeDurationAdd(node_duration, turn_penalty);
}

inline PhantomNode MakeZeroOffsetPhantom(const NodeID node)
{
    PhantomNode phantom;
    phantom.forward_segment_id = SegmentID{node, true};
    phantom.forward_duration = EdgeDuration{0};
    phantom.forward_weight = EdgeWeight{0};
    phantom.component = ComponentID{1, false};
    phantom.location = {util::FloatLongitude{0.0}, util::FloatLatitude{0.0}};
    phantom.input_location = phantom.location;
    return phantom;
}

inline PackedPath BuildPackedPath(const std::vector<NodeID> &parents,
                                  const std::vector<bool> &from_clique_arc,
                                  const std::vector<LevelID> &overlay_levels,
                                  const NodeID source,
                                  const NodeID target)
{
    PackedPath packed_path;
    if (source == target)
    {
        return packed_path;
    }

    auto current = target;
    while (current != source)
    {
        if (current == SPECIAL_NODEID || current >= parents.size() || parents[current] == SPECIAL_NODEID)
        {
            return {};
        }

        packed_path.push_back(
            {parents[current], current, from_clique_arc[current], overlay_levels[current]});
        current = parents[current];
    }

    std::reverse(packed_path.begin(), packed_path.end());
    return packed_path;
}

template <typename FacadeT>
std::vector<EdgeDuration> ComputeReverseLowerBounds(const FacadeT &facade,
                                                    const DirectedPhantomEndpoint &source,
                                                    const DirectedPhantomEndpoint &target,
                                                    const SearchRestriction &restriction)
{
    std::vector<EdgeDuration> lower_bounds(facade.GetNumberOfNodes(), INVALID_EDGE_DURATION);
    std::vector<bool> from_clique_arc(facade.GetNumberOfNodes(), false);
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> queue;

    const auto target_lower_bound = temporal::detail::GetPhantomTraversalLowerBound(facade, target);
    lower_bounds[target.node] = target_lower_bound;
    queue.push({target_lower_bound, target.node});

    const auto &partition = facade.GetMultiLevelPartition();
    const auto &cells = facade.GetCellStorage();

    while (!queue.empty())
    {
        const auto current = queue.top();
        queue.pop();

        if (current.cost != lower_bounds[current.node])
        {
            continue;
        }

        const auto level =
            GetNodeQueryLevel(partition, current.node, source, target, restriction);

        if (level >= 1 && !from_clique_arc[current.node])
        {
            const auto cell_id = partition.GetCell(level, current.node);
            const auto cell = cells.GetUnfilledCell(level, cell_id);

            for (const auto predecessor : cell.GetSourceNodes())
            {
                if (predecessor == current.node ||
                    !CheckParentCellRestriction(partition, level, predecessor, restriction))
                {
                    continue;
                }

                const auto shortcut_min =
                    facade.GetTemporalShortcutMinDuration(level, cell_id, predecessor, current.node);
                const auto candidate =
                    temporal::detail::SafeDurationAdd(current.cost, shortcut_min);

                if (candidate == INVALID_EDGE_DURATION)
                {
                    continue;
                }

                if (lower_bounds[predecessor] == INVALID_EDGE_DURATION ||
                    candidate < lower_bounds[predecessor])
                {
                    lower_bounds[predecessor] = candidate;
                    from_clique_arc[predecessor] = true;
                    queue.push({candidate, predecessor});
                }
            }
        }

        ForEachDescendingLevel(
            level,
            [&](const LevelID edge_level)
            {
                for (const auto edge : facade.GetBorderEdgeRange(edge_level, current.node))
                {
                    if (!facade.IsBackwardEdge(edge))
                    {
                        continue;
                    }

                    const auto predecessor = facade.GetTarget(edge);
                    if (facade.ExcludeNode(predecessor) ||
                        !CheckParentCellRestriction(
                            partition, edge_level, predecessor, restriction))
                    {
                        continue;
                    }

                    const auto node_duration =
                        temporal::detail::GetNodeLowerBoundDuration(facade, predecessor);
                    const auto turn_penalty = temporal::detail::TurnPenaltyToDuration(
                        facade.GetDurationPenaltyForEdgeID(facade.GetEdgeData(edge).turn_id));
                    const auto edge_cost =
                        temporal::detail::SafeDurationAdd(node_duration, turn_penalty);
                    const auto candidate =
                        temporal::detail::SafeDurationAdd(current.cost, edge_cost);

                    if (candidate == INVALID_EDGE_DURATION)
                    {
                        continue;
                    }

                    if (lower_bounds[predecessor] == INVALID_EDGE_DURATION ||
                        candidate < lower_bounds[predecessor])
                    {
                        lower_bounds[predecessor] = candidate;
                        from_clique_arc[predecessor] = false;
                        queue.push({candidate, predecessor});
                    }
                }
            });
    }

    return lower_bounds;
}

template <typename FacadeT>
TemporalOverlayPath SearchAtClock(const FacadeT &facade,
                                  const DirectedPhantomEndpoint &source,
                                  const DirectedPhantomEndpoint &target,
                                  const engine::temporal::TemporalClock departure_clock,
                                  const SearchRestriction &restriction)
{
    TemporalOverlayPath best_path;

    if (source.node == target.node)
    {
        best_path.total_duration =
            temporal::detail::EvaluateLocalPathDuration(facade, source, target, departure_clock);
        return best_path;
    }

    const auto reverse_lower_bounds =
        ComputeReverseLowerBounds(facade, source, target, restriction);
    if (source.node >= reverse_lower_bounds.size() ||
        reverse_lower_bounds[source.node] == INVALID_EDGE_DURATION)
    {
        return best_path;
    }

    const auto &partition = facade.GetMultiLevelPartition();
    const auto &cells = facade.GetCellStorage();

    std::vector<EdgeDuration> settled_costs(facade.GetNumberOfNodes(), INVALID_EDGE_DURATION);
    std::vector<NodeID> parents(facade.GetNumberOfNodes(), SPECIAL_NODEID);
    std::vector<bool> from_clique_arc(facade.GetNumberOfNodes(), false);
    std::vector<LevelID> overlay_levels(facade.GetNumberOfNodes(), INVALID_LEVEL_ID);
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

        auto lower_bound = reverse_lower_bounds[current.node];
        if (lower_bound == INVALID_EDGE_DURATION)
        {
            continue;
        }

        if (current.node == source.node && current.cost == EdgeDuration{0})
        {
            const auto node_lower_bound = temporal::detail::GetNodeLowerBoundDuration(
                facade, source.node);
            lower_bound = temporal::detail::GetSourceRemainingLowerBound(facade, source);
            lower_bound = temporal::detail::SafeDurationAdd(
                lower_bound,
                engine::temporal::detail::SafeDurationSubFloorZero(
                    reverse_lower_bounds[source.node], node_lower_bound));
        }

        const auto optimistic_total = temporal::detail::SafeDurationAdd(current.cost, lower_bound);
        if (best_path.is_valid() && optimistic_total != INVALID_EDGE_DURATION &&
            optimistic_total >= best_path.total_duration)
        {
            continue;
        }

        if (current.node == target.node)
        {
            const auto arrival_clock =
                engine::temporal::AdvanceTemporalClock(departure_clock, current.cost);
            const auto target_duration =
                temporal::detail::GetPhantomTraversalDurationAtClock(facade, target, arrival_clock);
            const auto candidate_total =
                temporal::detail::SafeDurationAdd(current.cost, target_duration);

            if (candidate_total != INVALID_EDGE_DURATION &&
                (!best_path.is_valid() || candidate_total < best_path.total_duration))
            {
                auto packed_path = BuildPackedPath(
                    parents, from_clique_arc, overlay_levels, source.node, current.node);
                if (source.node != current.node && packed_path.empty())
                {
                    continue;
                }

                best_path.total_duration = candidate_total;
                best_path.packed_path = std::move(packed_path);
            }
        }

        const auto current_clock =
            engine::temporal::AdvanceTemporalClock(departure_clock, current.cost);
        const auto current_level =
            GetNodeQueryLevel(partition, current.node, source, target, restriction);

        if (current_level >= 1 && !from_clique_arc[current.node])
        {
            const auto cell_id = partition.GetCell(current_level, current.node);
            const auto cell = cells.GetUnfilledCell(current_level, cell_id);
            const auto week_bucket = GetWeekBucketForClock(facade, current_clock);

            for (const auto destination : cell.GetDestinationNodes())
            {
                if (destination == current.node ||
                    !CheckParentCellRestriction(partition, current_level, destination, restriction))
                {
                    continue;
                }

                const auto shortcut_duration = facade.GetTemporalShortcutDuration(
                    current_level, cell_id, current.node, destination, week_bucket);
                const auto candidate =
                    temporal::detail::SafeDurationAdd(current.cost, shortcut_duration);

                if (candidate == INVALID_EDGE_DURATION)
                {
                    continue;
                }

                if (settled_costs[destination] == INVALID_EDGE_DURATION ||
                    candidate < settled_costs[destination])
                {
                    settled_costs[destination] = candidate;
                    parents[destination] = current.node;
                    from_clique_arc[destination] = true;
                    overlay_levels[destination] = current_level;
                    queue.push({candidate, destination});
                }
            }
        }

        const auto node_duration =
            current.node == source.node && current.cost == EdgeDuration{0}
                ? temporal::detail::GetSourceRemainingDurationAtClock(facade, source, current_clock)
                : temporal::detail::GetNodeDurationAtClock(facade, current.node, current_clock);

        ForEachDescendingLevel(
            current_level,
            [&](const LevelID edge_level)
            {
                for (const auto edge : facade.GetBorderEdgeRange(edge_level, current.node))
                {
                    if (!facade.IsForwardEdge(edge))
                    {
                        continue;
                    }

                    const auto next = facade.GetTarget(edge);
                    if (facade.ExcludeNode(next) ||
                        !CheckParentCellRestriction(partition, edge_level, next, restriction))
                    {
                        continue;
                    }

                    const auto turn_penalty = temporal::detail::TurnPenaltyToDuration(
                        facade.GetDurationPenaltyForEdgeID(facade.GetEdgeData(edge).turn_id));
                    const auto edge_cost =
                        temporal::detail::SafeDurationAdd(node_duration, turn_penalty);
                    const auto candidate =
                        temporal::detail::SafeDurationAdd(current.cost, edge_cost);

                    if (candidate == INVALID_EDGE_DURATION)
                    {
                        continue;
                    }

                    if (settled_costs[next] == INVALID_EDGE_DURATION ||
                        candidate < settled_costs[next])
                    {
                        settled_costs[next] = candidate;
                        parents[next] = current.node;
                        from_clique_arc[next] = false;
                        overlay_levels[next] = INVALID_LEVEL_ID;
                        queue.push({candidate, next});
                    }
                }
            });
    }

    return best_path;
}

template <typename FacadeT>
TemporalOverlayPath SearchImpl(const FacadeT &facade,
                               const DirectedPhantomEndpoint &source,
                               const DirectedPhantomEndpoint &target,
                               const std::time_t departure_timestamp,
                               const SearchRestriction &restriction)
{
    return SearchAtClock(
        facade, source, target, engine::temporal::ToTemporalClock(departure_timestamp), restriction);
}

template <typename FacadeT>
TemporalOverlayUnpackedPath UnpackPathImpl(const FacadeT &facade,
                                           const DirectedPhantomEndpoint &source,
                                           const DirectedPhantomEndpoint &target,
                                           const engine::temporal::TemporalClock departure_clock,
                                           const TemporalOverlayPath &path,
                                           const SearchRestriction &restriction)
{
    TemporalOverlayUnpackedPath unpacked;
    if (!path.is_valid())
    {
        return unpacked;
    }

    if (path.packed_path.empty())
    {
        unpacked.total_duration = temporal::detail::EvaluateLocalPathDuration(
            facade, source, target, departure_clock);
        unpacked.nodes = {source.node};
        return unpacked;
    }

    const auto &partition = facade.GetMultiLevelPartition();
    auto current_clock = departure_clock;
    auto first_step = true;

    unpacked.nodes.push_back(path.packed_path.front().from);
    for (const auto &packed_edge : path.packed_path)
    {
        const auto from = packed_edge.from;
        const auto to = packed_edge.to;

        if (!packed_edge.is_overlay)
        {
            const auto edge = facade.FindEdge(from, to);
            if (edge == SPECIAL_EDGEID)
            {
                return {};
            }

            const auto edge_duration = GetExactBaseEdgeDurationAtClock(
                facade, source, from, edge, first_step, current_clock);
            if (edge_duration == INVALID_EDGE_DURATION)
            {
                return {};
            }

            unpacked.nodes.push_back(to);
            unpacked.edges.push_back(edge);
            current_clock = engine::temporal::AdvanceTemporalClock(current_clock, edge_duration);
            first_step = false;
            continue;
        }

        const auto level = packed_edge.overlay_level;
        if (level == INVALID_LEVEL_ID || level == static_cast<LevelID>(0))
        {
            return {};
        }
        const auto parent_cell = partition.GetCell(level, from);
        const auto sublevel = static_cast<LevelID>(level - 1);

        auto sub_source_phantom = MakeZeroOffsetPhantom(from);
        auto sub_target_phantom = MakeZeroOffsetPhantom(to);
        const DirectedPhantomEndpoint sub_source{&sub_source_phantom, from, false};
        const DirectedPhantomEndpoint sub_target{&sub_target_phantom, to, false};
        const SearchRestriction sub_restriction{true, sublevel, parent_cell};
        const auto sub_path =
            SearchAtClock(facade, sub_source, sub_target, current_clock, sub_restriction);
        if (!sub_path.is_valid())
        {
            return {};
        }

        const auto unpacked_subpath =
            UnpackPathImpl(facade, sub_source, sub_target, current_clock, sub_path, sub_restriction);
        if (!unpacked_subpath.is_valid())
        {
            return {};
        }

        unpacked.nodes.insert(unpacked.nodes.end(),
                              std::next(unpacked_subpath.nodes.begin()),
                              unpacked_subpath.nodes.end());
        unpacked.edges.insert(unpacked.edges.end(),
                              unpacked_subpath.edges.begin(),
                              unpacked_subpath.edges.end());
        current_clock =
            engine::temporal::AdvanceTemporalClock(current_clock, unpacked_subpath.total_duration);
        first_step = false;
    }

    const auto target_duration =
        temporal::detail::GetPhantomTraversalDurationAtClock(facade, target, current_clock);
    if (target_duration == INVALID_EDGE_DURATION)
    {
        return {};
    }

    current_clock = engine::temporal::AdvanceTemporalClock(current_clock, target_duration);
    unpacked.total_duration = to_alias<EdgeDuration>(current_clock - departure_clock);
    return unpacked;
}
} // namespace detail

template <typename FacadeT>
TemporalOverlayPath Search(const FacadeT &facade,
                           const DirectedPhantomEndpoint &source,
                           const DirectedPhantomEndpoint &target,
                           const std::time_t departure_timestamp)
{
    return detail::SearchImpl(facade,
                              source,
                              target,
                              departure_timestamp,
                              detail::SearchRestriction{});
}

template <typename FacadeT>
TemporalOverlayPath Search(const FacadeT &facade,
                           const DirectedPhantomEndpoint &source,
                           const DirectedPhantomEndpoint &target,
                           const std::time_t departure_timestamp,
                           const LevelID restricted_level,
                           const CellID parent_cell)
{
    return detail::SearchImpl(facade,
                              source,
                              target,
                              departure_timestamp,
                              detail::SearchRestriction{true, restricted_level, parent_cell});
}

template <typename FacadeT>
TemporalOverlayUnpackedPath UnpackPath(const FacadeT &facade,
                                       const DirectedPhantomEndpoint &source,
                                       const DirectedPhantomEndpoint &target,
                                       const std::time_t departure_timestamp,
                                       const TemporalOverlayPath &path)
{
    return detail::UnpackPathImpl(facade,
                                  source,
                                  target,
                                  engine::temporal::ToTemporalClock(departure_timestamp),
                                  path,
                                  detail::SearchRestriction{});
}

} // namespace osrm::engine::routing_algorithms::mld::temporal::overlay

#endif
