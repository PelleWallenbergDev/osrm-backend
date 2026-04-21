#ifndef OSRM_ENGINE_ROUTING_BASE_TD_MLD_HPP
#define OSRM_ENGINE_ROUTING_BASE_TD_MLD_HPP

#include "engine/algorithm.hpp"
#include "engine/datafacade.hpp"
#include "engine/phantom_node.hpp"
#include "engine/routing_algorithms/temporal_asymmetric_mld.hpp"
#include "engine/temporal_traffic.hpp"

#include "util/log.hpp"
#include "util/typedefs.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <optional>
#include <queue>
#include <type_traits>
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
inline bool OverlayTimingEnabled()
{
    const auto &policy = util::LogPolicy::GetInstance();
    return !policy.IsMute() && policy.GetLevel() >= logDEBUG;
}

inline double OverlayDurationMs(const std::chrono::steady_clock::duration duration)
{
    return 0.000001 *
           std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

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
inline LevelID GetNodeQueryLevel(const MultiLevelPartition &partition,
                                 const NodeID node,
                                 const std::vector<DirectedPhantomEndpoint> &sources,
                                 const DirectedPhantomEndpoint &target,
                                 const SearchRestriction &restriction)
{
    if (restriction.restricted)
    {
        return restriction.level;
    }

    auto level = INVALID_LEVEL_ID;
    for (const auto &source : sources)
    {
        level = std::min(level, partition.GetQueryLevel(source.node, target.node, node));
    }

    return level;
}

template <typename MultiLevelPartition>
inline LevelID GetNodeQueryLevel(const MultiLevelPartition &partition,
                                 const NodeID node,
                                 const std::vector<DirectedPhantomEndpoint> &sources,
                                 const std::vector<DirectedPhantomEndpoint> &targets,
                                 const SearchRestriction &restriction)
{
    if (restriction.restricted)
    {
        return restriction.level;
    }

    auto level = INVALID_LEVEL_ID;
    for (const auto &source : sources)
    {
        for (const auto &target : targets)
        {
            level = std::min(level, partition.GetQueryLevel(source.node, target.node, node));
        }
    }

    return level;
}

template <typename MultiLevelPartition> struct PairQueryLevelPolicy
{
    const MultiLevelPartition &partition;
    const DirectedPhantomEndpoint &source;
    const DirectedPhantomEndpoint &target;

    LevelID GetNodeQueryLevel(const NodeID node, const SearchRestriction &restriction) const
    {
        return overlay::detail::GetNodeQueryLevel(partition, node, source, target, restriction);
    }
};

template <typename MultiLevelPartition> class SharedEndpointQueryLevelPolicy
{
  public:
    SharedEndpointQueryLevelPolicy(const MultiLevelPartition &partition_,
                                   const std::vector<DirectedPhantomEndpoint> &source_endpoints_,
                                   const std::vector<DirectedPhantomEndpoint> &target_endpoints_,
                                   const std::size_t number_of_nodes)
        : partition(partition_), source_endpoints(source_endpoints_),
          target_endpoints(target_endpoints_), cached_levels(number_of_nodes, INVALID_LEVEL_ID),
          cache_valid(number_of_nodes, false)
    {
    }

    LevelID GetNodeQueryLevel(const NodeID node, const SearchRestriction &restriction) const
    {
        if (restriction.restricted)
        {
            return restriction.level;
        }

        if (node >= cached_levels.size())
        {
            return INVALID_LEVEL_ID;
        }

        if (!cache_valid[node])
        {
            cached_levels[node] = overlay::detail::GetNodeQueryLevel(
                partition, node, source_endpoints, target_endpoints, restriction);
            cache_valid[node] = true;
        }

        return cached_levels[node];
    }

  private:
    const MultiLevelPartition &partition;
    const std::vector<DirectedPhantomEndpoint> &source_endpoints;
    const std::vector<DirectedPhantomEndpoint> &target_endpoints;
    mutable std::vector<LevelID> cached_levels;
    mutable std::vector<bool> cache_valid;
};

template <typename FacadeT>
auto MakeSharedEndpointQueryLevelPolicy(
    const FacadeT &facade,
    const std::vector<DirectedPhantomEndpoint> &source_endpoints,
    const std::vector<DirectedPhantomEndpoint> &target_endpoints)
{
    using PartitionT = std::decay_t<decltype(facade.GetMultiLevelPartition())>;

    return SharedEndpointQueryLevelPolicy<PartitionT>{
        facade.GetMultiLevelPartition(),
        source_endpoints,
        target_endpoints,
        facade.GetNumberOfNodes()};
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

template <typename FacadeT, typename QueryLevelPolicyT>
std::vector<EdgeDuration> ComputeReverseLowerBounds(const FacadeT &facade,
                                                    const DirectedPhantomEndpoint &target,
                                                    const SearchRestriction &restriction,
                                                    const QueryLevelPolicyT &query_level_policy);

template <typename FacadeT, typename QueryLevelPolicyT>
TemporalOverlayPath SearchAtClock(const FacadeT &facade,
                                  const DirectedPhantomEndpoint &source,
                                  const DirectedPhantomEndpoint &target,
                                  const engine::temporal::TemporalClock departure_clock,
                                  const SearchRestriction &restriction,
                                  const QueryLevelPolicyT &query_level_policy);

template <typename FacadeT, typename QueryLevelPolicyT>
TemporalOverlayPath SearchImpl(const FacadeT &facade,
                               const DirectedPhantomEndpoint &source,
                               const DirectedPhantomEndpoint &target,
                               const std::time_t departure_timestamp,
                               const SearchRestriction &restriction,
                               const QueryLevelPolicyT &query_level_policy);

template <typename FacadeT>
std::vector<EdgeDuration> ComputeReverseLowerBounds(const FacadeT &facade,
                                                    const DirectedPhantomEndpoint &source,
                                                    const DirectedPhantomEndpoint &target,
                                                    const SearchRestriction &restriction)
{
    const PairQueryLevelPolicy pair_query_level_policy{
        facade.GetMultiLevelPartition(), source, target};
    return ComputeReverseLowerBounds(facade, target, restriction, pair_query_level_policy);
}

template <typename FacadeT, typename QueryLevelPolicyT>
std::vector<EdgeDuration> ComputeReverseLowerBounds(const FacadeT &facade,
                                                    const DirectedPhantomEndpoint &target,
                                                    const SearchRestriction &restriction,
                                                    const QueryLevelPolicyT &query_level_policy)
{
    const auto timing_enabled = OverlayTimingEnabled();
    const auto reverse_search_start = std::chrono::steady_clock::now();
    std::vector<EdgeDuration> lower_bounds(facade.GetNumberOfNodes(), INVALID_EDGE_DURATION);
    std::vector<bool> from_clique_arc(facade.GetNumberOfNodes(), false);
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> queue;
    std::size_t queue_pops = 0;
    std::size_t clique_candidates = 0;
    std::size_t border_edge_candidates = 0;
    std::size_t lower_bound_updates = 1;

    const auto target_lower_bound = temporal::detail::GetPhantomTraversalLowerBound(facade, target);
    lower_bounds[target.node] = target_lower_bound;
    queue.push({target_lower_bound, target.node});

    const auto &partition = facade.GetMultiLevelPartition();
    const auto &cells = facade.GetCellStorage();

    while (!queue.empty())
    {
        const auto current = queue.top();
        queue.pop();
        ++queue_pops;

        if (current.cost != lower_bounds[current.node])
        {
            continue;
        }

        const auto level = query_level_policy.GetNodeQueryLevel(current.node, restriction);

        if (level >= 1 && !from_clique_arc[current.node])
        {
            const auto cell_id = partition.GetCell(level, current.node);
            const auto cell = cells.GetUnfilledCell(level, cell_id);

            for (const auto predecessor : cell.GetSourceNodes())
            {
                ++clique_candidates;
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
                    ++lower_bound_updates;
                }
            }
        }

        ForEachDescendingLevel(
            level,
            [&](const LevelID edge_level)
            {
                for (const auto edge : facade.GetBorderEdgeRange(edge_level, current.node))
                {
                    ++border_edge_candidates;
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
                        ++lower_bound_updates;
                    }
                }
            });
    }

    if (timing_enabled)
    {
        const auto reverse_search_stop = std::chrono::steady_clock::now();
        util::Log(logDEBUG)
            << "[overlay query] reverse_lower_bounds restricted=" << restriction.restricted
            << " level=" << static_cast<int>(restriction.level)
            << " parent_cell=" << restriction.parent_cell
            << " target_node=" << target.node
            << " ms="
            << OverlayDurationMs(reverse_search_stop - reverse_search_start)
            << " queue_pops=" << queue_pops
            << " clique_candidates=" << clique_candidates
            << " border_edge_candidates=" << border_edge_candidates
            << " lower_bound_updates=" << lower_bound_updates;
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
    const PairQueryLevelPolicy pair_query_level_policy{
        facade.GetMultiLevelPartition(), source, target};
    return SearchAtClock(
        facade, source, target, departure_clock, restriction, pair_query_level_policy);
}

template <typename FacadeT, typename QueryLevelPolicyT>
TemporalOverlayPath SearchAtClock(const FacadeT &facade,
                                  const DirectedPhantomEndpoint &source,
                                  const DirectedPhantomEndpoint &target,
                                  const engine::temporal::TemporalClock departure_clock,
                                  const SearchRestriction &restriction,
                                  const QueryLevelPolicyT &query_level_policy)
{
    const auto timing_enabled = OverlayTimingEnabled();
    const auto search_start = std::chrono::steady_clock::now();
    TemporalOverlayPath best_path;

    if (source.node == target.node)
    {
        best_path.total_duration =
            temporal::detail::EvaluateLocalPathDuration(facade, source, target, departure_clock);
        if (timing_enabled)
        {
            const auto search_stop = std::chrono::steady_clock::now();
            util::Log(logDEBUG)
                << "[overlay query] search restricted=" << restriction.restricted
                << " level=" << static_cast<int>(restriction.level)
                << " parent_cell=" << restriction.parent_cell
                << " source_node=" << source.node
                << " target_node=" << target.node
                << " ms=" << OverlayDurationMs(search_stop - search_start)
                << " reverse_ms=0 forward_ms=0 queue_pops=0 shortcut_candidates=0"
                << " border_edge_candidates=0 best_updates=1 result=local";
        }
        return best_path;
    }

    const auto reverse_lower_bounds_start = std::chrono::steady_clock::now();
    /*
    const auto reverse_lower_bounds =
        ComputeReverseLowerBounds(facade, target, restriction, query_level_policy);
    const auto reverse_lower_bounds_stop = std::chrono::steady_clock::now();
    if (source.node >= reverse_lower_bounds.size() ||
        reverse_lower_bounds[source.node] == INVALID_EDGE_DURATION)
    {
        if (timing_enabled)
        {
            const auto search_stop = std::chrono::steady_clock::now();
            util::Log(logDEBUG)
                << "[overlay query] search restricted=" << restriction.restricted
                << " level=" << static_cast<int>(restriction.level)
                << " parent_cell=" << restriction.parent_cell
                << " source_node=" << source.node
                << " target_node=" << target.node
                << " ms=" << OverlayDurationMs(search_stop - search_start)
                << " reverse_ms="
                << OverlayDurationMs(reverse_lower_bounds_stop - reverse_lower_bounds_start)
                << " forward_ms=0 queue_pops=0 shortcut_candidates=0"
                << " border_edge_candidates=0 best_updates=0 result=no_reverse_path";
        }
        return best_path;
    }
    */
    const auto reverse_lower_bounds_stop = reverse_lower_bounds_start;

    const auto &partition = facade.GetMultiLevelPartition();
    const auto &cells = facade.GetCellStorage();

    std::vector<EdgeDuration> settled_costs(facade.GetNumberOfNodes(), INVALID_EDGE_DURATION);
    std::vector<NodeID> parents(facade.GetNumberOfNodes(), SPECIAL_NODEID);
    std::vector<bool> from_clique_arc(facade.GetNumberOfNodes(), false);
    std::vector<LevelID> overlay_levels(facade.GetNumberOfNodes(), INVALID_LEVEL_ID);
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> queue;
    std::size_t queue_pops = 0;
    std::size_t shortcut_candidates = 0;
    std::size_t border_edge_candidates = 0;
    std::size_t best_updates = 0;

    settled_costs[source.node] = EdgeDuration{0};
    parents[source.node] = source.node;
    queue.push({EdgeDuration{0}, source.node});

    const auto forward_search_start = std::chrono::steady_clock::now();
    while (!queue.empty())
    {
        const auto current = queue.top();
        queue.pop();
        ++queue_pops;

        if (current.cost != settled_costs[current.node])
        {
            continue;
        }

        /*
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
        */

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
                ++best_updates;
            }
        }

        const auto current_clock =
            engine::temporal::AdvanceTemporalClock(departure_clock, current.cost);
        const auto current_level =
            query_level_policy.GetNodeQueryLevel(current.node, restriction);

        if (current_level >= 1 && !from_clique_arc[current.node])
        {
            const auto cell_id = partition.GetCell(current_level, current.node);
            const auto week_bucket = GetWeekBucketForClock(facade, current_clock);
            const auto shortcut_row =
                facade.GetTemporalShortcutRow(current_level, cell_id, current.node);

            for (std::size_t destination_index = 0; destination_index < shortcut_row.size;
                 ++destination_index)
            {
                ++shortcut_candidates;
                const auto destination = shortcut_row.destinations[destination_index];
                if (destination == current.node ||
                    !CheckParentCellRestriction(partition, current_level, destination, restriction))
                {
                    continue;
                }

                const auto shortcut_min_duration = shortcut_row.min_durations[destination_index];
                const auto function_id = shortcut_row.function_ids[destination_index];
                if (shortcut_min_duration == INVALID_EDGE_DURATION ||
                    function_id == customizer::INVALID_TEMPORAL_FUNCTION_ID)
                {
                    continue;
                }

                const auto shortcut_duration =
                    facade.GetTemporalFunctionDuration(function_id, week_bucket);
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
                    ++border_edge_candidates;
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

    if (timing_enabled)
    {
        const auto forward_search_stop = std::chrono::steady_clock::now();
        const auto search_stop = forward_search_stop;
        util::Log(logDEBUG)
            << "[overlay query] search restricted=" << restriction.restricted
            << " level=" << static_cast<int>(restriction.level)
            << " parent_cell=" << restriction.parent_cell
            << " source_node=" << source.node
            << " target_node=" << target.node
            << " ms=" << OverlayDurationMs(search_stop - search_start)
            << " reverse_ms="
            << OverlayDurationMs(reverse_lower_bounds_stop - reverse_lower_bounds_start)
            << " forward_ms=" << OverlayDurationMs(forward_search_stop - forward_search_start)
            << " queue_pops=" << queue_pops
            << " shortcut_candidates=" << shortcut_candidates
            << " border_edge_candidates=" << border_edge_candidates
            << " best_updates=" << best_updates
            << " valid=" << best_path.is_valid();
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
    const PairQueryLevelPolicy pair_query_level_policy{
        facade.GetMultiLevelPartition(), source, target};
    return SearchImpl(
        facade, source, target, departure_timestamp, restriction, pair_query_level_policy);
}

template <typename FacadeT, typename QueryLevelPolicyT>
TemporalOverlayPath SearchImpl(const FacadeT &facade,
                               const DirectedPhantomEndpoint &source,
                               const DirectedPhantomEndpoint &target,
                               const std::time_t departure_timestamp,
                               const SearchRestriction &restriction,
                               const QueryLevelPolicyT &query_level_policy)
{
    return SearchAtClock(
        facade,
        source,
        target,
        engine::temporal::ToTemporalClock(departure_timestamp),
        restriction,
        query_level_policy);
}

template <typename FacadeT>
TemporalOverlayUnpackedPath UnpackPathImpl(const FacadeT &facade,
                                           const DirectedPhantomEndpoint &source,
                                           const DirectedPhantomEndpoint &target,
                                           const engine::temporal::TemporalClock departure_clock,
                                           const TemporalOverlayPath &path,
                                           const SearchRestriction &restriction)
{
    const auto timing_enabled = OverlayTimingEnabled();
    const auto unpack_start = std::chrono::steady_clock::now();
    TemporalOverlayUnpackedPath unpacked;
    if (!path.is_valid())
    {
        if (timing_enabled)
        {
            const auto unpack_stop = std::chrono::steady_clock::now();
            util::Log(logDEBUG)
                << "[overlay query] unpack restricted=" << restriction.restricted
                << " level=" << static_cast<int>(restriction.level)
                << " parent_cell=" << restriction.parent_cell
                << " source_node=" << source.node
                << " target_node=" << target.node
                << " ms=" << OverlayDurationMs(unpack_stop - unpack_start)
                << " base_edge_ms=0 recursive_search_ms=0 recursive_unpack_ms=0"
                << " target_offset_ms=0 packed_edges=0 overlay_edges=0 valid=0";
        }
        return unpacked;
    }

    if (path.packed_path.empty())
    {
        unpacked.total_duration = temporal::detail::EvaluateLocalPathDuration(
            facade, source, target, departure_clock);
        unpacked.nodes = {source.node};
        if (timing_enabled)
        {
            const auto unpack_stop = std::chrono::steady_clock::now();
            util::Log(logDEBUG)
                << "[overlay query] unpack restricted=" << restriction.restricted
                << " level=" << static_cast<int>(restriction.level)
                << " parent_cell=" << restriction.parent_cell
                << " source_node=" << source.node
                << " target_node=" << target.node
                << " ms=" << OverlayDurationMs(unpack_stop - unpack_start)
                << " base_edge_ms=0 recursive_search_ms=0 recursive_unpack_ms=0"
                << " target_offset_ms=0 packed_edges=0 overlay_edges=0 valid=1 local=1";
        }
        return unpacked;
    }

    const auto &partition = facade.GetMultiLevelPartition();
    auto current_clock = departure_clock;
    auto first_step = true;
    std::chrono::steady_clock::duration base_edge_duration{0};
    std::chrono::steady_clock::duration recursive_search_duration{0};
    std::chrono::steady_clock::duration recursive_unpack_duration{0};
    std::chrono::steady_clock::duration target_offset_duration{0};
    std::size_t overlay_edge_count = 0;

    unpacked.nodes.push_back(path.packed_path.front().from);
    for (const auto &packed_edge : path.packed_path)
    {
        const auto from = packed_edge.from;
        const auto to = packed_edge.to;

        if (!packed_edge.is_overlay)
        {
            const auto base_edge_start = std::chrono::steady_clock::now();
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
            base_edge_duration += std::chrono::steady_clock::now() - base_edge_start;
            continue;
        }

        ++overlay_edge_count;
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
        const auto recursive_search_start = std::chrono::steady_clock::now();
        const auto sub_path =
            SearchAtClock(facade, sub_source, sub_target, current_clock, sub_restriction);
        recursive_search_duration += std::chrono::steady_clock::now() - recursive_search_start;
        if (!sub_path.is_valid())
        {
            return {};
        }

        const auto recursive_unpack_start = std::chrono::steady_clock::now();
        const auto unpacked_subpath =
            UnpackPathImpl(facade, sub_source, sub_target, current_clock, sub_path, sub_restriction);
        recursive_unpack_duration += std::chrono::steady_clock::now() - recursive_unpack_start;
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

    const auto target_offset_start = std::chrono::steady_clock::now();
    const auto target_duration =
        temporal::detail::GetPhantomTraversalDurationAtClock(facade, target, current_clock);
    if (target_duration == INVALID_EDGE_DURATION)
    {
        return {};
    }

    current_clock = engine::temporal::AdvanceTemporalClock(current_clock, target_duration);
    unpacked.total_duration = to_alias<EdgeDuration>(current_clock - departure_clock);
    target_offset_duration += std::chrono::steady_clock::now() - target_offset_start;

    if (timing_enabled)
    {
        const auto unpack_stop = std::chrono::steady_clock::now();
        util::Log(logDEBUG)
            << "[overlay query] unpack restricted=" << restriction.restricted
            << " level=" << static_cast<int>(restriction.level)
            << " parent_cell=" << restriction.parent_cell
            << " source_node=" << source.node
            << " target_node=" << target.node
            << " ms=" << OverlayDurationMs(unpack_stop - unpack_start)
            << " base_edge_ms=" << OverlayDurationMs(base_edge_duration)
            << " recursive_search_ms=" << OverlayDurationMs(recursive_search_duration)
            << " recursive_unpack_ms=" << OverlayDurationMs(recursive_unpack_duration)
            << " target_offset_ms=" << OverlayDurationMs(target_offset_duration)
            << " packed_edges=" << path.packed_path.size()
            << " overlay_edges=" << overlay_edge_count
            << " valid=1";
    }

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

template <typename FacadeT, typename QueryLevelPolicyT>
TemporalOverlayPath Search(const FacadeT &facade,
                           const DirectedPhantomEndpoint &source,
                           const DirectedPhantomEndpoint &target,
                           const std::time_t departure_timestamp,
                           const QueryLevelPolicyT &query_level_policy)
{
    return detail::SearchImpl(facade,
                              source,
                              target,
                              departure_timestamp,
                              detail::SearchRestriction{},
                              query_level_policy);
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
