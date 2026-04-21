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
#include <unordered_map>
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

struct ReverseLowerBoundSearchStats
{
    std::size_t queue_pops = 0;
    std::size_t clique_candidates = 0;
    std::size_t border_edge_candidates = 0;
    std::size_t lower_bound_updates = 0;
};

struct ReverseLowerBoundWorkspace
{
    std::vector<EdgeDuration> lower_bounds;
    std::vector<bool> from_clique_arc;
    std::vector<NodeID> touched_nodes;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> queue;

    ReverseLowerBoundWorkspace() = default;
    explicit ReverseLowerBoundWorkspace(const std::size_t number_of_nodes)
        : lower_bounds(number_of_nodes, INVALID_EDGE_DURATION),
          from_clique_arc(number_of_nodes, false)
    {
    }

    void Reset(const std::size_t number_of_nodes)
    {
        if (lower_bounds.size() != number_of_nodes)
        {
            lower_bounds.assign(number_of_nodes, INVALID_EDGE_DURATION);
            from_clique_arc.assign(number_of_nodes, false);
        }
        else
        {
            for (const auto node : touched_nodes)
            {
                if (node >= lower_bounds.size())
                {
                    continue;
                }

                lower_bounds[node] = INVALID_EDGE_DURATION;
                from_clique_arc[node] = false;
            }
        }

        touched_nodes.clear();
        queue = decltype(queue){};
    }
};

struct ReverseLowerBoundCacheKey
{
    NodeID target_node = SPECIAL_NODEID;
    bool target_traversed_in_reverse = false;
    bool restricted = false;
    LevelID level = INVALID_LEVEL_ID;
    CellID parent_cell = INVALID_CELL_ID;

    bool operator==(const ReverseLowerBoundCacheKey &other) const
    {
        return target_node == other.target_node &&
               target_traversed_in_reverse == other.target_traversed_in_reverse &&
               restricted == other.restricted && level == other.level &&
               parent_cell == other.parent_cell;
    }
};

struct ReverseLowerBoundCacheKeyHasher
{
    std::size_t operator()(const ReverseLowerBoundCacheKey &key) const noexcept
    {
        std::size_t seed = std::hash<std::uint64_t>{}(static_cast<std::uint64_t>(key.target_node));
        seed ^= std::hash<bool>{}(key.target_traversed_in_reverse) + 0x9e3779b9 + (seed << 6) +
                (seed >> 2);
        seed ^= std::hash<bool>{}(key.restricted) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<std::uint64_t>{}(static_cast<std::uint64_t>(key.level)) + 0x9e3779b9 +
                (seed << 6) + (seed >> 2);
        seed ^= std::hash<std::uint64_t>{}(static_cast<std::uint64_t>(key.parent_cell)) +
                0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

class ReverseLowerBoundCache
{
  public:
    ReverseLowerBoundCache() = default;
    explicit ReverseLowerBoundCache(const std::size_t number_of_nodes) : workspace(number_of_nodes)
    {
    }

    template <typename FacadeT, typename QueryLevelPolicyT>
    const std::vector<EdgeDuration> &GetOrCompute(const FacadeT &facade,
                                                  const DirectedPhantomEndpoint &target,
                                                  const SearchRestriction &restriction,
                                                  const QueryLevelPolicyT &query_level_policy,
                                                  ReverseLowerBoundSearchStats *stats = nullptr)
    {
        const auto key = ReverseLowerBoundCacheKey{
            target.node, target.traversed_in_reverse, restriction.restricted, restriction.level,
            restriction.parent_cell};

        const auto cached = entries.find(key);
        if (cached != entries.end())
        {
            ++hit_count;
            if (stats != nullptr)
            {
                *stats = {};
            }
            return cached->second;
        }

        ++miss_count;
        ReverseLowerBoundSearchStats local_stats;
        auto &active_stats = stats != nullptr ? *stats : local_stats;
        active_stats = {};

        const auto &computed = ComputeReverseLowerBounds(
            facade, target, restriction, query_level_policy, workspace, &active_stats);
        auto [inserted, success] =
            entries.emplace(key, std::vector<EdgeDuration>(computed.begin(), computed.end()));
        (void)success;
        return inserted->second;
    }

    std::size_t GetHitCount() const { return hit_count; }
    std::size_t GetMissCount() const { return miss_count; }
    std::size_t GetEntryCount() const { return entries.size(); }

  private:
    ReverseLowerBoundWorkspace workspace;
    std::unordered_map<ReverseLowerBoundCacheKey,
                       std::vector<EdgeDuration>,
                       ReverseLowerBoundCacheKeyHasher>
        entries;
    std::size_t hit_count = 0;
    std::size_t miss_count = 0;
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

inline void UpdateReverseLowerBound(ReverseLowerBoundWorkspace &workspace,
                                    ReverseLowerBoundSearchStats &stats,
                                    const NodeID node,
                                    const EdgeDuration candidate,
                                    const bool via_clique_arc)
{
    if (node >= workspace.lower_bounds.size())
    {
        return;
    }

    if (workspace.lower_bounds[node] == INVALID_EDGE_DURATION)
    {
        workspace.touched_nodes.push_back(node);
    }

    workspace.lower_bounds[node] = candidate;
    workspace.from_clique_arc[node] = via_clique_arc;
    workspace.queue.push({candidate, node});
    ++stats.lower_bound_updates;
}

template <typename FacadeT>
void SeedReverseLowerBoundTarget(const FacadeT &facade,
                                 const DirectedPhantomEndpoint &target,
                                 ReverseLowerBoundWorkspace &workspace,
                                 ReverseLowerBoundSearchStats &stats)
{
    if (target.node >= workspace.lower_bounds.size())
    {
        return;
    }

    const auto target_lower_bound = temporal::detail::GetPhantomTraversalLowerBound(facade, target);
    if (target_lower_bound == INVALID_EDGE_DURATION)
    {
        return;
    }

    if (workspace.lower_bounds[target.node] != INVALID_EDGE_DURATION &&
        target_lower_bound >= workspace.lower_bounds[target.node])
    {
        return;
    }

    UpdateReverseLowerBound(workspace, stats, target.node, target_lower_bound, false);
}

template <typename FacadeT>
void InitializeReverseLowerBounds(const FacadeT &facade,
                                  const DirectedPhantomEndpoint &target,
                                  ReverseLowerBoundWorkspace &workspace,
                                  ReverseLowerBoundSearchStats &stats)
{
    workspace.Reset(facade.GetNumberOfNodes());
    SeedReverseLowerBoundTarget(facade, target, workspace, stats);
}

template <typename FacadeT>
void InitializeReverseLowerBoundsForTargets(
    const FacadeT &facade,
    const std::vector<DirectedPhantomEndpoint> &targets,
    ReverseLowerBoundWorkspace &workspace,
    ReverseLowerBoundSearchStats &stats)
{
    workspace.Reset(facade.GetNumberOfNodes());

    for (const auto &target : targets)
    {
        SeedReverseLowerBoundTarget(facade, target, workspace, stats);
    }
}

template <typename FacadeT, typename QueryLevelPolicyT>
void RelaxReverseShortcutPredecessors(const FacadeT &facade,
                                      const NodeID current_node,
                                      const EdgeDuration current_cost,
                                      const LevelID level,
                                      const SearchRestriction &restriction,
                                      const QueryLevelPolicyT &query_level_policy,
                                      ReverseLowerBoundWorkspace &workspace,
                                      ReverseLowerBoundSearchStats &stats)
{
    (void)query_level_policy;

    if (level < 1 || current_node >= workspace.from_clique_arc.size() ||
        workspace.from_clique_arc[current_node])
    {
        return;
    }

    const auto &partition = facade.GetMultiLevelPartition();
    const auto &cells = facade.GetCellStorage();
    const auto cell_id = partition.GetCell(level, current_node);
    const auto cell = cells.GetUnfilledCell(level, cell_id);

    for (const auto predecessor : cell.GetSourceNodes())
    {
        ++stats.clique_candidates;
        if (predecessor == current_node ||
            !CheckParentCellRestriction(partition, level, predecessor, restriction))
        {
            continue;
        }

        const auto shortcut_min =
            facade.GetTemporalShortcutMinDuration(level, cell_id, predecessor, current_node);
        const auto candidate = temporal::detail::SafeDurationAdd(current_cost, shortcut_min);

        if (candidate == INVALID_EDGE_DURATION)
        {
            continue;
        }

        if (workspace.lower_bounds[predecessor] == INVALID_EDGE_DURATION ||
            candidate < workspace.lower_bounds[predecessor])
        {
            UpdateReverseLowerBound(workspace, stats, predecessor, candidate, true);
        }
    }
}

template <typename FacadeT>
void RelaxReverseBorderPredecessors(const FacadeT &facade,
                                    const NodeID current_node,
                                    const EdgeDuration current_cost,
                                    const LevelID level,
                                    const SearchRestriction &restriction,
                                    ReverseLowerBoundWorkspace &workspace,
                                    ReverseLowerBoundSearchStats &stats)
{
    const auto &partition = facade.GetMultiLevelPartition();

    ForEachDescendingLevel(
        level,
        [&](const LevelID edge_level)
        {
            for (const auto edge : facade.GetBorderEdgeRange(edge_level, current_node))
            {
                ++stats.border_edge_candidates;
                if (!facade.IsBackwardEdge(edge))
                {
                    continue;
                }

                const auto predecessor = facade.GetTarget(edge);
                if (facade.ExcludeNode(predecessor) ||
                    !CheckParentCellRestriction(partition, edge_level, predecessor, restriction))
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
                    temporal::detail::SafeDurationAdd(current_cost, edge_cost);

                if (candidate == INVALID_EDGE_DURATION)
                {
                    continue;
                }

                if (workspace.lower_bounds[predecessor] == INVALID_EDGE_DURATION ||
                    candidate < workspace.lower_bounds[predecessor])
                {
                    UpdateReverseLowerBound(workspace, stats, predecessor, candidate, false);
                }
            }
        });
}

template <typename FacadeT, typename QueryLevelPolicyT>
void ReverseLowerBoundStep(const FacadeT &facade,
                           const SearchRestriction &restriction,
                           const QueryLevelPolicyT &query_level_policy,
                           ReverseLowerBoundWorkspace &workspace,
                           ReverseLowerBoundSearchStats &stats)
{
    const auto current = workspace.queue.top();
    workspace.queue.pop();
    ++stats.queue_pops;

    if (current.node >= workspace.lower_bounds.size() ||
        current.cost != workspace.lower_bounds[current.node])
    {
        return;
    }

    const auto level = query_level_policy.GetNodeQueryLevel(current.node, restriction);
    RelaxReverseShortcutPredecessors(
        facade, current.node, current.cost, level, restriction, query_level_policy, workspace, stats);
    RelaxReverseBorderPredecessors(
        facade, current.node, current.cost, level, restriction, workspace, stats);
}

template <typename FacadeT, typename QueryLevelPolicyT>
const std::vector<EdgeDuration> &
RunReverseLowerBoundSearch(const FacadeT &facade,
                           const DirectedPhantomEndpoint &target,
                           const SearchRestriction &restriction,
                           const QueryLevelPolicyT &query_level_policy,
                           ReverseLowerBoundWorkspace &workspace,
                           ReverseLowerBoundSearchStats &stats)
{
    InitializeReverseLowerBounds(facade, target, workspace, stats);

    while (!workspace.queue.empty())
    {
        ReverseLowerBoundStep(facade, restriction, query_level_policy, workspace, stats);
    }

    return workspace.lower_bounds;
}

template <typename FacadeT, typename QueryLevelPolicyT>
const std::vector<EdgeDuration> &
RunReverseLowerBoundSearch(const FacadeT &facade,
                           const std::vector<DirectedPhantomEndpoint> &targets,
                           const SearchRestriction &restriction,
                           const QueryLevelPolicyT &query_level_policy,
                           ReverseLowerBoundWorkspace &workspace,
                           ReverseLowerBoundSearchStats &stats)
{
    InitializeReverseLowerBoundsForTargets(facade, targets, workspace, stats);

    while (!workspace.queue.empty())
    {
        ReverseLowerBoundStep(facade, restriction, query_level_policy, workspace, stats);
    }

    return workspace.lower_bounds;
}

template <typename FacadeT, typename QueryLevelPolicyT>
std::vector<EdgeDuration> ComputeReverseLowerBounds(const FacadeT &facade,
                                                    const DirectedPhantomEndpoint &target,
                                                    const SearchRestriction &restriction,
                                                    const QueryLevelPolicyT &query_level_policy);

template <typename FacadeT, typename QueryLevelPolicyT>
std::vector<EdgeDuration>
ComputeReverseLowerBoundsForTargets(const FacadeT &facade,
                                    const std::vector<DirectedPhantomEndpoint> &targets,
                                    const SearchRestriction &restriction,
                                    const QueryLevelPolicyT &query_level_policy);

template <typename FacadeT, typename QueryLevelPolicyT>
const std::vector<EdgeDuration> &ComputeReverseLowerBounds(const FacadeT &facade,
                                                           const DirectedPhantomEndpoint &target,
                                                           const SearchRestriction &restriction,
                                                           const QueryLevelPolicyT &query_level_policy,
                                                           ReverseLowerBoundWorkspace &workspace,
                                                           ReverseLowerBoundSearchStats *stats = nullptr);

template <typename FacadeT, typename QueryLevelPolicyT>
const std::vector<EdgeDuration> &
ComputeReverseLowerBoundsForTargets(const FacadeT &facade,
                                    const std::vector<DirectedPhantomEndpoint> &targets,
                                    const SearchRestriction &restriction,
                                    const QueryLevelPolicyT &query_level_policy,
                                    ReverseLowerBoundWorkspace &workspace,
                                    ReverseLowerBoundSearchStats *stats = nullptr);

template <typename FacadeT, typename QueryLevelPolicyT>
const std::vector<EdgeDuration> &ComputeReverseLowerBounds(const FacadeT &facade,
                                                           const DirectedPhantomEndpoint &target,
                                                           const SearchRestriction &restriction,
                                                           const QueryLevelPolicyT &query_level_policy,
                                                           ReverseLowerBoundCache &cache,
                                                           ReverseLowerBoundSearchStats *stats = nullptr);

template <typename FacadeT, typename QueryLevelPolicyT>
TemporalOverlayPath SearchAtClock(const FacadeT &facade,
                                  const DirectedPhantomEndpoint &source,
                                  const DirectedPhantomEndpoint &target,
                                  const engine::temporal::TemporalClock departure_clock,
                                  const SearchRestriction &restriction,
                                  const QueryLevelPolicyT &query_level_policy,
                                  const std::vector<EdgeDuration> &reverse_lower_bounds);

template <typename FacadeT, typename QueryLevelPolicyT>
TemporalOverlayPath SearchAtClock(const FacadeT &facade,
                                  const DirectedPhantomEndpoint &source,
                                  const DirectedPhantomEndpoint &target,
                                  const engine::temporal::TemporalClock departure_clock,
                                  const SearchRestriction &restriction,
                                  const QueryLevelPolicyT &query_level_policy,
                                  ReverseLowerBoundCache *reverse_lower_bound_cache = nullptr);

template <typename FacadeT, typename QueryLevelPolicyT>
TemporalOverlayPath SearchImpl(const FacadeT &facade,
                               const DirectedPhantomEndpoint &source,
                               const DirectedPhantomEndpoint &target,
                               const std::time_t departure_timestamp,
                               const SearchRestriction &restriction,
                               const QueryLevelPolicyT &query_level_policy,
                               const std::vector<EdgeDuration> &reverse_lower_bounds);

template <typename FacadeT, typename QueryLevelPolicyT>
TemporalOverlayPath SearchImpl(const FacadeT &facade,
                               const DirectedPhantomEndpoint &source,
                               const DirectedPhantomEndpoint &target,
                               const std::time_t departure_timestamp,
                               const SearchRestriction &restriction,
                               const QueryLevelPolicyT &query_level_policy,
                               ReverseLowerBoundCache *reverse_lower_bound_cache = nullptr);

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
    ReverseLowerBoundWorkspace workspace{facade.GetNumberOfNodes()};
    ReverseLowerBoundSearchStats stats;
    const auto &lower_bounds = ComputeReverseLowerBounds(
        facade, target, restriction, query_level_policy, workspace, &stats);

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
            << " queue_pops=" << stats.queue_pops
            << " clique_candidates=" << stats.clique_candidates
            << " border_edge_candidates=" << stats.border_edge_candidates
            << " lower_bound_updates=" << stats.lower_bound_updates;
    }

    return lower_bounds;
}

template <typename FacadeT, typename QueryLevelPolicyT>
std::vector<EdgeDuration>
ComputeReverseLowerBoundsForTargets(const FacadeT &facade,
                                    const std::vector<DirectedPhantomEndpoint> &targets,
                                    const SearchRestriction &restriction,
                                    const QueryLevelPolicyT &query_level_policy)
{
    ReverseLowerBoundWorkspace workspace{facade.GetNumberOfNodes()};
    return ComputeReverseLowerBoundsForTargets(
        facade, targets, restriction, query_level_policy, workspace);
}

template <typename FacadeT, typename QueryLevelPolicyT>
const std::vector<EdgeDuration> &ComputeReverseLowerBounds(
    const FacadeT &facade,
    const DirectedPhantomEndpoint &target,
    const SearchRestriction &restriction,
    const QueryLevelPolicyT &query_level_policy,
    ReverseLowerBoundWorkspace &workspace,
    ReverseLowerBoundSearchStats *stats)
{
    ReverseLowerBoundSearchStats local_stats;
    auto &active_stats = stats != nullptr ? *stats : local_stats;
    active_stats = {};

    return RunReverseLowerBoundSearch(
        facade, target, restriction, query_level_policy, workspace, active_stats);
}

template <typename FacadeT, typename QueryLevelPolicyT>
const std::vector<EdgeDuration> &
ComputeReverseLowerBoundsForTargets(const FacadeT &facade,
                                    const std::vector<DirectedPhantomEndpoint> &targets,
                                    const SearchRestriction &restriction,
                                    const QueryLevelPolicyT &query_level_policy,
                                    ReverseLowerBoundWorkspace &workspace,
                                    ReverseLowerBoundSearchStats *stats)
{
    ReverseLowerBoundSearchStats local_stats;
    auto &active_stats = stats != nullptr ? *stats : local_stats;
    active_stats = {};

    return RunReverseLowerBoundSearch(
        facade, targets, restriction, query_level_policy, workspace, active_stats);
}

template <typename FacadeT, typename QueryLevelPolicyT>
const std::vector<EdgeDuration> &ComputeReverseLowerBounds(
    const FacadeT &facade,
    const DirectedPhantomEndpoint &target,
    const SearchRestriction &restriction,
    const QueryLevelPolicyT &query_level_policy,
    ReverseLowerBoundCache &cache,
    ReverseLowerBoundSearchStats *stats)
{
    return cache.GetOrCompute(facade, target, restriction, query_level_policy, stats);
}

template <typename FacadeT>
TemporalOverlayPath SearchAtClock(const FacadeT &facade,
                                  const DirectedPhantomEndpoint &source,
                                  const DirectedPhantomEndpoint &target,
                                  const engine::temporal::TemporalClock departure_clock,
                                  const SearchRestriction &restriction,
                                  ReverseLowerBoundCache *reverse_lower_bound_cache = nullptr)
{
    const PairQueryLevelPolicy pair_query_level_policy{
        facade.GetMultiLevelPartition(), source, target};
    return SearchAtClock(
        facade,
        source,
        target,
        departure_clock,
        restriction,
        pair_query_level_policy,
        reverse_lower_bound_cache);
}

template <typename FacadeT, typename QueryLevelPolicyT>
TemporalOverlayPath SearchAtClockWithReverseLowerBounds(
    const FacadeT &facade,
    const DirectedPhantomEndpoint &source,
    const DirectedPhantomEndpoint &target,
    const engine::temporal::TemporalClock departure_clock,
    const SearchRestriction &restriction,
    const QueryLevelPolicyT &query_level_policy,
    const std::vector<EdgeDuration> &reverse_lower_bounds,
    const double reverse_search_ms,
    const bool shared_reverse_bounds,
    const std::chrono::steady_clock::time_point search_start)
{
    const auto timing_enabled = OverlayTimingEnabled();
    TemporalOverlayPath best_path;

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
                << " reverse_ms=" << reverse_search_ms
                << " shared_reverse_bounds=" << shared_reverse_bounds
                << " forward_ms=0 queue_pops=0 shortcut_candidates=0"
                << " border_edge_candidates=0 best_updates=0 result=no_reverse_path";
        }
        return best_path;
    }

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
            << " reverse_ms=" << reverse_search_ms
            << " shared_reverse_bounds=" << shared_reverse_bounds
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
TemporalOverlayPath SearchAtClock(const FacadeT &facade,
                                  const DirectedPhantomEndpoint &source,
                                  const DirectedPhantomEndpoint &target,
                                  const engine::temporal::TemporalClock departure_clock,
                                  const SearchRestriction &restriction,
                                  const QueryLevelPolicyT &query_level_policy,
                                  const std::vector<EdgeDuration> &reverse_lower_bounds)
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
                << " reverse_ms=0 shared_reverse_bounds=0 forward_ms=0 queue_pops=0 shortcut_candidates=0"
                << " border_edge_candidates=0 best_updates=1 result=local";
        }
        return best_path;
    }

    return SearchAtClockWithReverseLowerBounds(facade,
                                               source,
                                               target,
                                               departure_clock,
                                               restriction,
                                               query_level_policy,
                                               reverse_lower_bounds,
                                               0.,
                                               true,
                                               search_start);
}

template <typename FacadeT, typename QueryLevelPolicyT>
TemporalOverlayPath SearchAtClock(const FacadeT &facade,
                                  const DirectedPhantomEndpoint &source,
                                  const DirectedPhantomEndpoint &target,
                                  const engine::temporal::TemporalClock departure_clock,
                                  const SearchRestriction &restriction,
                                  const QueryLevelPolicyT &query_level_policy,
                                  ReverseLowerBoundCache *reverse_lower_bound_cache)
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
                << " reverse_ms=0 shared_reverse_bounds=0 forward_ms=0 queue_pops=0 shortcut_candidates=0"
                << " border_edge_candidates=0 best_updates=1 result=local";
        }
        return best_path;
    }

    const auto reverse_lower_bounds_start = std::chrono::steady_clock::now();
    ReverseLowerBoundWorkspace reverse_lower_bound_workspace;
    const auto &reverse_lower_bounds =
        restriction.restricted && reverse_lower_bound_cache != nullptr
            ? ComputeReverseLowerBounds(
                  facade, target, restriction, query_level_policy, *reverse_lower_bound_cache)
            : ComputeReverseLowerBounds(
                  facade, target, restriction, query_level_policy, reverse_lower_bound_workspace);
    const auto reverse_lower_bounds_stop = std::chrono::steady_clock::now();

    return SearchAtClockWithReverseLowerBounds(
        facade,
        source,
        target,
        departure_clock,
        restriction,
        query_level_policy,
        reverse_lower_bounds,
        OverlayDurationMs(reverse_lower_bounds_stop - reverse_lower_bounds_start),
        false,
        search_start);
}

template <typename FacadeT, typename QueryLevelPolicyT>
TemporalOverlayPath SearchImpl(const FacadeT &facade,
                               const DirectedPhantomEndpoint &source,
                               const DirectedPhantomEndpoint &target,
                               const std::time_t departure_timestamp,
                               const SearchRestriction &restriction,
                               const QueryLevelPolicyT &query_level_policy,
                               const std::vector<EdgeDuration> &reverse_lower_bounds)
{
    return SearchAtClock(facade,
                         source,
                         target,
                         engine::temporal::ToTemporalClock(departure_timestamp),
                         restriction,
                         query_level_policy,
                         reverse_lower_bounds);
}

template <typename FacadeT, typename QueryLevelPolicyT>
TemporalOverlayPath SearchImpl(const FacadeT &facade,
                               const DirectedPhantomEndpoint &source,
                               const DirectedPhantomEndpoint &target,
                               const std::time_t departure_timestamp,
                               const SearchRestriction &restriction,
                               const QueryLevelPolicyT &query_level_policy,
                               ReverseLowerBoundCache *reverse_lower_bound_cache)
{
    return SearchAtClock(
        facade,
        source,
        target,
        engine::temporal::ToTemporalClock(departure_timestamp),
        restriction,
        query_level_policy,
        reverse_lower_bound_cache);
}

template <typename FacadeT>
TemporalOverlayUnpackedPath UnpackPathImpl(const FacadeT &facade,
                                           const DirectedPhantomEndpoint &source,
                                           const DirectedPhantomEndpoint &target,
                                           const engine::temporal::TemporalClock departure_clock,
                                           const TemporalOverlayPath &path,
                                           const SearchRestriction &restriction,
                                           ReverseLowerBoundCache *reverse_lower_bound_cache =
                                               nullptr)
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
        const auto sub_path = SearchAtClock(
            facade, sub_source, sub_target, current_clock, sub_restriction, reverse_lower_bound_cache);
        recursive_search_duration += std::chrono::steady_clock::now() - recursive_search_start;
        if (!sub_path.is_valid())
        {
            return {};
        }

        const auto recursive_unpack_start = std::chrono::steady_clock::now();
        const auto unpacked_subpath = UnpackPathImpl(facade,
                                                     sub_source,
                                                     sub_target,
                                                     current_clock,
                                                     sub_path,
                                                     sub_restriction,
                                                     reverse_lower_bound_cache);
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

template <typename FacadeT, typename QueryLevelPolicyT>
TemporalOverlayPath Search(const FacadeT &facade,
                           const DirectedPhantomEndpoint &source,
                           const DirectedPhantomEndpoint &target,
                           const std::time_t departure_timestamp,
                           const QueryLevelPolicyT &query_level_policy,
                           const std::vector<EdgeDuration> &reverse_lower_bounds)
{
    return detail::SearchImpl(facade,
                              source,
                              target,
                              departure_timestamp,
                              detail::SearchRestriction{},
                              query_level_policy,
                              reverse_lower_bounds);
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

template <typename FacadeT>
TemporalOverlayUnpackedPath UnpackPath(const FacadeT &facade,
                                       const DirectedPhantomEndpoint &source,
                                       const DirectedPhantomEndpoint &target,
                                       const std::time_t departure_timestamp,
                                       const TemporalOverlayPath &path,
                                       detail::ReverseLowerBoundCache &reverse_lower_bound_cache)
{
    return detail::UnpackPathImpl(facade,
                                  source,
                                  target,
                                  engine::temporal::ToTemporalClock(departure_timestamp),
                                  path,
                                  detail::SearchRestriction{},
                                  &reverse_lower_bound_cache);
}

} // namespace osrm::engine::routing_algorithms::mld::temporal::overlay

#endif
