#include "engine/phantom_node.hpp"
#include "engine/routing_algorithms/routing_base_td_mld.hpp"
#include "engine/temporal_traffic.hpp"

#include <boost/test/unit_test.hpp>
#include <algorithm>
#include <array>
#include <functional>
#include <queue>
#include <ranges>
#include <utility>
#include <vector>

namespace
{
using namespace osrm;
using TemporalShortcutRowView =
    engine::datafacade::AlgorithmDataFacade<engine::datafacade::MLD>::TemporalShortcutRowView;
using TemporalIncomingShortcutRowView =
    engine::datafacade::AlgorithmDataFacade<engine::datafacade::MLD>::TemporalIncomingShortcutRowView;
using IncomingBorderEdgeRowView =
    engine::datafacade::AlgorithmDataFacade<engine::datafacade::MLD>::IncomingBorderEdgeRowView;

struct MockPartition
{
    std::uint8_t GetNumberOfLevels() const { return 2; }

    CellID GetCell(LevelID, NodeID) const { return 0; }

    LevelID GetQueryLevel(NodeID source, NodeID target, NodeID node) const
    {
        if ((source == 0 && target == 3) || (source == 3 && target == 0))
        {
            if (node == 0 || node == 3)
            {
                return 1;
            }
        }

        return 0;
    }
};

struct MockCell
{
    std::vector<NodeID> source_nodes;
    std::vector<NodeID> destination_nodes;

    auto GetSourceNodes() const
    {
        return std::ranges::subrange(source_nodes.begin(), source_nodes.end());
    }

    auto GetDestinationNodes() const
    {
        return std::ranges::subrange(destination_nodes.begin(), destination_nodes.end());
    }
};

struct MockCellStorage
{
    MockCell level_one_cell{{0}, {3}};
    MockCell empty_cell{{}, {}};

    const MockCell &GetUnfilledCell(LevelID level, CellID) const
    {
        return level == 1 ? level_one_cell : empty_cell;
    }
};

struct MockEdge
{
    struct EdgeData
    {
        EdgeID turn_id = 0;
    };

    NodeID target = SPECIAL_NODEID;
    bool forward = false;
    bool backward = false;
    EdgeData data{};
};

template <typename FacadeT>
void BuildIncomingBorderEdgeRows(const FacadeT &facade,
                                 std::vector<std::vector<EdgeID>> &incoming_border_edges,
                                 std::vector<std::vector<LevelID>> &incoming_border_levels)
{
    const auto number_of_nodes = static_cast<std::size_t>(facade.GetNumberOfNodes());
    const auto number_of_levels =
        static_cast<std::size_t>(facade.GetMultiLevelPartition().GetNumberOfLevels());

    incoming_border_edges.assign(number_of_nodes, {});
    incoming_border_levels.assign(number_of_nodes, {});

    for (NodeID node = 0; node < number_of_nodes; ++node)
    {
        std::vector<EdgeID> seen_edges;

        for (auto level_index = number_of_levels; level_index-- > 0;)
        {
            const auto level = static_cast<LevelID>(level_index);
            for (const auto edge : facade.GetBorderEdgeRange(level, node))
            {
                if (std::find(seen_edges.begin(), seen_edges.end(), edge) != seen_edges.end())
                {
                    continue;
                }

                seen_edges.push_back(edge);
                if (!facade.IsBackwardEdge(edge))
                {
                    continue;
                }

                incoming_border_edges[node].push_back(edge);
                incoming_border_levels[node].push_back(level);
            }
        }
    }
}

template <typename DerivedT> struct IncomingBorderEdgeIndexMixin
{
    IncomingBorderEdgeRowView GetIncomingBorderEdgeRow(const NodeID node) const
    {
        EnsureIncomingBorderEdgeRows();

        if (node >= incoming_border_edges.size() || incoming_border_edges[node].empty())
        {
            return {};
        }

        return {incoming_border_edges[node].data(),
                incoming_border_levels[node].data(),
                incoming_border_edges[node].size()};
    }

  private:
    void EnsureIncomingBorderEdgeRows() const
    {
        if (incoming_border_rows_initialized)
        {
            return;
        }

        const auto &derived = static_cast<const DerivedT &>(*this);
        BuildIncomingBorderEdgeRows(derived, incoming_border_edges, incoming_border_levels);
        incoming_border_rows_initialized = true;
    }

    mutable bool incoming_border_rows_initialized = false;
    mutable std::vector<std::vector<EdgeID>> incoming_border_edges;
    mutable std::vector<std::vector<LevelID>> incoming_border_levels;
};

struct MockFacade : IncomingBorderEdgeIndexMixin<MockFacade>
{
    MockPartition partition;
    MockCellStorage cells;
    std::vector<MockEdge> edges;
    std::array<std::vector<std::vector<EdgeID>>, 2> border_edges;
    std::vector<std::vector<SegmentDuration>> static_durations;
    std::vector<std::vector<EdgeDuration>> temporal_durations;
    std::vector<TurnPenalty> turn_penalties;
    std::vector<EdgeDuration> overlay_shortcut;
    std::array<customizer::TemporalFunctionID, 1> overlay_function_ids{{0}};
    std::array<EdgeDuration, 1> overlay_min_durations{{EdgeDuration{15}}};

    MockFacade() : border_edges{}
    {
        static_durations.resize(4);
        temporal_durations.resize(4);
        for (auto node = 0U; node < 4U; ++node)
        {
            static_durations[node] = {SegmentDuration{5}};
            temporal_durations[node] = {
                EdgeDuration{5}, EdgeDuration{7}, EdgeDuration{5}, EdgeDuration{7}};
        }

        overlay_shortcut = {
            EdgeDuration{15}, EdgeDuration{21}, EdgeDuration{15}, EdgeDuration{21}};

        for (auto &level_edges : border_edges)
        {
            level_edges.resize(static_durations.size());
        }

        // Level 0 base chain: 0 -> 1 -> 2 -> 3 and reverse-only edges for lower-bound search.
        AddEdge(0, 0, 1, true, false);
        AddEdge(0, 1, 0, false, true);
        AddEdge(0, 1, 2, true, false);
        AddEdge(0, 2, 1, false, true);
        AddEdge(0, 2, 3, true, false);
        AddEdge(0, 3, 2, false, true);
    }

    void AddEdge(LevelID level,
                 NodeID from,
                 NodeID to,
                 bool forward,
                 bool backward,
                 EdgeID turn_id = 0,
                 TurnPenalty penalty = TurnPenalty{0})
    {
        const auto edge_id = static_cast<EdgeID>(edges.size());
        edges.push_back(MockEdge{to, forward, backward, {turn_id}});
        border_edges[level][from].push_back(edge_id);

        if (turn_penalties.size() <= turn_id)
        {
            turn_penalties.resize(turn_id + 1, TurnPenalty{0});
        }
        turn_penalties[turn_id] = penalty;
    }

    unsigned GetNumberOfNodes() const { return static_cast<unsigned>(static_durations.size()); }

    const MockPartition &GetMultiLevelPartition() const { return partition; }

    const MockCellStorage &GetCellStorage() const { return cells; }

    std::uint32_t GetTemporalBucketSizeMinutes() const { return 5; }

    std::uint32_t GetTemporalWeekBucketCount() const { return 4; }

    auto GetBorderEdgeRange(LevelID level, NodeID node) const
    {
        const auto &range = border_edges[level][node];
        return std::ranges::subrange(range.begin(), range.end());
    }

    bool IsForwardEdge(EdgeID edge) const { return edges[edge].forward; }

    bool IsBackwardEdge(EdgeID edge) const { return edges[edge].backward; }

    NodeID GetTarget(EdgeID edge) const { return edges[edge].target; }

    const MockEdge::EdgeData &GetEdgeData(EdgeID edge) const
    {
        return edges[edge].data;
    }

    TurnPenalty GetDurationPenaltyForEdgeID(EdgeID turn_id) const
    {
        return turn_id < turn_penalties.size() ? turn_penalties[turn_id] : TurnPenalty{0};
    }

    bool ExcludeNode(NodeID) const { return false; }

    GeometryID GetGeometryIndex(NodeID node) const
    {
        return GeometryID{static_cast<PackedGeometryID>(node), true};
    }

    const auto &GetUncompressedForwardDurations(PackedGeometryID id) const
    {
        return static_durations[id];
    }

    auto GetUncompressedReverseDurations(PackedGeometryID id) const
    {
        return std::views::reverse(static_durations[id]);
    }

    bool HasTemporalForwardProfile(PackedGeometryID id) const
    {
        return id < temporal_durations.size();
    }

    bool HasTemporalReverseProfile(PackedGeometryID id) const
    {
        return HasTemporalForwardProfile(id);
    }

    EdgeDuration GetTemporalForwardDuration(PackedGeometryID id, std::uint32_t week_bucket) const
    {
        return temporal_durations[id][week_bucket];
    }

    EdgeDuration GetTemporalReverseDuration(PackedGeometryID id, std::uint32_t week_bucket) const
    {
        return GetTemporalForwardDuration(id, week_bucket);
    }

    EdgeDuration GetTemporalForwardMinDuration(PackedGeometryID id) const
    {
        return *std::min_element(temporal_durations[id].begin(), temporal_durations[id].end());
    }

    EdgeDuration GetTemporalReverseMinDuration(PackedGeometryID id) const
    {
        return GetTemporalForwardMinDuration(id);
    }

    EdgeDuration GetTemporalShortcutDuration(LevelID level,
                                             CellID cell_id,
                                             NodeID from,
                                             NodeID to,
                                             std::uint32_t week_bucket) const
    {
        if (level != 1 || cell_id != 0 || from != 0 || to != 3)
        {
            return INVALID_EDGE_DURATION;
        }

        return overlay_shortcut[week_bucket];
    }

    EdgeDuration GetTemporalShortcutMinDuration(LevelID level,
                                                CellID cell_id,
                                                NodeID from,
                                                NodeID to) const
    {
        if (level != 1 || cell_id != 0 || from != 0 || to != 3)
        {
            return INVALID_EDGE_DURATION;
        }

        return *std::min_element(overlay_shortcut.begin(), overlay_shortcut.end());
    }

    TemporalShortcutRowView GetTemporalShortcutRow(LevelID level, CellID cell_id, NodeID from) const
    {
        if (level != 1 || cell_id != 0 || from != 0)
        {
            return {};
        }

        return {cells.level_one_cell.destination_nodes.data(),
                overlay_function_ids.data(),
                overlay_min_durations.data(),
                overlay_function_ids.size()};
    }

    TemporalIncomingShortcutRowView
    GetTemporalIncomingShortcutRow(LevelID level, CellID cell_id, NodeID to) const
    {
        if (level != 1 || cell_id != 0 || to != 3)
        {
            return {};
        }

        return {cells.level_one_cell.source_nodes.data(),
                overlay_function_ids.data(),
                overlay_min_durations.data(),
                overlay_function_ids.size(),
                1};
    }

    EdgeDuration GetTemporalFunctionDuration(customizer::TemporalFunctionID function_id,
                                             std::uint32_t week_bucket) const
    {
        if (function_id != 0)
        {
            return INVALID_EDGE_DURATION;
        }

        return overlay_shortcut[week_bucket];
    }

    EdgeID FindEdge(NodeID from, NodeID to) const
    {
        for (const auto edge : border_edges[0][from])
        {
            if (edges[edge].forward && edges[edge].target == to)
            {
                return edge;
            }
        }

        return SPECIAL_EDGEID;
    }
};

struct ArrivalSensitivePartition
{
    std::uint8_t GetNumberOfLevels() const { return 2; }

    CellID GetCell(LevelID level, NodeID node) const
    {
        if (level == 1)
        {
            return node == 0 ? CellID{1} : CellID{0};
        }

        return CellID{0};
    }

    LevelID GetQueryLevel(NodeID source, NodeID target, NodeID node) const
    {
        const auto is_overlay_pair =
            (source == 1 && target == 4) || (source == 4 && target == 1) ||
            (source == 0 && target == 4) || (source == 4 && target == 0);

        if (is_overlay_pair && (node == 1 || node == 4))
        {
            return LevelID{1};
        }

        return LevelID{0};
    }
};

struct ArrivalSensitiveCellStorage
{
    MockCell level_one_cell{{1}, {4}};
    MockCell empty_cell{{}, {}};

    const MockCell &GetUnfilledCell(LevelID level, CellID cell_id) const
    {
        return level == 1 && cell_id == 0 ? level_one_cell : empty_cell;
    }
};

struct ArrivalSensitiveFacade : IncomingBorderEdgeIndexMixin<ArrivalSensitiveFacade>
{
    ArrivalSensitivePartition partition;
    ArrivalSensitiveCellStorage cells;
    std::vector<MockEdge> edges;
    std::array<std::vector<std::vector<EdgeID>>, 2> border_edges;
    std::vector<std::vector<SegmentDuration>> static_durations;
    std::vector<std::vector<EdgeDuration>> temporal_durations;
    std::vector<EdgeDuration> overlay_shortcut;
    std::array<customizer::TemporalFunctionID, 1> overlay_function_ids{{0}};
    std::array<EdgeDuration, 1> overlay_min_durations{{EdgeDuration{200}}};

    ArrivalSensitiveFacade() : border_edges{}
    {
        static_durations.resize(5);
        temporal_durations.resize(5);

        static_durations[0] = {SegmentDuration{600}};
        static_durations[1] = {SegmentDuration{100}};
        static_durations[2] = {SegmentDuration{100}};
        static_durations[3] = {SegmentDuration{100}};
        static_durations[4] = {SegmentDuration{100}};

        temporal_durations[0] = {
            EdgeDuration{600}, EdgeDuration{600}, EdgeDuration{600}, EdgeDuration{600}};
        temporal_durations[1] = {
            EdgeDuration{100}, EdgeDuration{100}, EdgeDuration{100}, EdgeDuration{100}};
        temporal_durations[2] = {
            EdgeDuration{100}, EdgeDuration{500}, EdgeDuration{100}, EdgeDuration{500}};
        temporal_durations[3] = {
            EdgeDuration{500}, EdgeDuration{100}, EdgeDuration{500}, EdgeDuration{100}};
        temporal_durations[4] = {
            EdgeDuration{100}, EdgeDuration{100}, EdgeDuration{100}, EdgeDuration{100}};

        overlay_shortcut = {
            EdgeDuration{200}, EdgeDuration{200}, EdgeDuration{200}, EdgeDuration{200}};

        for (auto &level_edges : border_edges)
        {
            level_edges.resize(static_durations.size());
        }

        AddEdge(0, 0, 1, true, false);
        AddEdge(0, 1, 0, false, true);
        AddEdge(0, 1, 2, true, false);
        AddEdge(0, 2, 1, false, true);
        AddEdge(0, 2, 4, true, false);
        AddEdge(0, 4, 2, false, true);
        AddEdge(0, 1, 3, true, false);
        AddEdge(0, 3, 1, false, true);
        AddEdge(0, 3, 4, true, false);
        AddEdge(0, 4, 3, false, true);

        AddEdge(1, 1, 0, false, true);
    }

    void AddEdge(LevelID level, NodeID from, NodeID to, bool forward, bool backward)
    {
        const auto edge_id = static_cast<EdgeID>(edges.size());
        edges.push_back(MockEdge{to, forward, backward, {}});
        border_edges[level][from].push_back(edge_id);
    }

    unsigned GetNumberOfNodes() const { return static_cast<unsigned>(static_durations.size()); }

    const ArrivalSensitivePartition &GetMultiLevelPartition() const { return partition; }

    const ArrivalSensitiveCellStorage &GetCellStorage() const { return cells; }

    std::uint32_t GetTemporalBucketSizeMinutes() const { return 1; }

    std::uint32_t GetTemporalWeekBucketCount() const { return 4; }

    auto GetBorderEdgeRange(LevelID level, NodeID node) const
    {
        const auto &range = border_edges[level][node];
        return std::ranges::subrange(range.begin(), range.end());
    }

    bool IsForwardEdge(EdgeID edge) const { return edges[edge].forward; }

    bool IsBackwardEdge(EdgeID edge) const { return edges[edge].backward; }

    NodeID GetTarget(EdgeID edge) const { return edges[edge].target; }

    const MockEdge::EdgeData &GetEdgeData(EdgeID edge) const
    {
        return edges[edge].data;
    }

    TurnPenalty GetDurationPenaltyForEdgeID(EdgeID) const { return TurnPenalty{0}; }

    bool ExcludeNode(NodeID) const { return false; }

    GeometryID GetGeometryIndex(NodeID node) const
    {
        return GeometryID{static_cast<PackedGeometryID>(node), true};
    }

    const auto &GetUncompressedForwardDurations(PackedGeometryID id) const
    {
        return static_durations[id];
    }

    auto GetUncompressedReverseDurations(PackedGeometryID id) const
    {
        return std::views::reverse(static_durations[id]);
    }

    bool HasTemporalForwardProfile(PackedGeometryID id) const
    {
        return id < temporal_durations.size();
    }

    bool HasTemporalReverseProfile(PackedGeometryID id) const
    {
        return HasTemporalForwardProfile(id);
    }

    EdgeDuration GetTemporalForwardDuration(PackedGeometryID id, std::uint32_t week_bucket) const
    {
        return temporal_durations[id][week_bucket];
    }

    EdgeDuration GetTemporalReverseDuration(PackedGeometryID id, std::uint32_t week_bucket) const
    {
        return GetTemporalForwardDuration(id, week_bucket);
    }

    EdgeDuration GetTemporalForwardMinDuration(PackedGeometryID id) const
    {
        return *std::min_element(temporal_durations[id].begin(), temporal_durations[id].end());
    }

    EdgeDuration GetTemporalReverseMinDuration(PackedGeometryID id) const
    {
        return GetTemporalForwardMinDuration(id);
    }

    EdgeDuration GetTemporalShortcutDuration(LevelID level,
                                             CellID cell_id,
                                             NodeID from,
                                             NodeID to,
                                             std::uint32_t week_bucket) const
    {
        if (level != 1 || cell_id != 0 || from != 1 || to != 4)
        {
            return INVALID_EDGE_DURATION;
        }

        return overlay_shortcut[week_bucket];
    }

    EdgeDuration GetTemporalShortcutMinDuration(LevelID level,
                                                CellID cell_id,
                                                NodeID from,
                                                NodeID to) const
    {
        if (level != 1 || cell_id != 0 || from != 1 || to != 4)
        {
            return INVALID_EDGE_DURATION;
        }

        return *std::min_element(overlay_shortcut.begin(), overlay_shortcut.end());
    }

    TemporalShortcutRowView GetTemporalShortcutRow(LevelID level, CellID cell_id, NodeID from) const
    {
        if (level != 1 || cell_id != 0 || from != 1)
        {
            return {};
        }

        return {cells.level_one_cell.destination_nodes.data(),
                overlay_function_ids.data(),
                overlay_min_durations.data(),
                overlay_function_ids.size()};
    }

    TemporalIncomingShortcutRowView
    GetTemporalIncomingShortcutRow(LevelID level, CellID cell_id, NodeID to) const
    {
        if (level != 1 || cell_id != 0 || to != 4)
        {
            return {};
        }

        return {cells.level_one_cell.source_nodes.data(),
                overlay_function_ids.data(),
                overlay_min_durations.data(),
                overlay_function_ids.size(),
                1};
    }

    EdgeDuration GetTemporalFunctionDuration(customizer::TemporalFunctionID function_id,
                                             std::uint32_t week_bucket) const
    {
        if (function_id != 0)
        {
            return INVALID_EDGE_DURATION;
        }

        return overlay_shortcut[week_bucket];
    }

    EdgeID FindEdge(NodeID from, NodeID to) const
    {
        for (const auto edge : border_edges[0][from])
        {
            if (edges[edge].forward && edges[edge].target == to)
            {
                return edge;
            }
        }

        return SPECIAL_EDGEID;
    }
};

engine::PhantomNode MakeZeroTraversalPhantom(NodeID node)
{
    engine::PhantomNode phantom;
    phantom.forward_segment_id = SegmentID{node, true};
    phantom.forward_duration = EdgeDuration{0};
    phantom.forward_weight = EdgeWeight{0};
    phantom.component = ComponentID{1, false};
    return phantom;
}

std::time_t mondayMidnightUtc()
{
    return 4 * 24 * 60 * 60;
}

struct ExactPathResult
{
    EdgeDuration total_duration = INVALID_EDGE_DURATION;
    std::vector<NodeID> nodes;

    bool is_valid() const { return total_duration != INVALID_EDGE_DURATION; }
};

template <typename FacadeT, typename NodeDurationFn>
ExactPathResult ExactBaseDijkstra(const FacadeT &facade,
                                  const NodeID source,
                                  const NodeID target,
                                  const NodeDurationFn &get_node_duration)
{
    struct QueueEntry
    {
        EdgeDuration duration = INVALID_EDGE_DURATION;
        NodeID node = SPECIAL_NODEID;
    };

    struct QueueCompare
    {
        bool operator()(const QueueEntry &lhs, const QueueEntry &rhs) const
        {
            return lhs.duration > rhs.duration;
        }
    };

    std::vector<EdgeDuration> best_durations(facade.GetNumberOfNodes(), INVALID_EDGE_DURATION);
    std::vector<NodeID> parents(facade.GetNumberOfNodes(), SPECIAL_NODEID);
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> queue;

    best_durations[source] = EdgeDuration{0};
    parents[source] = source;
    queue.push({EdgeDuration{0}, source});

    while (!queue.empty())
    {
        const auto current = queue.top();
        queue.pop();

        if (current.duration != best_durations[current.node])
        {
            continue;
        }

        if (current.node == target)
        {
            break;
        }

        const auto edge_duration = get_node_duration(current.node, current.duration);
        if (edge_duration == INVALID_EDGE_DURATION)
        {
            continue;
        }

        for (const auto edge : facade.GetBorderEdgeRange(LevelID{0}, current.node))
        {
            if (!facade.IsForwardEdge(edge))
            {
                continue;
            }

            const auto next = facade.GetTarget(edge);
            const auto next_duration =
                engine::temporal::detail::SafeDurationAdd(current.duration, edge_duration);
            if (next_duration == INVALID_EDGE_DURATION ||
                (best_durations[next] != INVALID_EDGE_DURATION &&
                 next_duration >= best_durations[next]))
            {
                continue;
            }

            best_durations[next] = next_duration;
            parents[next] = current.node;
            queue.push({next_duration, next});
        }
    }

    if (best_durations[target] == INVALID_EDGE_DURATION)
    {
        return {};
    }

    std::vector<NodeID> nodes;
    for (auto node = target; node != source; node = parents[node])
    {
        BOOST_REQUIRE_NE(parents[node], SPECIAL_NODEID);
        nodes.push_back(node);
    }
    nodes.push_back(source);
    std::reverse(nodes.begin(), nodes.end());

    return {best_durations[target], std::move(nodes)};
}

template <typename FacadeT>
ExactPathResult ExactStaticBaseDijkstra(const FacadeT &facade, const NodeID source, const NodeID target)
{
    return ExactBaseDijkstra(
        facade,
        source,
        target,
        [&facade](const NodeID node, const EdgeDuration)
        {
            const auto geometry = facade.GetGeometryIndex(node);
            return engine::temporal::GetStaticGeometryDuration(
                facade, geometry.id, geometry.forward);
        });
}

template <typename FacadeT>
ExactPathResult ExactTemporalBaseDijkstra(const FacadeT &facade,
                                          const NodeID source,
                                          const NodeID target,
                                          const std::time_t departure_timestamp)
{
    return ExactBaseDijkstra(
        facade,
        source,
        target,
        [&facade, departure_timestamp](const NodeID node, const EdgeDuration elapsed_duration)
        {
            const auto geometry = facade.GetGeometryIndex(node);
            const auto current_clock = engine::temporal::AdvanceTemporalClock(
                engine::temporal::ToTemporalClock(departure_timestamp), elapsed_duration);
            return engine::temporal::GetGeometryDurationAtClock(
                       facade, geometry.id, geometry.forward, current_clock)
                .duration;
        });
}

struct DivergencePartition
{
    std::uint8_t GetNumberOfLevels() const { return 2; }

    CellID GetCell(LevelID, NodeID) const { return 0; }

    LevelID GetQueryLevel(NodeID source, NodeID target, NodeID node) const
    {
        if ((source == 1 && target == 4) || (source == 4 && target == 1))
        {
            if (node == 1 || node == 4)
            {
                return LevelID{1};
            }
        }

        return LevelID{0};
    }
};

struct DivergenceCellStorage
{
    MockCell level_one_cell{{1}, {4}};
    MockCell empty_cell{{}, {}};

    const MockCell &GetUnfilledCell(LevelID level, CellID) const
    {
        return level == 1 ? level_one_cell : empty_cell;
    }
};

struct StaticTemporalDivergenceFacade : IncomingBorderEdgeIndexMixin<StaticTemporalDivergenceFacade>
{
    DivergencePartition partition;
    DivergenceCellStorage cells;
    std::vector<MockEdge> edges;
    std::array<std::vector<std::vector<EdgeID>>, 2> border_edges;
    std::vector<std::vector<SegmentDuration>> static_durations;
    std::vector<std::vector<EdgeDuration>> temporal_durations;
    std::vector<EdgeDuration> overlay_shortcut;
    std::array<customizer::TemporalFunctionID, 1> overlay_function_ids{{0}};
    std::array<EdgeDuration, 1> overlay_min_durations{{EdgeDuration{20}}};

    StaticTemporalDivergenceFacade() : border_edges{}
    {
        static_durations.resize(5);
        temporal_durations.resize(5);

        for (auto &level_edges : border_edges)
        {
            level_edges.resize(static_durations.size());
        }

        static_durations[1] = {SegmentDuration{10}};
        static_durations[2] = {SegmentDuration{10}};
        static_durations[3] = {SegmentDuration{30}};
        static_durations[4] = {SegmentDuration{10}};

        temporal_durations[1] = {
            EdgeDuration{10}, EdgeDuration{10}, EdgeDuration{10}, EdgeDuration{10}};
        temporal_durations[2] = {
            EdgeDuration{100}, EdgeDuration{10}, EdgeDuration{100}, EdgeDuration{10}};
        temporal_durations[3] = {
            EdgeDuration{20}, EdgeDuration{200}, EdgeDuration{20}, EdgeDuration{200}};
        temporal_durations[4] = {
            EdgeDuration{10}, EdgeDuration{10}, EdgeDuration{10}, EdgeDuration{10}};

        overlay_shortcut = {
            EdgeDuration{30}, EdgeDuration{20}, EdgeDuration{30}, EdgeDuration{20}};

        AddEdge(LevelID{0}, 1, 2, true, false);
        AddEdge(LevelID{0}, 2, 1, false, true);
        AddEdge(LevelID{0}, 2, 4, true, false);
        AddEdge(LevelID{0}, 4, 2, false, true);
        AddEdge(LevelID{0}, 1, 3, true, false);
        AddEdge(LevelID{0}, 3, 1, false, true);
        AddEdge(LevelID{0}, 3, 4, true, false);
        AddEdge(LevelID{0}, 4, 3, false, true);
    }

    void AddEdge(LevelID level, NodeID from, NodeID to, bool forward, bool backward)
    {
        const auto edge_id = static_cast<EdgeID>(edges.size());
        edges.push_back(MockEdge{to, forward, backward, {}});
        border_edges[level][from].push_back(edge_id);
    }

    unsigned GetNumberOfNodes() const { return static_cast<unsigned>(static_durations.size()); }

    const DivergencePartition &GetMultiLevelPartition() const { return partition; }

    const DivergenceCellStorage &GetCellStorage() const { return cells; }

    std::uint32_t GetTemporalBucketSizeMinutes() const { return 1; }

    std::uint32_t GetTemporalWeekBucketCount() const { return 4; }

    auto GetBorderEdgeRange(LevelID level, NodeID node) const
    {
        const auto &range = border_edges[level][node];
        return std::ranges::subrange(range.begin(), range.end());
    }

    bool IsForwardEdge(EdgeID edge) const { return edges[edge].forward; }

    bool IsBackwardEdge(EdgeID edge) const { return edges[edge].backward; }

    NodeID GetTarget(EdgeID edge) const { return edges[edge].target; }

    const MockEdge::EdgeData &GetEdgeData(EdgeID edge) const { return edges[edge].data; }

    TurnPenalty GetDurationPenaltyForEdgeID(EdgeID) const { return TurnPenalty{0}; }

    bool ExcludeNode(NodeID) const { return false; }

    GeometryID GetGeometryIndex(NodeID node) const
    {
        return GeometryID{static_cast<PackedGeometryID>(node), true};
    }

    const auto &GetUncompressedForwardDurations(PackedGeometryID id) const
    {
        return static_durations[id];
    }

    auto GetUncompressedReverseDurations(PackedGeometryID id) const
    {
        return std::views::reverse(static_durations[id]);
    }

    bool HasTemporalForwardProfile(PackedGeometryID id) const
    {
        return id < temporal_durations.size();
    }

    bool HasTemporalReverseProfile(PackedGeometryID id) const
    {
        return HasTemporalForwardProfile(id);
    }

    EdgeDuration GetTemporalForwardDuration(PackedGeometryID id, std::uint32_t week_bucket) const
    {
        return temporal_durations[id][week_bucket];
    }

    EdgeDuration GetTemporalReverseDuration(PackedGeometryID id, std::uint32_t week_bucket) const
    {
        return GetTemporalForwardDuration(id, week_bucket);
    }

    EdgeDuration GetTemporalForwardMinDuration(PackedGeometryID id) const
    {
        return *std::min_element(temporal_durations[id].begin(), temporal_durations[id].end());
    }

    EdgeDuration GetTemporalReverseMinDuration(PackedGeometryID id) const
    {
        return GetTemporalForwardMinDuration(id);
    }

    EdgeDuration GetTemporalShortcutDuration(LevelID level,
                                             CellID cell_id,
                                             NodeID from,
                                             NodeID to,
                                             std::uint32_t week_bucket) const
    {
        if (level != 1 || cell_id != 0 || from != 1 || to != 4)
        {
            return INVALID_EDGE_DURATION;
        }

        return overlay_shortcut[week_bucket];
    }

    EdgeDuration GetTemporalShortcutMinDuration(LevelID level,
                                                CellID cell_id,
                                                NodeID from,
                                                NodeID to) const
    {
        if (level != 1 || cell_id != 0 || from != 1 || to != 4)
        {
            return INVALID_EDGE_DURATION;
        }

        return *std::min_element(overlay_shortcut.begin(), overlay_shortcut.end());
    }

    TemporalShortcutRowView GetTemporalShortcutRow(LevelID level, CellID cell_id, NodeID from) const
    {
        if (level != 1 || cell_id != 0 || from != 1)
        {
            return {};
        }

        return {cells.level_one_cell.destination_nodes.data(),
                overlay_function_ids.data(),
                overlay_min_durations.data(),
                overlay_function_ids.size()};
    }

    TemporalIncomingShortcutRowView
    GetTemporalIncomingShortcutRow(LevelID level, CellID cell_id, NodeID to) const
    {
        if (level != 1 || cell_id != 0 || to != 4)
        {
            return {};
        }

        return {cells.level_one_cell.source_nodes.data(),
                overlay_function_ids.data(),
                overlay_min_durations.data(),
                overlay_function_ids.size(),
                1};
    }

    EdgeDuration GetTemporalFunctionDuration(customizer::TemporalFunctionID function_id,
                                             std::uint32_t week_bucket) const
    {
        if (function_id != 0)
        {
            return INVALID_EDGE_DURATION;
        }

        return overlay_shortcut[week_bucket];
    }

    EdgeID FindEdge(NodeID from, NodeID to) const
    {
        for (const auto edge : border_edges[0][from])
        {
            if (edges[edge].forward && edges[edge].target == to)
            {
                return edge;
            }
        }

        return SPECIAL_EDGEID;
    }
};

struct MissingShortcutFallbackFacade : StaticTemporalDivergenceFacade
{
    TemporalShortcutRowView GetTemporalShortcutRow(LevelID, CellID, NodeID) const { return {}; }

    TemporalIncomingShortcutRowView GetTemporalIncomingShortcutRow(LevelID, CellID, NodeID) const
    {
        return {};
    }

    EdgeDuration GetTemporalShortcutDuration(LevelID,
                                             CellID,
                                             NodeID,
                                             NodeID,
                                             std::uint32_t) const
    {
        return INVALID_EDGE_DURATION;
    }

    EdgeDuration GetTemporalShortcutMinDuration(LevelID, CellID, NodeID, NodeID) const
    {
        return INVALID_EDGE_DURATION;
    }
};

struct IncomingShortcutRowOnlyFacade : MockFacade
{
    IncomingShortcutRowOnlyFacade()
    {
        for (auto node = 0U; node < static_durations.size(); ++node)
        {
            static_durations[node] = {SegmentDuration{10}};
            temporal_durations[node] = {
                EdgeDuration{10}, EdgeDuration{10}, EdgeDuration{10}, EdgeDuration{10}};
        }
    }

    EdgeDuration GetTemporalShortcutMinDuration(LevelID, CellID, NodeID, NodeID) const
    {
        return INVALID_EDGE_DURATION;
    }
};

struct DuplicateRangeFacade : MockFacade
{
    DuplicateRangeFacade()
    {
        edges.clear();
        turn_penalties.clear();
        for (auto &level_edges : border_edges)
        {
            for (auto &node_edges : level_edges)
            {
                node_edges.clear();
            }
        }

        AddEdge(LevelID{1}, 3, 2, false, true);
        border_edges[0][3].push_back(EdgeID{0});
        AddEdge(LevelID{0}, 3, 1, false, true);
        AddEdge(LevelID{1}, 3, 0, true, false);
        border_edges[0][3].push_back(EdgeID{2});
    }
};

struct ParallelEdgeFacade : MockFacade
{
    ParallelEdgeFacade()
    {
        edges.clear();
        turn_penalties.clear();
        for (auto &level_edges : border_edges)
        {
            for (auto &node_edges : level_edges)
            {
                node_edges.clear();
            }
        }

        AddEdge(LevelID{0}, 3, 2, false, true, EdgeID{0}, TurnPenalty{5});
        AddEdge(LevelID{0}, 3, 2, false, true, EdgeID{1}, TurnPenalty{0});
    }
};

template <typename FacadeT>
void RelaxReverseBorderPredecessorsReference(
    const FacadeT &facade,
    const NodeID current_node,
    const EdgeDuration current_cost,
    const LevelID level,
    const engine::routing_algorithms::mld::temporal::overlay::detail::SearchRestriction &restriction,
    engine::routing_algorithms::mld::temporal::overlay::detail::ReverseLowerBoundWorkspace &workspace,
    engine::routing_algorithms::mld::temporal::overlay::detail::ReverseLowerBoundSearchStats &stats)
{
    namespace overlay_detail = engine::routing_algorithms::mld::temporal::overlay::detail;

    const auto &partition = facade.GetMultiLevelPartition();

    overlay_detail::ForEachDescendingLevel(
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
                    !overlay_detail::CheckParentCellRestriction(
                        partition, edge_level, predecessor, restriction))
                {
                    continue;
                }

                const auto node_duration =
                    engine::routing_algorithms::mld::temporal::detail::GetNodeLowerBoundDuration(
                        facade, predecessor);
                const auto turn_penalty =
                    engine::routing_algorithms::mld::temporal::detail::TurnPenaltyToDuration(
                    facade.GetDurationPenaltyForEdgeID(facade.GetEdgeData(edge).turn_id));
                const auto edge_cost =
                    engine::temporal::detail::SafeDurationAdd(node_duration, turn_penalty);
                const auto candidate =
                    engine::temporal::detail::SafeDurationAdd(current_cost, edge_cost);

                if (candidate == INVALID_EDGE_DURATION)
                {
                    continue;
                }

                if (workspace.lower_bounds[predecessor] == INVALID_EDGE_DURATION ||
                    candidate < workspace.lower_bounds[predecessor])
                {
                    overlay_detail::UpdateReverseLowerBound(
                        workspace, stats, predecessor, candidate, false);
                }
            }
        });
}

template <typename FacadeT, typename QueryLevelPolicyT>
std::vector<EdgeDuration> ComputeReverseLowerBoundsWithReferenceBorderScan(
    const FacadeT &facade,
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint &target,
    const engine::routing_algorithms::mld::temporal::overlay::detail::SearchRestriction &restriction,
    const QueryLevelPolicyT &query_level_policy)
{
    namespace overlay_detail = engine::routing_algorithms::mld::temporal::overlay::detail;

    overlay_detail::ReverseLowerBoundWorkspace workspace{facade.GetNumberOfNodes()};
    overlay_detail::ReverseLowerBoundSearchStats stats;
    overlay_detail::InitializeReverseLowerBounds(facade, target, workspace, stats);

    while (!workspace.queue.empty())
    {
        const auto current = workspace.queue.top();
        workspace.queue.pop();

        if (current.node >= workspace.lower_bounds.size() ||
            current.cost != workspace.lower_bounds[current.node])
        {
            continue;
        }

        const auto level = query_level_policy.GetNodeQueryLevel(current.node, restriction);
        overlay_detail::RelaxReverseShortcutPredecessors(
            facade,
            current.node,
            current.cost,
            level,
            restriction,
            query_level_policy,
            workspace,
            stats);
        RelaxReverseBorderPredecessorsReference(
            facade, current.node, current.cost, level, restriction, workspace, stats);
    }

    return workspace.lower_bounds;
}
} // namespace

BOOST_AUTO_TEST_SUITE(temporal_overlay_query)

BOOST_AUTO_TEST_CASE(shared_endpoint_query_level_policy_uses_minimum_level_across_endpoint_pairs)
{
    MockFacade facade;

    const auto source_zero_phantom = MakeZeroTraversalPhantom(0);
    const auto source_one_phantom = MakeZeroTraversalPhantom(1);
    const auto target_two_phantom = MakeZeroTraversalPhantom(2);
    const auto target_three_phantom = MakeZeroTraversalPhantom(3);

    const std::vector<engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint>
        source_endpoints{{&source_zero_phantom, 0, false}, {&source_one_phantom, 1, false}};
    const std::vector<engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint>
        target_endpoints{{&target_two_phantom, 2, false}, {&target_three_phantom, 3, false}};

    const auto pair_level =
        engine::routing_algorithms::mld::temporal::overlay::detail::GetNodeQueryLevel(
            facade.GetMultiLevelPartition(),
            NodeID{0},
            source_endpoints.front(),
            target_endpoints.back(),
            {});
    const auto shared_query_level_policy =
        engine::routing_algorithms::mld::temporal::overlay::detail::MakeSharedEndpointQueryLevelPolicy(
            facade, source_endpoints, target_endpoints);

    BOOST_CHECK_EQUAL(pair_level, LevelID{1});
    BOOST_CHECK_EQUAL(
        shared_query_level_policy.GetNodeQueryLevel(NodeID{0}, {}), LevelID{0});
    BOOST_CHECK_EQUAL(
        shared_query_level_policy.GetNodeQueryLevel(NodeID{3}, {}), LevelID{0});
}

BOOST_AUTO_TEST_CASE(shared_corridor_search_preserves_unpacked_result_when_broadened)
{
    MockFacade facade;

    const auto source_zero_phantom = MakeZeroTraversalPhantom(0);
    const auto source_one_phantom = MakeZeroTraversalPhantom(1);
    const auto target_two_phantom = MakeZeroTraversalPhantom(2);
    const auto target_three_phantom = MakeZeroTraversalPhantom(3);

    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_zero_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_three_phantom, 3, false};
    const std::vector<engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint>
        source_endpoints{source, {&source_one_phantom, 1, false}};
    const std::vector<engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint>
        target_endpoints{{&target_two_phantom, 2, false}, target};
    const auto shared_query_level_policy =
        engine::routing_algorithms::mld::temporal::overlay::detail::MakeSharedEndpointQueryLevelPolicy(
            facade, source_endpoints, target_endpoints);

    const auto pair_specific_path = engine::routing_algorithms::mld::temporal::overlay::Search(
        facade, source, target, mondayMidnightUtc());
    const auto shared_corridor_path = engine::routing_algorithms::mld::temporal::overlay::Search(
        facade, source, target, mondayMidnightUtc(), shared_query_level_policy);
    const auto pair_specific_unpacked = engine::routing_algorithms::mld::temporal::overlay::UnpackPath(
        facade, source, target, mondayMidnightUtc(), pair_specific_path);
    const auto shared_corridor_unpacked =
        engine::routing_algorithms::mld::temporal::overlay::UnpackPath(
            facade, source, target, mondayMidnightUtc(), shared_corridor_path);

    BOOST_REQUIRE(pair_specific_path.is_valid());
    BOOST_REQUIRE(shared_corridor_path.is_valid());
    BOOST_REQUIRE(pair_specific_unpacked.is_valid());
    BOOST_REQUIRE(shared_corridor_unpacked.is_valid());

    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(pair_specific_unpacked.total_duration), 15);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(shared_corridor_unpacked.total_duration), 15);
    BOOST_CHECK_EQUAL_COLLECTIONS(shared_corridor_unpacked.nodes.begin(),
                                  shared_corridor_unpacked.nodes.end(),
                                  pair_specific_unpacked.nodes.begin(),
                                  pair_specific_unpacked.nodes.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(shared_corridor_unpacked.edges.begin(),
                                  shared_corridor_unpacked.edges.end(),
                                  pair_specific_unpacked.edges.begin(),
                                  pair_specific_unpacked.edges.end());
}

BOOST_AUTO_TEST_CASE(restricted_level_one_search_uses_temporal_shortcut)
{
    MockFacade facade;

    const auto source_phantom = MakeZeroTraversalPhantom(0);
    const auto target_phantom = MakeZeroTraversalPhantom(3);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 3, false};

    const auto reverse_lower_bounds =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBounds(
            facade,
            source,
            target,
            {true, LevelID{1}, CellID{0}});
    BOOST_REQUIRE_EQUAL(reverse_lower_bounds.size(), 4U);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(reverse_lower_bounds[3]), 0);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(reverse_lower_bounds[0]), 15);

    const auto path = engine::routing_algorithms::mld::temporal::overlay::Search(
        facade, source, target, mondayMidnightUtc(), LevelID{1}, CellID{0});

    BOOST_REQUIRE(path.is_valid());
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(path.total_duration), 15);
    BOOST_REQUIRE_EQUAL(path.packed_path.size(), 1U);
    BOOST_CHECK_EQUAL(path.packed_path.front().from, 0);
    BOOST_CHECK_EQUAL(path.packed_path.front().to, 3);
    BOOST_CHECK(path.packed_path.front().is_overlay);
    BOOST_CHECK_EQUAL(path.packed_path.front().overlay_level, LevelID{1});
}

BOOST_AUTO_TEST_CASE(reverse_lower_bound_workspace_reuse_matches_public_results)
{
    MockFacade facade;

    const auto source_phantom = MakeZeroTraversalPhantom(0);
    const auto target_phantom = MakeZeroTraversalPhantom(3);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 3, false};

    const auto unrestricted_reference =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBounds(
            facade, source, target, {});
    const auto restricted_reference =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBounds(
            facade,
            source,
            target,
            {true, LevelID{1}, CellID{0}});

    engine::routing_algorithms::mld::temporal::overlay::detail::ReverseLowerBoundWorkspace
        workspace{facade.GetNumberOfNodes()};
    const engine::routing_algorithms::mld::temporal::overlay::detail::PairQueryLevelPolicy
        query_level_policy{facade.GetMultiLevelPartition(), source, target};

    const auto &unrestricted_workspace_result =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBounds(
            facade, target, {}, query_level_policy, workspace);
    std::vector<EdgeDuration> unrestricted_copy(unrestricted_workspace_result.begin(),
                                                unrestricted_workspace_result.end());

    const auto &restricted_workspace_result =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBounds(
            facade,
            target,
            {true, LevelID{1}, CellID{0}},
            query_level_policy,
            workspace);
    std::vector<EdgeDuration> restricted_copy(restricted_workspace_result.begin(),
                                              restricted_workspace_result.end());

    BOOST_CHECK_EQUAL_COLLECTIONS(unrestricted_copy.begin(),
                                  unrestricted_copy.end(),
                                  unrestricted_reference.begin(),
                                  unrestricted_reference.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(restricted_copy.begin(),
                                  restricted_copy.end(),
                                  restricted_reference.begin(),
                                  restricted_reference.end());
}

BOOST_AUTO_TEST_CASE(reverse_lower_bounds_match_reference_border_scan_for_arrival_sensitive_facade)
{
    ArrivalSensitiveFacade facade;

    const auto source_phantom = MakeZeroTraversalPhantom(1);
    const auto target_phantom = MakeZeroTraversalPhantom(4);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 1, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 4, false};
    const engine::routing_algorithms::mld::temporal::overlay::detail::PairQueryLevelPolicy
        query_level_policy{facade.GetMultiLevelPartition(), source, target};
    const auto restriction =
        engine::routing_algorithms::mld::temporal::overlay::detail::SearchRestriction{
            true, LevelID{1}, CellID{0}};

    const auto unrestricted =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBounds(
            facade, target, {}, query_level_policy);
    const auto unrestricted_reference = ComputeReverseLowerBoundsWithReferenceBorderScan(
        facade, target, {}, query_level_policy);
    const auto restricted =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBounds(
            facade, target, restriction, query_level_policy);
    const auto restricted_reference = ComputeReverseLowerBoundsWithReferenceBorderScan(
        facade, target, restriction, query_level_policy);

    BOOST_CHECK_EQUAL_COLLECTIONS(
        unrestricted.begin(), unrestricted.end(), unrestricted_reference.begin(), unrestricted_reference.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(
        restricted.begin(), restricted.end(), restricted_reference.begin(), restricted_reference.end());
}

BOOST_AUTO_TEST_CASE(incoming_border_edge_row_deduplicates_repeated_backward_edges)
{
    DuplicateRangeFacade facade;

    const auto row = facade.GetIncomingBorderEdgeRow(3);
    BOOST_REQUIRE_EQUAL(row.size, 2U);

    std::vector<std::pair<NodeID, LevelID>> predecessor_levels;
    predecessor_levels.reserve(row.size);
    for (auto index = std::size_t{0}; index < row.size; ++index)
    {
        predecessor_levels.push_back({facade.GetTarget(row.edges[index]), row.highest_border_levels[index]});
    }
    std::sort(predecessor_levels.begin(), predecessor_levels.end());

    BOOST_CHECK_EQUAL(predecessor_levels[0].first, 1U);
    BOOST_CHECK_EQUAL(predecessor_levels[0].second, LevelID{0});
    BOOST_CHECK_EQUAL(predecessor_levels[1].first, 2U);
    BOOST_CHECK_EQUAL(predecessor_levels[1].second, LevelID{1});

    const auto source_phantom = MakeZeroTraversalPhantom(0);
    const auto target_phantom = MakeZeroTraversalPhantom(3);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 3, false};
    const engine::routing_algorithms::mld::temporal::overlay::detail::PairQueryLevelPolicy
        query_level_policy{facade.GetMultiLevelPartition(), source, target};

    const auto indexed =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBounds(
            facade, target, {}, query_level_policy);
    const auto reference =
        ComputeReverseLowerBoundsWithReferenceBorderScan(facade, target, {}, query_level_policy);

    BOOST_CHECK_EQUAL_COLLECTIONS(indexed.begin(), indexed.end(), reference.begin(), reference.end());
}

BOOST_AUTO_TEST_CASE(incoming_border_edge_row_preserves_parallel_backward_edges)
{
    ParallelEdgeFacade facade;

    const auto row = facade.GetIncomingBorderEdgeRow(3);
    BOOST_REQUIRE_EQUAL(row.size, 2U);
    BOOST_CHECK_NE(row.edges[0], row.edges[1]);
    BOOST_CHECK_EQUAL(facade.GetTarget(row.edges[0]), 2U);
    BOOST_CHECK_EQUAL(facade.GetTarget(row.edges[1]), 2U);

    const auto source_phantom = MakeZeroTraversalPhantom(0);
    const auto target_phantom = MakeZeroTraversalPhantom(3);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 3, false};
    const engine::routing_algorithms::mld::temporal::overlay::detail::PairQueryLevelPolicy
        query_level_policy{facade.GetMultiLevelPartition(), source, target};

    const auto indexed =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBounds(
            facade, target, {}, query_level_policy);
    const auto reference =
        ComputeReverseLowerBoundsWithReferenceBorderScan(facade, target, {}, query_level_policy);

    BOOST_CHECK_EQUAL_COLLECTIONS(indexed.begin(), indexed.end(), reference.begin(), reference.end());
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(indexed[2]), 5);
}

BOOST_AUTO_TEST_CASE(reverse_lower_bound_target_set_initialization_keeps_minimum_seed_per_node)
{
    MockFacade facade;

    auto target_high_phantom = MakeZeroTraversalPhantom(3);
    target_high_phantom.forward_duration = EdgeDuration{5};
    const auto target_low_phantom = MakeZeroTraversalPhantom(3);
    auto target_other_phantom = MakeZeroTraversalPhantom(2);
    target_other_phantom.forward_duration = EdgeDuration{5};

    const std::vector<engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint> targets{
        {&target_high_phantom, 3, false},
        {&target_low_phantom, 3, false},
        {&target_other_phantom, 2, false}};

    engine::routing_algorithms::mld::temporal::overlay::detail::ReverseLowerBoundWorkspace
        workspace{facade.GetNumberOfNodes()};
    engine::routing_algorithms::mld::temporal::overlay::detail::ReverseLowerBoundSearchStats stats;

    engine::routing_algorithms::mld::temporal::overlay::detail::InitializeReverseLowerBoundsForTargets(
        facade, targets, workspace, stats);

    BOOST_CHECK_EQUAL(stats.lower_bound_updates, 3U);
    BOOST_CHECK_EQUAL(workspace.touched_nodes.size(), 2U);
    BOOST_CHECK_EQUAL(workspace.queue.size(), 3U);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(workspace.lower_bounds[3]), 0);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(workspace.lower_bounds[2]), 5);
    BOOST_CHECK_EQUAL(workspace.from_clique_arc[3], false);
    BOOST_CHECK_EQUAL(workspace.from_clique_arc[2], false);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(workspace.lower_bounds[0]),
                      from_alias<std::int32_t>(INVALID_EDGE_DURATION));
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(workspace.lower_bounds[1]),
                      from_alias<std::int32_t>(INVALID_EDGE_DURATION));
}

BOOST_AUTO_TEST_CASE(reverse_lower_bound_target_set_matches_pointwise_minimum_of_per_target_results)
{
    MockFacade facade;

    const auto source_zero_phantom = MakeZeroTraversalPhantom(0);
    const auto source_one_phantom = MakeZeroTraversalPhantom(1);
    const auto target_two_phantom = MakeZeroTraversalPhantom(2);
    const auto target_three_phantom = MakeZeroTraversalPhantom(3);
    const std::vector<engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint>
        source_endpoints{{&source_zero_phantom, 0, false}, {&source_one_phantom, 1, false}};
    const std::vector<engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint>
        target_endpoints{{&target_two_phantom, 2, false}, {&target_three_phantom, 3, false}};
    const auto shared_query_level_policy =
        engine::routing_algorithms::mld::temporal::overlay::detail::MakeSharedEndpointQueryLevelPolicy(
            facade, source_endpoints, target_endpoints);

    const auto lower_bounds_to_two =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBounds(
            facade, target_endpoints[0], {}, shared_query_level_policy);
    const auto lower_bounds_to_three =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBounds(
            facade, target_endpoints[1], {}, shared_query_level_policy);
    const auto shared_lower_bounds =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBoundsForTargets(
            facade, target_endpoints, {}, shared_query_level_policy);

    BOOST_REQUIRE_EQUAL(shared_lower_bounds.size(), lower_bounds_to_two.size());
    BOOST_REQUIRE_EQUAL(shared_lower_bounds.size(), lower_bounds_to_three.size());

    const auto pointwise_minimum = [](const EdgeDuration lhs, const EdgeDuration rhs)
    {
        if (lhs == INVALID_EDGE_DURATION)
        {
            return rhs;
        }
        if (rhs == INVALID_EDGE_DURATION)
        {
            return lhs;
        }
        return std::min(lhs, rhs);
    };

    for (std::size_t index = 0; index < shared_lower_bounds.size(); ++index)
    {
        BOOST_CHECK_EQUAL(shared_lower_bounds[index],
                          pointwise_minimum(lower_bounds_to_two[index],
                                            lower_bounds_to_three[index]));
    }
}

BOOST_AUTO_TEST_CASE(
    bounded_reverse_expansion_matches_full_reverse_for_nodes_within_bound_and_resume_reaches_full_result)
{
    MockFacade facade;

    const auto source_phantom = MakeZeroTraversalPhantom(0);
    const auto target_phantom = MakeZeroTraversalPhantom(3);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 3, false};
    const std::vector<engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint>
        source_endpoints{source};
    const std::vector<engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint>
        target_endpoints{target};
    const auto shared_query_level_policy =
        engine::routing_algorithms::mld::temporal::overlay::detail::MakeSharedEndpointQueryLevelPolicy(
            facade, source_endpoints, target_endpoints);

    const auto full_lower_bounds =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBoundsForTargets(
            facade, target_endpoints, {}, shared_query_level_policy);

    engine::routing_algorithms::mld::temporal::overlay::detail::ReverseLowerBoundWorkspace
        workspace{facade.GetNumberOfNodes()};
    engine::routing_algorithms::mld::temporal::overlay::detail::ReverseLowerBoundSearchStats stats;
    engine::routing_algorithms::mld::temporal::overlay::detail::InitializeReverseLowerBoundsForTargets(
        facade, target_endpoints, workspace, stats);

    engine::routing_algorithms::mld::temporal::overlay::detail::AdvanceReverseLowerBoundsToBound(
        facade, {}, shared_query_level_policy, workspace, stats, EdgeDuration{5});

    BOOST_CHECK_EQUAL_COLLECTIONS(workspace.lower_bounds.begin(),
                                  workspace.lower_bounds.end(),
                                  full_lower_bounds.begin(),
                                  full_lower_bounds.end());

    const auto first_frontier =
        engine::routing_algorithms::mld::temporal::overlay::detail::PeekNextReverseLowerBoundCost(
            workspace);
    BOOST_REQUIRE(first_frontier);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(*first_frontier), 10);

    engine::routing_algorithms::mld::temporal::overlay::detail::AdvanceReverseLowerBoundsToBound(
        facade, {}, shared_query_level_policy, workspace, stats, EdgeDuration{10});

    BOOST_CHECK_EQUAL_COLLECTIONS(workspace.lower_bounds.begin(),
                                  workspace.lower_bounds.end(),
                                  full_lower_bounds.begin(),
                                  full_lower_bounds.end());

    const auto second_frontier =
        engine::routing_algorithms::mld::temporal::overlay::detail::PeekNextReverseLowerBoundCost(
            workspace);
    BOOST_REQUIRE(second_frontier);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(*second_frontier), 15);

    engine::routing_algorithms::mld::temporal::overlay::detail::DrainReverseLowerBounds(
        facade, {}, shared_query_level_policy, workspace, stats);

    BOOST_CHECK_EQUAL_COLLECTIONS(workspace.lower_bounds.begin(),
                                  workspace.lower_bounds.end(),
                                  full_lower_bounds.begin(),
                                  full_lower_bounds.end());
    BOOST_CHECK(!engine::routing_algorithms::mld::temporal::overlay::detail::PeekNextReverseLowerBoundCost(
        workspace));
}

BOOST_AUTO_TEST_CASE(growing_bounded_shared_reverse_search_finds_exact_path_after_small_seed_bound)
{
    MockFacade facade;

    const auto source_phantom = MakeZeroTraversalPhantom(0);
    const auto target_phantom = MakeZeroTraversalPhantom(3);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 3, false};
    const std::vector<engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint>
        source_endpoints{source};
    const std::vector<engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint>
        target_endpoints{target};
    const auto shared_query_level_policy =
        engine::routing_algorithms::mld::temporal::overlay::detail::MakeSharedEndpointQueryLevelPolicy(
            facade, source_endpoints, target_endpoints);

    engine::routing_algorithms::mld::temporal::overlay::detail::ReverseLowerBoundWorkspace
        workspace{facade.GetNumberOfNodes()};
    engine::routing_algorithms::mld::temporal::overlay::detail::ReverseLowerBoundSearchStats stats;
    engine::routing_algorithms::mld::temporal::overlay::detail::InitializeReverseLowerBoundsForTargets(
        facade, target_endpoints, workspace, stats);

    auto current_bound = EdgeDuration{4};
    std::size_t reverse_bound_expansions = 0;
    engine::routing_algorithms::mld::temporal::overlay::TemporalOverlayPath best_path;

    while (true)
    {
        ++reverse_bound_expansions;
        engine::routing_algorithms::mld::temporal::overlay::detail::AdvanceReverseLowerBoundsToBound(
            facade, {}, shared_query_level_policy, workspace, stats, current_bound);

        const auto candidate = engine::routing_algorithms::mld::temporal::overlay::Search(
            facade,
            source,
            target,
            mondayMidnightUtc(),
            shared_query_level_policy,
            workspace.lower_bounds);
        if (candidate.is_valid() &&
            (!best_path.is_valid() || candidate.total_duration < best_path.total_duration))
        {
            best_path = candidate;
        }

        const auto next_frontier =
            engine::routing_algorithms::mld::temporal::overlay::detail::PeekNextReverseLowerBoundCost(
                workspace);
        if (best_path.is_valid() && (!next_frontier || best_path.total_duration <= *next_frontier))
        {
            break;
        }

        BOOST_REQUIRE(next_frontier);
        current_bound = best_path.is_valid()
                            ? best_path.total_duration
                            : engine::routing_algorithms::mld::temporal::overlay::detail::
                                  GetNextReverseLowerBoundExpansion(current_bound, *next_frontier);
    }

    const auto full_path = engine::routing_algorithms::mld::temporal::overlay::Search(
        facade, source, target, mondayMidnightUtc(), shared_query_level_policy);

    BOOST_REQUIRE(best_path.is_valid());
    BOOST_REQUIRE(full_path.is_valid());
    BOOST_CHECK(reverse_bound_expansions > 1U);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(best_path.total_duration),
                      from_alias<std::int32_t>(full_path.total_duration));
    BOOST_REQUIRE_EQUAL(best_path.packed_path.size(), full_path.packed_path.size());
    for (std::size_t index = 0; index < best_path.packed_path.size(); ++index)
    {
        BOOST_CHECK_EQUAL(best_path.packed_path[index].from, full_path.packed_path[index].from);
        BOOST_CHECK_EQUAL(best_path.packed_path[index].to, full_path.packed_path[index].to);
        BOOST_CHECK_EQUAL(best_path.packed_path[index].is_overlay,
                          full_path.packed_path[index].is_overlay);
        BOOST_CHECK_EQUAL(best_path.packed_path[index].overlay_level,
                          full_path.packed_path[index].overlay_level);
    }
}

BOOST_AUTO_TEST_CASE(reverse_lower_bounds_use_incoming_shortcut_row_view)
{
    IncomingShortcutRowOnlyFacade facade;

    const auto source_phantom = MakeZeroTraversalPhantom(0);
    const auto target_phantom = MakeZeroTraversalPhantom(3);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 3, false};
    const std::vector<engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint>
        source_endpoints{source};
    const std::vector<engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint>
        target_endpoints{target};
    const auto shared_query_level_policy =
        engine::routing_algorithms::mld::temporal::overlay::detail::MakeSharedEndpointQueryLevelPolicy(
            facade, source_endpoints, target_endpoints);

    const auto lower_bounds =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBounds(
            facade, target, {}, shared_query_level_policy);

    BOOST_REQUIRE_EQUAL(lower_bounds.size(), facade.GetNumberOfNodes());
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(lower_bounds[3]), 0);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(lower_bounds[2]), 10);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(lower_bounds[1]), 20);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(lower_bounds[0]), 15);
}

BOOST_AUTO_TEST_CASE(precomputed_target_set_reverse_lower_bounds_search_matches_exact_target_search)
{
    MockFacade facade;

    const auto source_zero_phantom = MakeZeroTraversalPhantom(0);
    const auto source_one_phantom = MakeZeroTraversalPhantom(1);
    const auto target_two_phantom = MakeZeroTraversalPhantom(2);
    const auto target_three_phantom = MakeZeroTraversalPhantom(3);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_zero_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_three_phantom, 3, false};
    const std::vector<engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint>
        source_endpoints{source, {&source_one_phantom, 1, false}};
    const std::vector<engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint>
        target_endpoints{{&target_two_phantom, 2, false}, target};
    const auto shared_query_level_policy =
        engine::routing_algorithms::mld::temporal::overlay::detail::MakeSharedEndpointQueryLevelPolicy(
            facade, source_endpoints, target_endpoints);
    const auto shared_reverse_lower_bounds =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBoundsForTargets(
            facade, target_endpoints, {}, shared_query_level_policy);

    const auto exact_target_path = engine::routing_algorithms::mld::temporal::overlay::Search(
        facade, source, target, mondayMidnightUtc(), shared_query_level_policy);
    const auto precomputed_path = engine::routing_algorithms::mld::temporal::overlay::Search(
        facade,
        source,
        target,
        mondayMidnightUtc(),
        shared_query_level_policy,
        shared_reverse_lower_bounds);

    BOOST_REQUIRE(exact_target_path.is_valid());
    BOOST_REQUIRE(precomputed_path.is_valid());
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(precomputed_path.total_duration),
                      from_alias<std::int32_t>(exact_target_path.total_duration));
    BOOST_REQUIRE_EQUAL(precomputed_path.packed_path.size(), exact_target_path.packed_path.size());

    for (std::size_t i = 0; i < exact_target_path.packed_path.size(); ++i)
    {
        BOOST_CHECK_EQUAL(precomputed_path.packed_path[i].from, exact_target_path.packed_path[i].from);
        BOOST_CHECK_EQUAL(precomputed_path.packed_path[i].to, exact_target_path.packed_path[i].to);
        BOOST_CHECK_EQUAL(precomputed_path.packed_path[i].is_overlay,
                          exact_target_path.packed_path[i].is_overlay);
        BOOST_CHECK_EQUAL(precomputed_path.packed_path[i].overlay_level,
                          exact_target_path.packed_path[i].overlay_level);
    }
}

BOOST_AUTO_TEST_CASE(shared_target_set_top_level_pair_loop_matches_exact_target_pair_loop)
{
    MockFacade facade;

    const auto source_zero_phantom = MakeZeroTraversalPhantom(0);
    const auto source_one_phantom = MakeZeroTraversalPhantom(1);
    const auto target_two_phantom = MakeZeroTraversalPhantom(2);
    const auto target_three_phantom = MakeZeroTraversalPhantom(3);
    const std::vector<engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint>
        source_endpoints{{&source_zero_phantom, 0, false}, {&source_one_phantom, 1, false}};
    const std::vector<engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint>
        target_endpoints{{&target_two_phantom, 2, false}, {&target_three_phantom, 3, false}};
    const auto shared_query_level_policy =
        engine::routing_algorithms::mld::temporal::overlay::detail::MakeSharedEndpointQueryLevelPolicy(
            facade, source_endpoints, target_endpoints);
    const auto shared_reverse_lower_bounds =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBoundsForTargets(
            facade, target_endpoints, {}, shared_query_level_policy);

    engine::routing_algorithms::mld::temporal::overlay::TemporalOverlayPath exact_target_best_path;
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint *exact_target_best_source =
        nullptr;
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint *exact_target_best_target =
        nullptr;
    for (const auto &source : source_endpoints)
    {
        for (const auto &target : target_endpoints)
        {
            const auto candidate = engine::routing_algorithms::mld::temporal::overlay::Search(
                facade, source, target, mondayMidnightUtc(), shared_query_level_policy);
            if (!candidate.is_valid() ||
                (exact_target_best_path.is_valid() &&
                 candidate.total_duration >= exact_target_best_path.total_duration))
            {
                continue;
            }

            exact_target_best_path = candidate;
            exact_target_best_source = &source;
            exact_target_best_target = &target;
        }
    }

    engine::routing_algorithms::mld::temporal::overlay::TemporalOverlayPath
        shared_target_set_best_path;
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint
        *shared_target_set_best_source = nullptr;
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint
        *shared_target_set_best_target = nullptr;
    for (const auto &source : source_endpoints)
    {
        for (const auto &target : target_endpoints)
        {
            const auto candidate = engine::routing_algorithms::mld::temporal::overlay::Search(
                facade,
                source,
                target,
                mondayMidnightUtc(),
                shared_query_level_policy,
                shared_reverse_lower_bounds);
            if (!candidate.is_valid() ||
                (shared_target_set_best_path.is_valid() &&
                 candidate.total_duration >= shared_target_set_best_path.total_duration))
            {
                continue;
            }

            shared_target_set_best_path = candidate;
            shared_target_set_best_source = &source;
            shared_target_set_best_target = &target;
        }
    }

    BOOST_REQUIRE(exact_target_best_path.is_valid());
    BOOST_REQUIRE(shared_target_set_best_path.is_valid());
    BOOST_REQUIRE(exact_target_best_source != nullptr);
    BOOST_REQUIRE(exact_target_best_target != nullptr);
    BOOST_REQUIRE(shared_target_set_best_source != nullptr);
    BOOST_REQUIRE(shared_target_set_best_target != nullptr);

    const auto exact_target_unpacked = engine::routing_algorithms::mld::temporal::overlay::UnpackPath(
        facade,
        *exact_target_best_source,
        *exact_target_best_target,
        mondayMidnightUtc(),
        exact_target_best_path);
    const auto shared_target_set_unpacked =
        engine::routing_algorithms::mld::temporal::overlay::UnpackPath(
            facade,
            *shared_target_set_best_source,
            *shared_target_set_best_target,
            mondayMidnightUtc(),
            shared_target_set_best_path);

    BOOST_REQUIRE(exact_target_unpacked.is_valid());
    BOOST_REQUIRE(shared_target_set_unpacked.is_valid());
    BOOST_CHECK_EQUAL(shared_target_set_best_source->node, exact_target_best_source->node);
    BOOST_CHECK_EQUAL(shared_target_set_best_target->node, exact_target_best_target->node);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(shared_target_set_best_path.total_duration),
                      from_alias<std::int32_t>(exact_target_best_path.total_duration));
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(shared_target_set_unpacked.total_duration),
                      from_alias<std::int32_t>(exact_target_unpacked.total_duration));
    BOOST_REQUIRE_EQUAL(shared_target_set_best_path.packed_path.size(),
                        exact_target_best_path.packed_path.size());
    for (std::size_t i = 0; i < exact_target_best_path.packed_path.size(); ++i)
    {
        BOOST_CHECK_EQUAL(shared_target_set_best_path.packed_path[i].from,
                          exact_target_best_path.packed_path[i].from);
        BOOST_CHECK_EQUAL(shared_target_set_best_path.packed_path[i].to,
                          exact_target_best_path.packed_path[i].to);
        BOOST_CHECK_EQUAL(shared_target_set_best_path.packed_path[i].is_overlay,
                          exact_target_best_path.packed_path[i].is_overlay);
        BOOST_CHECK_EQUAL(shared_target_set_best_path.packed_path[i].overlay_level,
                          exact_target_best_path.packed_path[i].overlay_level);
    }
    BOOST_CHECK_EQUAL_COLLECTIONS(shared_target_set_unpacked.nodes.begin(),
                                  shared_target_set_unpacked.nodes.end(),
                                  exact_target_unpacked.nodes.begin(),
                                  exact_target_unpacked.nodes.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(shared_target_set_unpacked.edges.begin(),
                                  shared_target_set_unpacked.edges.end(),
                                  exact_target_unpacked.edges.begin(),
                                  exact_target_unpacked.edges.end());
}

BOOST_AUTO_TEST_CASE(reverse_lower_bound_cache_reuses_identical_restricted_requests)
{
    MockFacade facade;

    const auto source_phantom = MakeZeroTraversalPhantom(0);
    const auto target_phantom = MakeZeroTraversalPhantom(3);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 3, false};
    const engine::routing_algorithms::mld::temporal::overlay::detail::PairQueryLevelPolicy
        query_level_policy{facade.GetMultiLevelPartition(), source, target};
    engine::routing_algorithms::mld::temporal::overlay::detail::ReverseLowerBoundCache cache{
        facade.GetNumberOfNodes()};
    const auto restriction =
        engine::routing_algorithms::mld::temporal::overlay::detail::SearchRestriction{
            true, LevelID{1}, CellID{0}};

    const auto &first =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBounds(
            facade, target, restriction, query_level_policy, cache);
    const auto &second =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBounds(
            facade, target, restriction, query_level_policy, cache);

    BOOST_CHECK_EQUAL(cache.GetMissCount(), 1U);
    BOOST_CHECK_EQUAL(cache.GetHitCount(), 1U);
    BOOST_CHECK_EQUAL(cache.GetEntryCount(), 1U);
    BOOST_CHECK_EQUAL_COLLECTIONS(first.begin(), first.end(), second.begin(), second.end());
}

BOOST_AUTO_TEST_CASE(reverse_lower_bound_cache_keeps_restricted_requests_separate)
{
    MockFacade facade;

    const auto source_phantom = MakeZeroTraversalPhantom(0);
    const auto target_phantom = MakeZeroTraversalPhantom(3);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 3, false};
    const engine::routing_algorithms::mld::temporal::overlay::detail::PairQueryLevelPolicy
        query_level_policy{facade.GetMultiLevelPartition(), source, target};
    engine::routing_algorithms::mld::temporal::overlay::detail::ReverseLowerBoundCache cache{
        facade.GetNumberOfNodes()};
    const auto level_one_restriction =
        engine::routing_algorithms::mld::temporal::overlay::detail::SearchRestriction{
            true, LevelID{1}, CellID{0}};
    const auto level_zero_restriction =
        engine::routing_algorithms::mld::temporal::overlay::detail::SearchRestriction{
            true, LevelID{0}, CellID{0}};

    const auto &level_one =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBounds(
            facade, target, level_one_restriction, query_level_policy, cache);
    const auto &level_zero =
        engine::routing_algorithms::mld::temporal::overlay::detail::ComputeReverseLowerBounds(
            facade, target, level_zero_restriction, query_level_policy, cache);

    BOOST_CHECK_EQUAL(cache.GetMissCount(), 2U);
    BOOST_CHECK_EQUAL(cache.GetHitCount(), 0U);
    BOOST_CHECK_EQUAL(cache.GetEntryCount(), 2U);
    BOOST_CHECK_EQUAL(level_one.size(), level_zero.size());
}

BOOST_AUTO_TEST_CASE(recursive_unpacking_expands_overlay_edge_to_base_path)
{
    MockFacade facade;

    const auto source_phantom = MakeZeroTraversalPhantom(0);
    const auto target_phantom = MakeZeroTraversalPhantom(3);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 3, false};

    const auto path = engine::routing_algorithms::mld::temporal::overlay::Search(
        facade, source, target, mondayMidnightUtc());
    const auto unpacked = engine::routing_algorithms::mld::temporal::overlay::UnpackPath(
        facade, source, target, mondayMidnightUtc(), path);
    const std::array<NodeID, 4> expected_nodes{{0, 1, 2, 3}};

    BOOST_REQUIRE(unpacked.is_valid());
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(unpacked.total_duration), 15);
    BOOST_REQUIRE_EQUAL(unpacked.nodes.size(), 4U);
    BOOST_CHECK_EQUAL_COLLECTIONS(unpacked.nodes.begin(),
                                  unpacked.nodes.end(),
                                  expected_nodes.begin(),
                                  expected_nodes.end());
    BOOST_CHECK_EQUAL(unpacked.edges.size(), 3U);
}

BOOST_AUTO_TEST_CASE(cached_overlay_unpack_matches_uncached_unpack_and_reuses_restricted_reverse_bounds)
{
    MockFacade facade;

    const auto source_phantom = MakeZeroTraversalPhantom(0);
    const auto target_phantom = MakeZeroTraversalPhantom(3);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 3, false};
    engine::routing_algorithms::mld::temporal::overlay::detail::ReverseLowerBoundCache cache{
        facade.GetNumberOfNodes()};

    const auto path = engine::routing_algorithms::mld::temporal::overlay::Search(
        facade, source, target, mondayMidnightUtc());
    const auto uncached_unpacked = engine::routing_algorithms::mld::temporal::overlay::UnpackPath(
        facade, source, target, mondayMidnightUtc(), path);
    const auto cached_unpacked_first =
        engine::routing_algorithms::mld::temporal::overlay::UnpackPath(
            facade, source, target, mondayMidnightUtc(), path, cache);
    const auto cached_unpacked_second =
        engine::routing_algorithms::mld::temporal::overlay::UnpackPath(
            facade, source, target, mondayMidnightUtc(), path, cache);

    BOOST_REQUIRE(path.is_valid());
    BOOST_REQUIRE(uncached_unpacked.is_valid());
    BOOST_REQUIRE(cached_unpacked_first.is_valid());
    BOOST_REQUIRE(cached_unpacked_second.is_valid());
    BOOST_CHECK_EQUAL(cache.GetMissCount(), 1U);
    BOOST_CHECK_EQUAL(cache.GetHitCount(), 1U);
    BOOST_CHECK_EQUAL(cache.GetEntryCount(), 1U);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(cached_unpacked_first.total_duration),
                      from_alias<std::int32_t>(uncached_unpacked.total_duration));
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(cached_unpacked_second.total_duration),
                      from_alias<std::int32_t>(uncached_unpacked.total_duration));
    BOOST_CHECK_EQUAL_COLLECTIONS(cached_unpacked_first.nodes.begin(),
                                  cached_unpacked_first.nodes.end(),
                                  uncached_unpacked.nodes.begin(),
                                  uncached_unpacked.nodes.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(cached_unpacked_second.nodes.begin(),
                                  cached_unpacked_second.nodes.end(),
                                  uncached_unpacked.nodes.begin(),
                                  uncached_unpacked.nodes.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(cached_unpacked_first.edges.begin(),
                                  cached_unpacked_first.edges.end(),
                                  uncached_unpacked.edges.begin(),
                                  uncached_unpacked.edges.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(cached_unpacked_second.edges.begin(),
                                  cached_unpacked_second.edges.end(),
                                  uncached_unpacked.edges.begin(),
                                  uncached_unpacked.edges.end());
}

BOOST_AUTO_TEST_CASE(overlay_query_uses_bucketed_shortcut_duration)
{
    MockFacade facade;

    const auto source_phantom = MakeZeroTraversalPhantom(0);
    const auto target_phantom = MakeZeroTraversalPhantom(3);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 3, false};

    const auto departure = mondayMidnightUtc() + 5 * 60;
    const auto path =
        engine::routing_algorithms::mld::temporal::overlay::Search(facade, source, target, departure);

    BOOST_REQUIRE(path.is_valid());
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(path.total_duration), 21);
}

BOOST_AUTO_TEST_CASE(unpacking_uses_departure_clock_for_direct_overlay_shortcut)
{
    ArrivalSensitiveFacade facade;

    const auto source_phantom = MakeZeroTraversalPhantom(1);
    const auto target_phantom = MakeZeroTraversalPhantom(4);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 1, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 4, false};

    const auto path = engine::routing_algorithms::mld::temporal::overlay::Search(
        facade, source, target, mondayMidnightUtc());
    const auto unpacked = engine::routing_algorithms::mld::temporal::overlay::UnpackPath(
        facade, source, target, mondayMidnightUtc(), path);
    const std::array<NodeID, 3> expected_nodes{{1, 2, 4}};

    BOOST_REQUIRE(path.is_valid());
    BOOST_REQUIRE_EQUAL(path.packed_path.size(), 1U);
    BOOST_CHECK(path.packed_path.front().is_overlay);
    BOOST_CHECK_EQUAL(path.packed_path.front().overlay_level, LevelID{1});

    BOOST_REQUIRE(unpacked.is_valid());
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(unpacked.total_duration), 200);
    BOOST_REQUIRE_EQUAL(unpacked.nodes.size(), expected_nodes.size());
    BOOST_CHECK_EQUAL_COLLECTIONS(unpacked.nodes.begin(),
                                  unpacked.nodes.end(),
                                  expected_nodes.begin(),
                                  expected_nodes.end());
}

BOOST_AUTO_TEST_CASE(unpacking_uses_realized_arrival_clock_after_prefix_edge)
{
    ArrivalSensitiveFacade facade;

    const auto source_phantom = MakeZeroTraversalPhantom(0);
    const auto target_phantom = MakeZeroTraversalPhantom(4);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 0, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 4, false};

    const auto path = engine::routing_algorithms::mld::temporal::overlay::Search(
        facade, source, target, mondayMidnightUtc());
    const auto unpacked = engine::routing_algorithms::mld::temporal::overlay::UnpackPath(
        facade, source, target, mondayMidnightUtc(), path);
    const std::array<NodeID, 4> expected_nodes{{0, 1, 3, 4}};

    BOOST_REQUIRE(path.is_valid());
    BOOST_REQUIRE_EQUAL(path.packed_path.size(), 2U);
    BOOST_CHECK(!path.packed_path[0].is_overlay);
    BOOST_CHECK_EQUAL(path.packed_path[0].from, 0);
    BOOST_CHECK_EQUAL(path.packed_path[0].to, 1);
    BOOST_CHECK(path.packed_path[1].is_overlay);
    BOOST_CHECK_EQUAL(path.packed_path[1].from, 1);
    BOOST_CHECK_EQUAL(path.packed_path[1].to, 4);
    BOOST_CHECK_EQUAL(path.packed_path[1].overlay_level, LevelID{1});

    BOOST_REQUIRE(unpacked.is_valid());
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(unpacked.total_duration), 800);
    BOOST_REQUIRE_EQUAL(unpacked.nodes.size(), expected_nodes.size());
    BOOST_CHECK_EQUAL_COLLECTIONS(unpacked.nodes.begin(),
                                  unpacked.nodes.end(),
                                  expected_nodes.begin(),
                                  expected_nodes.end());
}

BOOST_AUTO_TEST_CASE(overlay_query_matches_exact_temporal_dijkstra_when_static_and_temporal_differ)
{
    StaticTemporalDivergenceFacade facade;

    const auto source_phantom = MakeZeroTraversalPhantom(1);
    const auto target_phantom = MakeZeroTraversalPhantom(4);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 1, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 4, false};

    const auto departure = mondayMidnightUtc();
    const auto static_exact = ExactStaticBaseDijkstra(facade, 1, 4);
    const auto temporal_exact = ExactTemporalBaseDijkstra(facade, 1, 4, departure);
    const auto overlay_path =
        engine::routing_algorithms::mld::temporal::overlay::Search(facade, source, target, departure);
    const auto overlay_unpacked = engine::routing_algorithms::mld::temporal::overlay::UnpackPath(
        facade, source, target, departure, overlay_path);
    const std::array<NodeID, 3> expected_static_nodes{{1, 2, 4}};
    const std::array<NodeID, 3> expected_temporal_nodes{{1, 3, 4}};

    BOOST_REQUIRE(static_exact.is_valid());
    BOOST_REQUIRE(temporal_exact.is_valid());
    BOOST_REQUIRE(overlay_unpacked.is_valid());

    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(static_exact.total_duration), 20);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(temporal_exact.total_duration), 30);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(overlay_unpacked.total_duration), 30);
    BOOST_CHECK_EQUAL_COLLECTIONS(static_exact.nodes.begin(),
                                  static_exact.nodes.end(),
                                  expected_static_nodes.begin(),
                                  expected_static_nodes.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(temporal_exact.nodes.begin(),
                                  temporal_exact.nodes.end(),
                                  expected_temporal_nodes.begin(),
                                  expected_temporal_nodes.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(overlay_unpacked.nodes.begin(),
                                  overlay_unpacked.nodes.end(),
                                  temporal_exact.nodes.begin(),
                                  temporal_exact.nodes.end());
}

BOOST_AUTO_TEST_CASE(overlay_query_switches_shortcut_choice_with_departure_bucket_and_matches_exact)
{
    StaticTemporalDivergenceFacade facade;

    const auto source_phantom = MakeZeroTraversalPhantom(1);
    const auto target_phantom = MakeZeroTraversalPhantom(4);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 1, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 4, false};

    const auto departure = mondayMidnightUtc() + 60;
    const auto temporal_exact = ExactTemporalBaseDijkstra(facade, 1, 4, departure);
    const auto overlay_path =
        engine::routing_algorithms::mld::temporal::overlay::Search(facade, source, target, departure);
    const auto overlay_unpacked = engine::routing_algorithms::mld::temporal::overlay::UnpackPath(
        facade, source, target, departure, overlay_path);
    const std::array<NodeID, 3> expected_temporal_nodes{{1, 2, 4}};

    BOOST_REQUIRE(temporal_exact.is_valid());
    BOOST_REQUIRE(overlay_unpacked.is_valid());

    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(temporal_exact.total_duration), 20);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(overlay_unpacked.total_duration), 20);
    BOOST_CHECK_EQUAL_COLLECTIONS(temporal_exact.nodes.begin(),
                                  temporal_exact.nodes.end(),
                                  expected_temporal_nodes.begin(),
                                  expected_temporal_nodes.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(overlay_unpacked.nodes.begin(),
                                  overlay_unpacked.nodes.end(),
                                  temporal_exact.nodes.begin(),
                                  temporal_exact.nodes.end());
}

BOOST_AUTO_TEST_CASE(overlay_query_falls_back_to_lower_level_edges_when_shortcut_is_missing)
{
    MissingShortcutFallbackFacade facade;

    const auto source_phantom = MakeZeroTraversalPhantom(1);
    const auto target_phantom = MakeZeroTraversalPhantom(4);
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint source{
        &source_phantom, 1, false};
    const engine::routing_algorithms::mld::temporal::DirectedPhantomEndpoint target{
        &target_phantom, 4, false};

    const auto departure = mondayMidnightUtc();
    const auto temporal_exact = ExactTemporalBaseDijkstra(facade, 1, 4, departure);
    const auto overlay_path =
        engine::routing_algorithms::mld::temporal::overlay::Search(facade, source, target, departure);
    const auto overlay_unpacked = engine::routing_algorithms::mld::temporal::overlay::UnpackPath(
        facade, source, target, departure, overlay_path);
    const std::array<NodeID, 3> expected_temporal_nodes{{1, 3, 4}};

    BOOST_REQUIRE(temporal_exact.is_valid());
    BOOST_REQUIRE(overlay_path.is_valid());
    BOOST_REQUIRE(overlay_unpacked.is_valid());

    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(temporal_exact.total_duration), 30);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(overlay_path.total_duration), 30);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(overlay_unpacked.total_duration), 30);
    BOOST_CHECK(overlay_path.packed_path.size() >= 2U);
    BOOST_CHECK(std::ranges::all_of(
        overlay_path.packed_path, [](const auto &packed_edge) { return !packed_edge.is_overlay; }));
    BOOST_CHECK_EQUAL_COLLECTIONS(temporal_exact.nodes.begin(),
                                  temporal_exact.nodes.end(),
                                  expected_temporal_nodes.begin(),
                                  expected_temporal_nodes.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(overlay_unpacked.nodes.begin(),
                                  overlay_unpacked.nodes.end(),
                                  temporal_exact.nodes.begin(),
                                  temporal_exact.nodes.end());
}

BOOST_AUTO_TEST_SUITE_END()
