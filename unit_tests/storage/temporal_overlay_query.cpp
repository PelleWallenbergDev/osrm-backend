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

struct MockFacade
{
    MockPartition partition;
    MockCellStorage cells;
    std::vector<MockEdge> edges;
    std::array<std::vector<std::vector<EdgeID>>, 2> border_edges;
    std::vector<std::vector<SegmentDuration>> static_durations;
    std::vector<std::vector<EdgeDuration>> temporal_durations;
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

    void AddEdge(LevelID level, NodeID from, NodeID to, bool forward, bool backward)
    {
        const auto edge_id = static_cast<EdgeID>(edges.size());
        edges.push_back(MockEdge{to, forward, backward, {}});
        border_edges[level][from].push_back(edge_id);
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

struct ArrivalSensitiveFacade
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

struct StaticTemporalDivergenceFacade
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
} // namespace

BOOST_AUTO_TEST_SUITE(temporal_overlay_query)

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
