#ifndef OSRM_ENGINE_DATAFACADE_ALGORITHM_DATAFACADE_HPP
#define OSRM_ENGINE_DATAFACADE_ALGORITHM_DATAFACADE_HPP

#include "contractor/query_edge.hpp"
#include "customizer/edge_based_graph.hpp"
#include "customizer/temporal_profiles.hpp"
#include "extractor/edge_based_edge.hpp"
#include "engine/algorithm.hpp"

#include "partitioner/cell_storage.hpp"
#include "partitioner/multi_level_partition.hpp"

#include "util/filtered_graph.hpp"
#include "util/integer_range.hpp"

namespace osrm::engine::datafacade
{

// Namespace local aliases for algorithms
using CH = routing_algorithms::ch::Algorithm;
using MLD = routing_algorithms::mld::Algorithm;

template <typename AlgorithmT> class AlgorithmDataFacade;

template <> class AlgorithmDataFacade<CH>
{
  public:
    using EdgeData = contractor::QueryEdge::EdgeData;
    using EdgeRange = util::filtered_range<EdgeID, util::vector_view<bool>>;

    virtual ~AlgorithmDataFacade() = default;

    // search graph access
    virtual unsigned GetNumberOfNodes() const = 0;

    virtual unsigned GetNumberOfEdges() const = 0;

    virtual unsigned GetOutDegree(const NodeID edge_based_node_id) const = 0;

    virtual NodeID GetTarget(const EdgeID edge_based_edge_id) const = 0;

    virtual const EdgeData &GetEdgeData(const EdgeID edge_based_edge_id) const = 0;

    virtual EdgeRange GetAdjacentEdgeRange(const NodeID edge_based_node_id) const = 0;

    // searches for a specific edge
    virtual EdgeID FindEdge(const NodeID edge_based_node_from,
                            const NodeID edge_based_node_to) const = 0;

    virtual EdgeID FindEdgeInEitherDirection(const NodeID edge_based_node_from,
                                             const NodeID edge_based_node_to) const = 0;

    virtual EdgeID FindEdgeIndicateIfReverse(const NodeID edge_based_node_from,
                                             const NodeID edge_based_node_to,
                                             bool &result) const = 0;

    virtual EdgeID FindSmallestEdge(const NodeID edge_based_node_from,
                                    const NodeID edge_based_node_to,
                                    const std::function<bool(const EdgeData &)> &filter) const = 0;
};

template <> class AlgorithmDataFacade<MLD>
{
  public:
    using EdgeData = customizer::EdgeBasedGraphEdgeData;
    using EdgeRange = util::range<EdgeID>;
    struct TemporalShortcutRowView
    {
        const NodeID *destinations = nullptr;
        const customizer::TemporalFunctionID *function_ids = nullptr;
        const EdgeDuration *min_durations = nullptr;
        std::size_t size = 0;

        bool empty() const
        {
            return size == 0 || destinations == nullptr || function_ids == nullptr;
        }
    };

    struct TemporalIncomingShortcutRowView
    {
        const NodeID *sources = nullptr;
        const customizer::TemporalFunctionID *function_ids = nullptr;
        const EdgeDuration *min_durations = nullptr;
        std::size_t size = 0;
        std::size_t value_stride = 1;

        bool empty() const
        {
            return size == 0 || sources == nullptr || function_ids == nullptr ||
                   min_durations == nullptr;
        }

        customizer::TemporalFunctionID GetFunctionID(const std::size_t index) const
        {
            return function_ids[index * value_stride];
        }

        EdgeDuration GetMinDuration(const std::size_t index) const
        {
            return min_durations[index * value_stride];
        }
    };

    struct IncomingBorderEdgeRowView
    {
        const EdgeID *edges = nullptr;
        const LevelID *highest_border_levels = nullptr;
        std::size_t size = 0;

        bool empty() const
        {
            return size == 0 || edges == nullptr || highest_border_levels == nullptr;
        }
    };

    virtual ~AlgorithmDataFacade() = default;

    // search graph access
    virtual unsigned GetNumberOfNodes() const = 0;

    virtual unsigned GetMaxBorderNodeID() const = 0;

    virtual unsigned GetNumberOfEdges() const = 0;

    virtual unsigned GetOutDegree(const NodeID edge_based_node_id) const = 0;

    virtual EdgeRange GetAdjacentEdgeRange(const NodeID edge_based_node_id) const = 0;

    virtual EdgeWeight GetNodeWeight(const NodeID edge_based_node_id) const = 0;

    virtual EdgeDuration
    GetNodeDuration(const NodeID edge_based_node_id) const = 0; // TODO: to be removed

    virtual EdgeDistance GetNodeDistance(const NodeID edge_based_node_id) const = 0;

    virtual bool IsForwardEdge(EdgeID edge_based_edge_id) const = 0;

    virtual bool IsBackwardEdge(EdgeID edge_based_edge_id) const = 0;

    virtual NodeID GetTarget(const EdgeID edge_based_edge_id) const = 0;

    virtual const EdgeData &GetEdgeData(const EdgeID edge_based_edge_id) const = 0;

    virtual const partitioner::MultiLevelPartitionView &GetMultiLevelPartition() const = 0;

    virtual const partitioner::CellStorageView &GetCellStorage() const = 0;

    virtual const customizer::CellMetricView &GetCellMetric() const = 0;

    virtual bool HasTemporalShortcut(const LevelID level,
                                     const CellID cell_id,
                                     const NodeID from,
                                     const NodeID to) const = 0;

    virtual EdgeDuration GetTemporalShortcutDuration(const LevelID level,
                                                     const CellID cell_id,
                                                     const NodeID from,
                                                     const NodeID to,
                                                     const std::uint32_t week_bucket) const = 0;

    virtual EdgeDuration GetTemporalShortcutMinDuration(const LevelID level,
                                                        const CellID cell_id,
                                                        const NodeID from,
                                                        const NodeID to) const = 0;

    virtual TemporalShortcutRowView GetTemporalShortcutRow(const LevelID level,
                                                           const CellID cell_id,
                                                           const NodeID from) const = 0;

    virtual TemporalIncomingShortcutRowView
    GetTemporalIncomingShortcutRow(const LevelID level,
                                   const CellID cell_id,
                                   const NodeID to) const = 0;

    virtual IncomingBorderEdgeRowView
    GetIncomingBorderEdgeRow(const NodeID edge_based_node_id) const = 0;

    virtual EdgeDuration
    GetTemporalFunctionDuration(const customizer::TemporalFunctionID function_id,
                                const std::uint32_t week_bucket) const = 0;

    virtual EdgeRange GetBorderEdgeRange(const LevelID level,
                                         const NodeID edge_based_node_id) const = 0;

    // searches for a specific edge
    virtual EdgeID FindEdge(const NodeID edge_based_node_from,
                            const NodeID edge_based_node_to) const = 0;
};
} // namespace osrm::engine::datafacade

#endif
