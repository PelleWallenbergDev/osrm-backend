#ifndef OSRM_PARTITIONER_CUSTOMIZE_CELL_STORAGE_HPP
#define OSRM_PARTITIONER_CUSTOMIZE_CELL_STORAGE_HPP

#include "partitioner/multi_level_partition.hpp"

#include "util/assert.hpp"
#include "util/for_each_range.hpp"
#include "util/log.hpp"
#include "util/typedefs.hpp"
#include "util/vector_view.hpp"

#include "storage/io_fwd.hpp"
#include "storage/shared_memory_ownership.hpp"

#include "customizer/cell_metric.hpp"

#include <ranges>
#include <tbb/parallel_sort.h>

#include <algorithm>
#include <numeric>
#include <utility>
#include <vector>

namespace osrm::partitioner
{
namespace detail
{
template <storage::Ownership Ownership> class CellStorageImpl;
}
using CellStorage = detail::CellStorageImpl<storage::Ownership::Container>;
using CellStorageView = detail::CellStorageImpl<storage::Ownership::View>;

namespace serialization
{
template <storage::Ownership Ownership>
inline void read(storage::tar::FileReader &reader,
                 const std::string &name,
                 detail::CellStorageImpl<Ownership> &storage);
template <storage::Ownership Ownership>
inline void write(storage::tar::FileWriter &writer,
                  const std::string &name,
                  const detail::CellStorageImpl<Ownership> &storage);
} // namespace serialization

namespace detail
{
template <storage::Ownership Ownership> class CellStorageImpl
{
  public:
    using ValueOffset = std::uint64_t;
    using BoundaryOffset = std::uint64_t;
    using BoundarySize = std::uint32_t;
    using BoundaryIndex = BoundaryOffset;

    static constexpr auto INVALID_VALUE_OFFSET = std::numeric_limits<ValueOffset>::max();
    static constexpr auto INVALID_BOUNDARY_OFFSET = std::numeric_limits<BoundaryOffset>::max();
    static constexpr auto INVALID_BOUNDARY_INDEX = INVALID_BOUNDARY_OFFSET;

    struct CellData
    {
        ValueOffset value_offset = INVALID_VALUE_OFFSET;
        BoundaryOffset source_boundary_offset = INVALID_BOUNDARY_OFFSET;
        BoundaryOffset destination_boundary_offset = INVALID_BOUNDARY_OFFSET;
        BoundarySize num_source_nodes = 0;
        BoundarySize num_destination_nodes = 0;
    };

  private:
    template <typename T> using Vector = util::ViewOrVector<T, Ownership>;

    // Implementation of the cell view. We need a template parameter here
    // because we need to derive a read-only and read-write view from this.
    template <typename WeightValueT, typename DurationValueT, typename DistanceValueT>
    class CellImpl
    {
      private:
        using WeightPtrT = WeightValueT *;
        using DurationPtrT = DurationValueT *;
        using DistancePtrT = DistanceValueT *;
        BoundarySize num_source_nodes;
        BoundarySize num_destination_nodes;

        WeightPtrT const weights;
        DurationPtrT const durations;
        DistancePtrT const distances;
        const NodeID *const source_boundary;
        const NodeID *const destination_boundary;
        const BoundaryIndex *const source_boundary_index;
        const BoundaryIndex *const destination_boundary_index;
        BoundaryOffset source_boundary_offset;
        BoundaryOffset destination_boundary_offset;
        BoundaryOffset boundary_index_size;

        using RowIterator = WeightPtrT;
        // Possibly replace with
        // http://www.boost.org/doc/libs/1_55_0/libs/range/doc/html/range/reference/adaptors/reference/strided.html

        template <typename ValuePtrT>
        class ColumnIterator : public boost::iterator_facade<ColumnIterator<ValuePtrT>,
                                                             decltype(*std::declval<ValuePtrT>()),
                                                             boost::random_access_traversal_tag>
        {

            using ValueT = decltype(*std::declval<ValuePtrT>());
            using base_t = boost::
                iterator_facade<ColumnIterator<ValueT>, ValueT, boost::random_access_traversal_tag>;

          public:
            using value_type = typename base_t::value_type;
            using difference_type = typename base_t::difference_type;
            using reference = typename base_t::reference;
            using iterator_category = std::random_access_iterator_tag;

            explicit ColumnIterator() : current(nullptr), stride(1) {}

            explicit ColumnIterator(ValuePtrT begin, std::size_t row_length)
                : current(begin), stride(row_length)
            {
                BOOST_ASSERT(begin != nullptr);
            }

          private:
            void increment() { current += stride; }
            void decrement() { current -= stride; }
            void advance(difference_type offset) { current += stride * offset; }
            bool equal(const ColumnIterator &other) const { return current == other.current; }
            reference dereference() const { return *current; }
            difference_type distance_to(const ColumnIterator &other) const
            {
                return (other.current - current) / static_cast<std::intptr_t>(stride);
            }

            friend class ::boost::iterator_core_access;
            ValuePtrT current;
            std::size_t stride;
        };

        BoundaryIndex GetSourceBoundaryPosition(const NodeID node) const
        {
            if (source_boundary_index == nullptr ||
                static_cast<BoundaryOffset>(node) >= boundary_index_size)
            {
                return INVALID_BOUNDARY_INDEX;
            }

            const auto boundary_index = source_boundary_index[node];
            if (boundary_index == INVALID_BOUNDARY_INDEX || boundary_index < source_boundary_offset ||
                boundary_index >= source_boundary_offset + num_source_nodes)
            {
                return INVALID_BOUNDARY_INDEX;
            }

            return boundary_index;
        }

        BoundaryIndex GetDestinationBoundaryPosition(const NodeID node) const
        {
            if (destination_boundary_index == nullptr ||
                static_cast<BoundaryOffset>(node) >= boundary_index_size)
            {
                return INVALID_BOUNDARY_INDEX;
            }

            const auto boundary_index = destination_boundary_index[node];
            if (boundary_index == INVALID_BOUNDARY_INDEX ||
                boundary_index < destination_boundary_offset ||
                boundary_index >= destination_boundary_offset + num_destination_nodes)
            {
                return INVALID_BOUNDARY_INDEX;
            }

            return boundary_index;
        }

        template <typename ValuePtr> auto GetOutRange(const ValuePtr ptr, const NodeID node) const
        {
            const auto boundary_index = GetSourceBoundaryPosition(node);
            if (boundary_index == INVALID_BOUNDARY_INDEX)
                return std::ranges::subrange(ptr, ptr);

            auto row = boundary_index - source_boundary_offset;
            auto begin = ptr + num_destination_nodes * row;
            auto end = begin + num_destination_nodes;
            return std::ranges::subrange(begin, end);
        }

        template <typename ValuePtr> auto GetInRange(const ValuePtr ptr, const NodeID node) const
        {
            const auto boundary_index = GetDestinationBoundaryPosition(node);
            if (boundary_index == INVALID_BOUNDARY_INDEX)
                return std::ranges::subrange(ColumnIterator<ValuePtr>{},
                                             ColumnIterator<ValuePtr>{});

            auto column = boundary_index - destination_boundary_offset;
            auto begin = ColumnIterator<ValuePtr>{ptr + column, num_destination_nodes};
            auto end = ColumnIterator<ValuePtr>{
                ptr + column + num_source_nodes * num_destination_nodes, num_destination_nodes};
            return std::ranges::subrange(begin, end);
        }

      public:
        auto GetOutWeight(NodeID node) const { return GetOutRange(weights, node); }

        auto GetInWeight(NodeID node) const { return GetInRange(weights, node); }

        auto GetOutDuration(NodeID node) const { return GetOutRange(durations, node); }

        auto GetInDuration(NodeID node) const { return GetInRange(durations, node); }

        auto GetInDistance(NodeID node) const { return GetInRange(distances, node); }

        auto GetOutDistance(NodeID node) const { return GetOutRange(distances, node); }

        auto GetSourceNodes() const
        {
            return std::ranges::subrange(source_boundary, source_boundary + num_source_nodes);
        }

        auto GetDestinationNodes() const
        {
            return std::ranges::subrange(destination_boundary,
                                         destination_boundary + num_destination_nodes);
        }

        CellImpl(const CellData &data,
                 WeightPtrT const all_weights,
                 DurationPtrT const all_durations,
                 DistancePtrT const all_distances,
                 const NodeID *const all_sources,
                 const NodeID *const all_destinations,
                 const BoundaryIndex *const all_source_boundary_index,
                 const BoundaryIndex *const all_destination_boundary_index,
                 const BoundaryOffset boundary_index_size_)
            : num_source_nodes{data.num_source_nodes},
              num_destination_nodes{data.num_destination_nodes},
              weights{all_weights + data.value_offset},
              durations{all_durations + data.value_offset},
              distances{all_distances + data.value_offset},
              source_boundary{all_sources + data.source_boundary_offset},
              destination_boundary{all_destinations + data.destination_boundary_offset},
              source_boundary_index{all_source_boundary_index},
              destination_boundary_index{all_destination_boundary_index},
              source_boundary_offset{data.source_boundary_offset},
              destination_boundary_offset{data.destination_boundary_offset},
              boundary_index_size{boundary_index_size_}
        {
            BOOST_ASSERT(all_weights != nullptr);
            BOOST_ASSERT(all_durations != nullptr);
            BOOST_ASSERT(all_distances != nullptr);
            BOOST_ASSERT(num_source_nodes == 0 || all_sources != nullptr);
            BOOST_ASSERT(num_destination_nodes == 0 || all_destinations != nullptr);
        }

        // Consturcts an emptry cell without weights. Useful when only access
        // to the cell structure is needed, without a concrete metric.
        CellImpl(const CellData &data,
                 const NodeID *const all_sources,
                 const NodeID *const all_destinations,
                 const BoundaryIndex *const all_source_boundary_index,
                 const BoundaryIndex *const all_destination_boundary_index,
                 const BoundaryOffset boundary_index_size_)
            : num_source_nodes{data.num_source_nodes},
              num_destination_nodes{data.num_destination_nodes}, weights{nullptr},
              durations{nullptr}, distances{nullptr},
              source_boundary{all_sources + data.source_boundary_offset},
              destination_boundary{all_destinations + data.destination_boundary_offset},
              source_boundary_index{all_source_boundary_index},
              destination_boundary_index{all_destination_boundary_index},
              source_boundary_offset{data.source_boundary_offset},
              destination_boundary_offset{data.destination_boundary_offset},
              boundary_index_size{boundary_index_size_}
        {
            BOOST_ASSERT(num_source_nodes == 0 || all_sources != nullptr);
            BOOST_ASSERT(num_destination_nodes == 0 || all_destinations != nullptr);
        }
    };

    std::size_t LevelIDToIndex(LevelID level) const { return level - 1; }

  public:
    using Cell = CellImpl<EdgeWeight, EdgeDuration, EdgeDistance>;
    using ConstCell = CellImpl<const EdgeWeight, const EdgeDuration, const EdgeDistance>;

    CellStorageImpl() {}

    template <typename GraphT,
              typename = std::enable_if<Ownership == storage::Ownership::Container>>
    CellStorageImpl(const partitioner::MultiLevelPartition &partition, const GraphT &base_graph)
    {
        // pre-allocate storge for CellData so we can have random access to it by cell id
        unsigned number_of_cells = 0;
        for (LevelID level = 1u; level < partition.GetNumberOfLevels(); ++level)
        {
            level_to_cell_offset.push_back(number_of_cells);
            number_of_cells += partition.GetNumberOfCells(level);
        }
        level_to_cell_offset.push_back(number_of_cells);
        cells.resize(number_of_cells);

        std::vector<std::pair<CellID, NodeID>> level_source_boundary;
        std::vector<std::pair<CellID, NodeID>> level_destination_boundary;

        std::size_t number_of_unconneced = 0;

        for (LevelID level = 1u; level < partition.GetNumberOfLevels(); ++level)
        {
            auto level_offset = level_to_cell_offset[LevelIDToIndex(level)];
            level_to_boundary_index_offset.push_back(source_boundary_index.size());
            source_boundary_index.resize(source_boundary_index.size() + base_graph.GetNumberOfNodes(),
                                         INVALID_BOUNDARY_INDEX);
            destination_boundary_index.resize(
                destination_boundary_index.size() + base_graph.GetNumberOfNodes(),
                INVALID_BOUNDARY_INDEX);
            const auto level_boundary_index_offset =
                level_to_boundary_index_offset.back();

            level_source_boundary.clear();
            level_destination_boundary.clear();

            for (auto node = 0u; node < base_graph.GetNumberOfNodes(); ++node)
            {
                const CellID cell_id = partition.GetCell(level, node);
                bool is_source_node = false;
                bool is_destination_node = false;
                bool is_boundary_node = false;

                for (auto edge : base_graph.GetAdjacentEdgeRange(node))
                {
                    auto other = base_graph.GetTarget(edge);
                    const auto &data = base_graph.GetEdgeData(edge);

                    is_boundary_node |= partition.GetCell(level, other) != cell_id;
                    is_source_node |= partition.GetCell(level, other) == cell_id && data.forward;
                    is_destination_node |=
                        partition.GetCell(level, other) == cell_id && data.backward;
                }

                if (is_boundary_node)
                {
                    if (is_source_node)
                        level_source_boundary.emplace_back(cell_id, node);
                    if (is_destination_node)
                        level_destination_boundary.emplace_back(cell_id, node);

                    // if a node is unconnected we still need to keep it for correctness
                    // this adds it to the destination array to form an "empty" column
                    if (!is_source_node && !is_destination_node)
                    {
                        number_of_unconneced++;
                        util::Log(logWARNING) << "Found unconnected boundary node " << node << "("
                                              << cell_id << ") on level " << (int)level;
                        level_destination_boundary.emplace_back(cell_id, node);
                    }
                }
            }

            tbb::parallel_sort(level_source_boundary.begin(), level_source_boundary.end());
            tbb::parallel_sort(level_destination_boundary.begin(),
                               level_destination_boundary.end());

            const auto insert_cell_boundary = [this, level_offset](auto &boundary,
                                                                   auto &boundary_index,
                                                                   const auto level_boundary_index_offset,
                                                                   auto set_num_nodes_fn,
                                                                   auto set_boundary_offset_fn,
                                                                   auto begin,
                                                                   auto end)
            {
                BOOST_ASSERT(std::distance(begin, end) > 0);

                const auto cell_id = begin->first;
                BOOST_ASSERT(level_offset + cell_id < cells.size());
                auto &cell = cells[level_offset + cell_id];
                set_num_nodes_fn(cell, std::distance(begin, end));
                set_boundary_offset_fn(cell, boundary.size());

                const auto boundary_start = boundary.size();
                auto local_offset = BoundaryOffset{0};
                for (auto iter = begin; iter != end; ++iter, ++local_offset)
                {
                    boundary.push_back(iter->second);
                    boundary_index[level_boundary_index_offset + iter->second] =
                        boundary_start + local_offset;
                }
            };

            util::for_each_range(
                level_source_boundary.begin(),
                level_source_boundary.end(),
                [this, insert_cell_boundary, level_boundary_index_offset](auto begin, auto end)
                {
                    insert_cell_boundary(
                        source_boundary,
                        source_boundary_index,
                        level_boundary_index_offset,
                        [](auto &cell, auto value) { cell.num_source_nodes = value; },
                        [](auto &cell, auto value) { cell.source_boundary_offset = value; },
                        begin,
                        end);
                });
            util::for_each_range(
                level_destination_boundary.begin(),
                level_destination_boundary.end(),
                [this, insert_cell_boundary, level_boundary_index_offset](auto begin, auto end)
                {
                    insert_cell_boundary(
                        destination_boundary,
                        destination_boundary_index,
                        level_boundary_index_offset,
                        [](auto &cell, auto value) { cell.num_destination_nodes = value; },
                        [](auto &cell, auto value) { cell.destination_boundary_offset = value; },
                        begin,
                        end);
                });
        }
        level_to_boundary_index_offset.push_back(source_boundary_index.size());

        // a partition that contains boundary nodes that have no arcs going into
        // the cells or coming out of it is bad. These nodes should be reassigned
        // to a different cell.
        if (number_of_unconneced > 0)
        {
            util::Log(logWARNING) << "Node needs to either have incoming or outgoing edges in cell."
                                  << " Number of unconnected nodes is " << number_of_unconneced;
        }

        // Set cell values offsets and calculate total storage size
        ValueOffset value_offset = 0;
        for (auto &cell : cells)
        {
            cell.value_offset = value_offset;
            value_offset += cell.num_source_nodes * cell.num_destination_nodes;
        }
    }

    // Returns a new metric that can be used with this container
    customizer::CellMetric MakeMetric() const
    {
        customizer::CellMetric metric;

        if (cells.empty())
        {
            return metric;
        }

        const auto &last_cell = cells.back();
        ValueOffset total_size =
            last_cell.value_offset + last_cell.num_source_nodes * last_cell.num_destination_nodes;

        metric.weights.resize(total_size + 1, INVALID_EDGE_WEIGHT);
        metric.durations.resize(total_size + 1, MAXIMAL_EDGE_DURATION);
        metric.distances.resize(total_size + 1, INVALID_EDGE_DISTANCE);

        return metric;
    }

    ValueOffset GetTotalValueCount() const
    {
        if (cells.empty())
        {
            return 0;
        }

        const auto &last_cell = cells.back();
        return last_cell.value_offset +
               static_cast<ValueOffset>(last_cell.num_source_nodes) * last_cell.num_destination_nodes;
    }

    const CellData &GetCellData(LevelID level, CellID id) const
    {
        const auto level_index = LevelIDToIndex(level);
        BOOST_ASSERT(level_index < level_to_cell_offset.size());
        const auto offset = level_to_cell_offset[level_index];
        const auto cell_index = offset + id;
        BOOST_ASSERT(cell_index < cells.size());
        return cells[cell_index];
    }

    const NodeID *GetSourceBoundaryData() const
    {
        return source_boundary.empty() ? nullptr : source_boundary.data();
    }

    const NodeID *GetDestinationBoundaryData() const
    {
        return destination_boundary.empty() ? nullptr : destination_boundary.data();
    }

    const BoundaryIndex *GetSourceBoundaryIndexData(LevelID level) const
    {
        if (source_boundary_index.empty())
        {
            return nullptr;
        }

        const auto level_index = LevelIDToIndex(level);
        BOOST_ASSERT(level_index + 1 < level_to_boundary_index_offset.size());
        return source_boundary_index.data() + level_to_boundary_index_offset[level_index];
    }

    const BoundaryIndex *GetDestinationBoundaryIndexData(LevelID level) const
    {
        if (destination_boundary_index.empty())
        {
            return nullptr;
        }

        const auto level_index = LevelIDToIndex(level);
        BOOST_ASSERT(level_index + 1 < level_to_boundary_index_offset.size());
        return destination_boundary_index.data() + level_to_boundary_index_offset[level_index];
    }

    BoundaryOffset GetBoundaryIndexCount(LevelID level) const
    {
        if (level_to_boundary_index_offset.empty())
        {
            return 0;
        }

        const auto level_index = LevelIDToIndex(level);
        BOOST_ASSERT(level_index + 1 < level_to_boundary_index_offset.size());
        return level_to_boundary_index_offset[level_index + 1] -
               level_to_boundary_index_offset[level_index];
    }

    template <typename = std::enable_if<Ownership == storage::Ownership::View>>
    CellStorageImpl(Vector<NodeID> source_boundary_,
                    Vector<NodeID> destination_boundary_,
                    Vector<BoundaryIndex> source_boundary_index_,
                    Vector<BoundaryIndex> destination_boundary_index_,
                    Vector<CellData> cells_,
                    Vector<std::uint64_t> level_to_cell_offset_,
                    Vector<std::uint64_t> level_to_boundary_index_offset_)
        : source_boundary(std::move(source_boundary_)),
          destination_boundary(std::move(destination_boundary_)),
          source_boundary_index(std::move(source_boundary_index_)),
          destination_boundary_index(std::move(destination_boundary_index_)),
          cells(std::move(cells_)),
          level_to_cell_offset(std::move(level_to_cell_offset_)),
          level_to_boundary_index_offset(std::move(level_to_boundary_index_offset_))
    {
    }

    ConstCell GetCell(const customizer::detail::CellMetricImpl<Ownership> &metric,
                      LevelID level,
                      CellID id) const
    {
        const auto level_index = LevelIDToIndex(level);
        BOOST_ASSERT(level_index < level_to_cell_offset.size());
        const auto offset = level_to_cell_offset[level_index];
        const auto cell_index = offset + id;
        BOOST_ASSERT(cell_index < cells.size());
        const auto boundary_index_size = GetBoundaryIndexCount(level);
        return ConstCell{cells[cell_index],
                         metric.weights.data(),
                         metric.durations.data(),
                         metric.distances.data(),
                         source_boundary.empty() ? nullptr : source_boundary.data(),
                         destination_boundary.empty() ? nullptr : destination_boundary.data(),
                         GetSourceBoundaryIndexData(level),
                         GetDestinationBoundaryIndexData(level),
                         boundary_index_size};
    }

    ConstCell GetUnfilledCell(LevelID level, CellID id) const
    {
        const auto level_index = LevelIDToIndex(level);
        BOOST_ASSERT(level_index < level_to_cell_offset.size());
        const auto offset = level_to_cell_offset[level_index];
        const auto cell_index = offset + id;
        BOOST_ASSERT(cell_index < cells.size());
        const auto boundary_index_size = GetBoundaryIndexCount(level);
        return ConstCell{cells[cell_index],
                         source_boundary.empty() ? nullptr : source_boundary.data(),
                         destination_boundary.empty() ? nullptr : destination_boundary.data(),
                         GetSourceBoundaryIndexData(level),
                         GetDestinationBoundaryIndexData(level),
                         boundary_index_size};
    }

    template <typename = std::enable_if<Ownership == storage::Ownership::Container>>
    Cell GetCell(customizer::CellMetric &metric, LevelID level, CellID id) const
    {
        const auto level_index = LevelIDToIndex(level);
        BOOST_ASSERT(level_index < level_to_cell_offset.size());
        const auto offset = level_to_cell_offset[level_index];
        const auto cell_index = offset + id;
        BOOST_ASSERT(cell_index < cells.size());
        const auto boundary_index_size = GetBoundaryIndexCount(level);
        return Cell{cells[cell_index],
                    metric.weights.data(),
                    metric.durations.data(),
                    metric.distances.data(),
                    source_boundary.data(),
                    destination_boundary.data(),
                    GetSourceBoundaryIndexData(level),
                    GetDestinationBoundaryIndexData(level),
                    boundary_index_size};
    }

    friend void serialization::read<Ownership>(storage::tar::FileReader &reader,
                                               const std::string &name,
                                               detail::CellStorageImpl<Ownership> &storage);
    friend void serialization::write<Ownership>(storage::tar::FileWriter &writer,
                                                const std::string &name,
                                                const detail::CellStorageImpl<Ownership> &storage);

  private:
    Vector<NodeID> source_boundary;
    Vector<NodeID> destination_boundary;
    Vector<BoundaryIndex> source_boundary_index;
    Vector<BoundaryIndex> destination_boundary_index;
    Vector<CellData> cells;
    Vector<std::uint64_t> level_to_cell_offset;
    Vector<std::uint64_t> level_to_boundary_index_offset;
};
} // namespace detail
} // namespace osrm::partitioner

#endif // OSRM_PARTITIONER_CUSTOMIZE_CELL_STORAGE_HPP
