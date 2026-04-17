#ifndef OSRM_CUSTOMIZER_TEMPORAL_CELL_METRIC_HPP
#define OSRM_CUSTOMIZER_TEMPORAL_CELL_METRIC_HPP

#include "customizer/temporal_profiles.hpp"
#include "partitioner/cell_storage.hpp"
#include "storage/io_fwd.hpp"
#include "storage/shared_memory_ownership.hpp"
#include "util/typedefs.hpp"
#include "util/vector_view.hpp"

#include <boost/iterator/iterator_facade.hpp>

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <optional>
#include <ranges>
#include <utility>

namespace osrm::customizer
{
namespace detail
{
template <storage::Ownership Ownership> struct TemporalCellMetricImpl
{
    template <typename T> using Vector = util::ViewOrVector<T, Ownership>;

    Vector<TemporalFunctionID> function_ids;
    Vector<EdgeDuration> min_durations;

    template <typename FunctionIdValueT, typename DurationValueT> class CellImpl
    {
      private:
        using FunctionIdPtrT = FunctionIdValueT *;
        using DurationPtrT = DurationValueT *;
        using FunctionIdRowIterator = FunctionIdPtrT;
        using BoundaryIndex = partitioner::CellStorageView::BoundaryIndex;
        using BoundaryOffset = partitioner::CellStorageView::BoundaryOffset;

        template <typename ValuePtrT>
        class ColumnIterator
            : public boost::iterator_facade<ColumnIterator<ValuePtrT>,
                                            decltype(*std::declval<ValuePtrT>()),
                                            boost::random_access_traversal_tag>
        {
            using ValueT = decltype(*std::declval<ValuePtrT>());
            using base_t = boost::iterator_facade<ColumnIterator<ValueT>,
                                                  ValueT,
                                                  boost::random_access_traversal_tag>;

          public:
            using value_type = typename base_t::value_type;
            using difference_type = typename base_t::difference_type;
            using reference = typename base_t::reference;
            using iterator_category = std::random_access_iterator_tag;

            explicit ColumnIterator() : current(nullptr), stride(1) {}

            explicit ColumnIterator(ValuePtrT begin, std::size_t row_length)
                : current(begin), stride(row_length)
            {
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

            friend class boost::iterator_core_access;
            ValuePtrT current;
            std::size_t stride;
        };

        BoundaryIndex GetSourceBoundaryPosition(const NodeID node) const
        {
            if (source_boundary_index == nullptr ||
                static_cast<BoundaryOffset>(node) >= boundary_index_size)
            {
                return partitioner::CellStorageView::INVALID_BOUNDARY_INDEX;
            }

            const auto boundary_index = source_boundary_index[node];
            if (boundary_index == partitioner::CellStorageView::INVALID_BOUNDARY_INDEX ||
                boundary_index < source_boundary_offset ||
                boundary_index >= source_boundary_offset + num_source_nodes)
            {
                return partitioner::CellStorageView::INVALID_BOUNDARY_INDEX;
            }

            return boundary_index;
        }

        BoundaryIndex GetDestinationBoundaryPosition(const NodeID node) const
        {
            if (destination_boundary_index == nullptr ||
                static_cast<BoundaryOffset>(node) >= boundary_index_size)
            {
                return partitioner::CellStorageView::INVALID_BOUNDARY_INDEX;
            }

            const auto boundary_index = destination_boundary_index[node];
            if (boundary_index == partitioner::CellStorageView::INVALID_BOUNDARY_INDEX ||
                boundary_index < destination_boundary_offset ||
                boundary_index >= destination_boundary_offset + num_destination_nodes)
            {
                return partitioner::CellStorageView::INVALID_BOUNDARY_INDEX;
            }

            return boundary_index;
        }

        template <typename ValuePtrT>
        auto GetOutRange(const ValuePtrT ptr, const NodeID node) const
        {
            const auto boundary_index = GetSourceBoundaryPosition(node);
            if (boundary_index == partitioner::CellStorageView::INVALID_BOUNDARY_INDEX)
            {
                return std::ranges::subrange(ptr, ptr);
            }

            const auto row = boundary_index - source_boundary_offset;
            auto begin = ptr + num_destination_nodes * row;
            auto end = begin + num_destination_nodes;
            return std::ranges::subrange(begin, end);
        }

        template <typename ValuePtrT>
        auto GetInRange(const ValuePtrT ptr, const NodeID node) const
        {
            const auto boundary_index = GetDestinationBoundaryPosition(node);
            if (boundary_index == partitioner::CellStorageView::INVALID_BOUNDARY_INDEX)
            {
                return std::ranges::subrange(ColumnIterator<ValuePtrT>{},
                                             ColumnIterator<ValuePtrT>{});
            }

            const auto column = boundary_index - destination_boundary_offset;
            auto begin = ColumnIterator<ValuePtrT>{ptr + column, num_destination_nodes};
            auto end = ColumnIterator<ValuePtrT>{
                ptr + column + num_source_nodes * num_destination_nodes, num_destination_nodes};
            return std::ranges::subrange(begin, end);
        }

        std::optional<std::size_t> GetMatrixIndex(const NodeID from, const NodeID to) const
        {
            const auto source_position = GetSourceBoundaryPosition(from);
            const auto destination_position = GetDestinationBoundaryPosition(to);
            if (source_position == partitioner::CellStorageView::INVALID_BOUNDARY_INDEX ||
                destination_position == partitioner::CellStorageView::INVALID_BOUNDARY_INDEX)
            {
                return std::nullopt;
            }

            const auto row = source_position - source_boundary_offset;
            const auto column = destination_position - destination_boundary_offset;
            return static_cast<std::size_t>(row) * num_destination_nodes + column;
        }

      public:
        template <typename CellDataT>
        CellImpl(const CellDataT &data,
                 FunctionIdPtrT all_function_ids,
                 DurationPtrT all_min_durations,
                 const NodeID *all_sources,
                 const NodeID *all_destinations,
                 const BoundaryIndex *all_source_boundary_index,
                 const BoundaryIndex *all_destination_boundary_index,
                 const BoundaryOffset boundary_index_size_)
            : num_source_nodes(data.num_source_nodes),
              num_destination_nodes(data.num_destination_nodes),
              function_ids(all_function_ids + data.value_offset),
              min_durations(all_min_durations + data.value_offset),
              source_boundary(all_sources + data.source_boundary_offset),
              destination_boundary(all_destinations + data.destination_boundary_offset),
              source_boundary_index(all_source_boundary_index),
              destination_boundary_index(all_destination_boundary_index),
              source_boundary_offset(data.source_boundary_offset),
              destination_boundary_offset(data.destination_boundary_offset),
              boundary_index_size(boundary_index_size_)
        {
        }

        auto GetOutFunctionID(const NodeID node) const { return GetOutRange(function_ids, node); }

        auto GetInFunctionID(const NodeID node) const { return GetInRange(function_ids, node); }

        auto GetOutMinDuration(const NodeID node) const { return GetOutRange(min_durations, node); }

        auto GetInMinDuration(const NodeID node) const { return GetInRange(min_durations, node); }

        TemporalFunctionID GetFunctionID(const NodeID from, const NodeID to) const
        {
            const auto matrix_index = GetMatrixIndex(from, to);
            return matrix_index ? function_ids[*matrix_index] : INVALID_TEMPORAL_FUNCTION_ID;
        }

        EdgeDuration GetMinDuration(const NodeID from, const NodeID to) const
        {
            const auto matrix_index = GetMatrixIndex(from, to);
            return matrix_index ? min_durations[*matrix_index] : INVALID_EDGE_DURATION;
        }

        auto GetSourceNodes() const
        {
            return std::ranges::subrange(source_boundary, source_boundary + num_source_nodes);
        }

        auto GetDestinationNodes() const
        {
            return std::ranges::subrange(destination_boundary,
                                         destination_boundary + num_destination_nodes);
        }

      private:
        partitioner::CellStorageView::BoundarySize num_source_nodes;
        partitioner::CellStorageView::BoundarySize num_destination_nodes;
        FunctionIdRowIterator function_ids;
        DurationPtrT min_durations;
        const NodeID *source_boundary;
        const NodeID *destination_boundary;
        const BoundaryIndex *source_boundary_index;
        const BoundaryIndex *destination_boundary_index;
        BoundaryOffset source_boundary_offset;
        BoundaryOffset destination_boundary_offset;
        BoundaryOffset boundary_index_size;
    };

    using Cell = CellImpl<TemporalFunctionID, EdgeDuration>;
    using ConstCell = CellImpl<const TemporalFunctionID, const EdgeDuration>;

    ConstCell GetCell(const partitioner::detail::CellStorageImpl<Ownership> &cells,
                      const LevelID level,
                      const CellID id) const
    {
        const auto &cell_data = cells.GetCellData(level, id);
        return ConstCell{cell_data,
                         function_ids.data(),
                         min_durations.data(),
                         cells.GetSourceBoundaryData(),
                         cells.GetDestinationBoundaryData(),
                         cells.GetSourceBoundaryIndexData(level),
                         cells.GetDestinationBoundaryIndexData(level),
                         cells.GetBoundaryIndexCount(level)};
    }
};
} // namespace detail

using TemporalCellMetric = detail::TemporalCellMetricImpl<storage::Ownership::Container>;
using TemporalCellMetricView = detail::TemporalCellMetricImpl<storage::Ownership::View>;

inline TemporalCellMetric MakeTemporalCellMetric(const partitioner::CellStorage &cells)
{
    TemporalCellMetric metric;
    const auto total_size = cells.GetTotalValueCount();
    metric.function_ids.resize(total_size + 1, INVALID_TEMPORAL_FUNCTION_ID);
    metric.min_durations.resize(total_size + 1, INVALID_EDGE_DURATION);
    return metric;
}
} // namespace osrm::customizer

#endif
