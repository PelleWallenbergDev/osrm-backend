#ifndef OSRM_EXTRACTOR_WAY_NODE_STORAGE_HPP
#define OSRM_EXTRACTOR_WAY_NODE_STORAGE_HPP

#include "util/typedefs.hpp"

#include <cstddef>
#include <cstdint>
#include <ranges>
#include <vector>

namespace osrm::extractor
{

struct WayNodeStorage
{
    using WayIDVector = std::vector<OSMWayID>;
    using NodeVector = std::vector<OSMNodeID>;
    using NodeOffsetVector = std::vector<std::uint64_t>;

    WayNodeStorage() : node_offsets{0} {}

    auto size() const { return way_ids.size(); }

    auto GetNodes(std::size_t index) const
    {
        const auto begin = node_ids.begin() + static_cast<std::ptrdiff_t>(node_offsets[index]);
        const auto end = node_ids.begin() + static_cast<std::ptrdiff_t>(node_offsets[index + 1]);
        return std::ranges::subrange(begin, end);
    }

    WayIDVector way_ids;
    NodeOffsetVector node_offsets;
    NodeVector node_ids;
};

} // namespace osrm::extractor

#endif
