#ifndef OSRM_EXTRACTOR_LINK_MAP_HPP
#define OSRM_EXTRACTOR_LINK_MAP_HPP

#include "extractor/packed_osm_ids.hpp"
#include "extractor/segment_data_container.hpp"
#include "extractor/way_node_storage.hpp"

#include "util/typedefs.hpp"

#include <algorithm>
#include <cstdint>
#include <tuple>
#include <utility>
#include <vector>

namespace osrm::extractor
{

inline constexpr std::uint8_t LINK_MAP_DIRECTION_FORWARD = 0;
inline constexpr std::uint8_t LINK_MAP_DIRECTION_REVERSE = 1;

struct LinkMapKey final
{
    std::uint64_t way_id = 0;
    std::uint8_t direction = LINK_MAP_DIRECTION_FORWARD;

    bool operator<(const LinkMapKey &rhs) const
    {
        return std::tie(way_id, direction) < std::tie(rhs.way_id, rhs.direction);
    }

    bool operator==(const LinkMapKey &rhs) const
    {
        return std::tie(way_id, direction) == std::tie(rhs.way_id, rhs.direction);
    }
};

struct LinkMapEntry final
{
    PackedGeometryID geometry_id = 0;
    std::uint8_t geometry_direction = LINK_MAP_DIRECTION_FORWARD;
    std::uint32_t segment_begin = 0;
    std::uint32_t segment_end = 0;
};

struct LinkMapStorage final
{
    using OffsetVector = std::vector<std::uint64_t>;

    bool empty() const { return way_ids.empty(); }

    std::pair<std::size_t, std::size_t> FindRange(const std::uint64_t way_id,
                                                  const std::uint8_t direction) const
    {
        const auto needle = std::make_pair(way_id, direction);
        const auto first =
            std::lower_bound(way_ids.begin(),
                             way_ids.end(),
                             needle,
                             [&](const auto &lhs, const auto &rhs)
                             {
                                 return std::tie(lhs.first, lhs.second) <
                                        std::tie(rhs.first, rhs.second);
                             });

        if (first == way_ids.end() || first->first != way_id || first->second != direction)
        {
            return {0, 0};
        }

        const auto index = static_cast<std::size_t>(first - way_ids.begin());
        return {static_cast<std::size_t>(offsets[index]),
                static_cast<std::size_t>(offsets[index + 1])};
    }

    std::vector<std::pair<std::uint64_t, std::uint8_t>> way_ids;
    OffsetVector offsets;
    std::vector<PackedGeometryID> geometry_ids;
    std::vector<std::uint8_t> geometry_directions;
    std::vector<std::uint32_t> segment_begins;
    std::vector<std::uint32_t> segment_ends;
};

LinkMapStorage BuildLinkMap(const WayNodeStorage &way_node_storage,
                            const SegmentDataContainer &segment_data,
                            const PackedOSMIDs &osm_node_ids);

} // namespace osrm::extractor

#endif
