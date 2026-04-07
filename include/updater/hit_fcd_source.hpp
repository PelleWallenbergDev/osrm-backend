#ifndef OSRM_UPDATER_HIT_FCD_SOURCE_HPP
#define OSRM_UPDATER_HIT_FCD_SOURCE_HPP

#include "extractor/link_map.hpp"
#include "updater/source.hpp"
#include "updater/temporal_source.hpp"

#include "util/coordinate.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace osrm::updater
{

inline constexpr std::uint8_t HIT_FCD_DIRECTION_FORWARD = 0;
inline constexpr std::uint8_t HIT_FCD_DIRECTION_REVERSE = 1;

struct HitFCDKey final
{
    std::uint64_t way_id = 0;
    std::uint8_t direction = HIT_FCD_DIRECTION_FORWARD;
    std::uint32_t week_bucket = 0;

    bool operator<(const HitFCDKey &rhs) const
    {
        return std::tie(way_id, direction, week_bucket) <
               std::tie(rhs.way_id, rhs.direction, rhs.week_bucket);
    }

    bool operator==(const HitFCDKey &rhs) const
    {
        return std::tie(way_id, direction, week_bucket) ==
               std::tie(rhs.way_id, rhs.direction, rhs.week_bucket);
    }
};

struct HitFCDValue final
{
    double speed_kmh = 0.;
    std::uint64_t sample_count = 0;
    std::uint8_t source = 0;
};

using HitFCDLookupTable = LookupTable<HitFCDKey, HitFCDValue>;

HitFCDLookupTable readHitFCDValues(const std::vector<std::string> &paths,
                                   std::uint32_t bucket_size_minutes,
                                   std::uint32_t forward_direction_value,
                                   std::uint32_t reverse_direction_value);

TemporalLookupTable buildTemporalLookupFromHitFCD(
    const HitFCDLookupTable &hit_fcd_lookup,
    const extractor::LinkMapStorage &link_map,
    const extractor::SegmentDataContainer &segment_data,
    const std::vector<util::Coordinate> &coordinates,
    const extractor::PackedOSMIDs &osm_node_ids);

} // namespace osrm::updater

#endif
