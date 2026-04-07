#ifndef OSRM_UPDATER_TEMPORAL_SOURCE_HPP
#define OSRM_UPDATER_TEMPORAL_SOURCE_HPP

#include "updater/source.hpp"

#include <cstdint>

namespace osrm::updater
{

inline constexpr std::uint8_t TEMPORAL_DIRECTION_FORWARD = 0;
inline constexpr std::uint8_t TEMPORAL_DIRECTION_REVERSE = 1;

struct TemporalBucketKey final
{
    std::uint64_t from, to;
    std::uint8_t direction;
    std::uint32_t week_bucket;

    TemporalBucketKey() : from(0), to(0), direction(TEMPORAL_DIRECTION_FORWARD), week_bucket(0) {}
    TemporalBucketKey(const std::uint64_t from_,
                      const std::uint64_t to_,
                      const std::uint8_t direction_,
                      const std::uint32_t week_bucket_)
        : from(from_), to(to_), direction(direction_), week_bucket(week_bucket_)
    {
    }

    bool operator<(const TemporalBucketKey &rhs) const
    {
        return std::tie(from, to, direction, week_bucket) <
               std::tie(rhs.from, rhs.to, rhs.direction, rhs.week_bucket);
    }

    bool operator==(const TemporalBucketKey &rhs) const
    {
        return std::tie(from, to, direction, week_bucket) ==
               std::tie(rhs.from, rhs.to, rhs.direction, rhs.week_bucket);
    }
};

struct TemporalBucketValue final
{
    TemporalBucketValue() : duration_ds(0), source(0) {}
    explicit TemporalBucketValue(const std::int32_t duration_ds_) : duration_ds(duration_ds_), source(0) {}

    std::int32_t duration_ds;
    std::uint8_t source;
};

using TemporalLookupTable = LookupTable<TemporalBucketKey, TemporalBucketValue>;

TemporalLookupTable readTemporalValues(const std::vector<std::string> &paths);

} // namespace osrm::updater

#endif
