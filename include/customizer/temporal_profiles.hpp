#ifndef OSRM_CUSTOMIZER_TEMPORAL_PROFILES_HPP
#define OSRM_CUSTOMIZER_TEMPORAL_PROFILES_HPP

#include "util/typedefs.hpp"
#include "util/vector_view.hpp"

#include <cstdint>
#include <limits>
#include <vector>

namespace osrm::customizer
{

using TemporalProfileID = std::uint32_t;
using TemporalProfileBucketValue = EdgeDuration::value_type;
using TemporalProfileSpeedValue = std::uint32_t;

inline constexpr TemporalProfileID INVALID_TEMPORAL_PROFILE_ID =
    std::numeric_limits<TemporalProfileID>::max();
inline constexpr std::uint32_t TEMPORAL_PROFILE_ENCODING_VERSION_DENSE = 1;

struct TemporalProfileMeta
{
    std::uint32_t bucket_size_minutes = 0;
    std::uint32_t week_bucket_count = 0;
    std::uint32_t encoding_version = TEMPORAL_PROFILE_ENCODING_VERSION_DENSE;
};

struct TemporalProfileIndex
{
    std::vector<TemporalProfileID> forward_profile_ids;
    std::vector<TemporalProfileID> reverse_profile_ids;

    bool HasForwardProfile(const PackedGeometryID id) const
    {
        return id < forward_profile_ids.size() &&
               forward_profile_ids[id] != INVALID_TEMPORAL_PROFILE_ID;
    }

    bool HasReverseProfile(const PackedGeometryID id) const
    {
        return id < reverse_profile_ids.size() &&
               reverse_profile_ids[id] != INVALID_TEMPORAL_PROFILE_ID;
    }

    TemporalProfileID GetForwardProfileID(const PackedGeometryID id) const
    {
        return HasForwardProfile(id) ? forward_profile_ids[id] : INVALID_TEMPORAL_PROFILE_ID;
    }

    TemporalProfileID GetReverseProfileID(const PackedGeometryID id) const
    {
        return HasReverseProfile(id) ? reverse_profile_ids[id] : INVALID_TEMPORAL_PROFILE_ID;
    }
};

struct TemporalProfileIndexView
{
    util::vector_view<TemporalProfileID> forward_profile_ids;
    util::vector_view<TemporalProfileID> reverse_profile_ids;

    bool HasForwardProfile(const PackedGeometryID id) const
    {
        return id < forward_profile_ids.size() &&
               forward_profile_ids[id] != INVALID_TEMPORAL_PROFILE_ID;
    }

    bool HasReverseProfile(const PackedGeometryID id) const
    {
        return id < reverse_profile_ids.size() &&
               reverse_profile_ids[id] != INVALID_TEMPORAL_PROFILE_ID;
    }

    TemporalProfileID GetForwardProfileID(const PackedGeometryID id) const
    {
        return HasForwardProfile(id) ? forward_profile_ids[id] : INVALID_TEMPORAL_PROFILE_ID;
    }

    TemporalProfileID GetReverseProfileID(const PackedGeometryID id) const
    {
        return HasReverseProfile(id) ? reverse_profile_ids[id] : INVALID_TEMPORAL_PROFILE_ID;
    }
};

struct TemporalProfileStorage
{
    TemporalProfileMeta meta;
    std::vector<std::uint64_t> profile_offsets;
    std::vector<std::uint32_t> profile_sizes;
    std::vector<TemporalProfileSpeedValue> freeflow_speeds;
    std::vector<TemporalProfileSpeedValue> constrained_speeds;
    std::vector<TemporalProfileBucketValue> values;

    std::uint32_t GetBucketSizeMinutes() const { return meta.bucket_size_minutes; }
    std::uint32_t GetWeekBucketCount() const { return meta.week_bucket_count; }
    std::uint32_t GetEncodingVersion() const { return meta.encoding_version; }

    bool HasProfile(const TemporalProfileID id) const { return id < profile_offsets.size(); }

    EdgeDuration GetDuration(const TemporalProfileID id, const std::uint32_t week_bucket) const
    {
        if (!HasProfile(id) || meta.encoding_version != TEMPORAL_PROFILE_ENCODING_VERSION_DENSE ||
            week_bucket >= meta.week_bucket_count || week_bucket >= profile_sizes[id])
        {
            return INVALID_EDGE_DURATION;
        }

        const auto bucket_index = static_cast<std::size_t>(profile_offsets[id]) + week_bucket;
        return EdgeDuration{values[bucket_index]};
    }
};

struct TemporalProfileStorageView
{
    util::vector_view<std::uint64_t> profile_offsets;
    util::vector_view<std::uint32_t> profile_sizes;
    std::uint32_t *bucket_size_minutes = nullptr;
    std::uint32_t *week_bucket_count = nullptr;
    std::uint32_t *encoding_version = nullptr;
    util::vector_view<TemporalProfileSpeedValue> freeflow_speeds;
    util::vector_view<TemporalProfileSpeedValue> constrained_speeds;
    util::vector_view<TemporalProfileBucketValue> values;

    std::uint32_t GetBucketSizeMinutes() const
    {
        return bucket_size_minutes ? *bucket_size_minutes : 0;
    }

    std::uint32_t GetWeekBucketCount() const
    {
        return week_bucket_count ? *week_bucket_count : 0;
    }

    std::uint32_t GetEncodingVersion() const
    {
        return encoding_version ? *encoding_version : 0;
    }

    bool HasProfile(const TemporalProfileID id) const { return id < profile_offsets.size(); }

    EdgeDuration GetDuration(const TemporalProfileID id, const std::uint32_t week_bucket) const
    {
        if (!HasProfile(id) || GetEncodingVersion() != TEMPORAL_PROFILE_ENCODING_VERSION_DENSE ||
            week_bucket >= GetWeekBucketCount() || week_bucket >= profile_sizes[id])
        {
            return INVALID_EDGE_DURATION;
        }

        const auto bucket_index = static_cast<std::size_t>(profile_offsets[id]) + week_bucket;
        return EdgeDuration{values[bucket_index]};
    }
};

} // namespace osrm::customizer

#endif
