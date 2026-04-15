#ifndef OSRM_CUSTOMIZER_TEMPORAL_PROFILES_HPP
#define OSRM_CUSTOMIZER_TEMPORAL_PROFILES_HPP

#include "engine/temporal/temporal_profile_decoder.hpp"
#include "util/typedefs.hpp"
#include "util/vector_view.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>

namespace osrm::customizer
{

using TemporalProfileID = std::uint32_t;
using TemporalProfileBucketValue = EdgeDuration::value_type;
using TemporalProfileDurationValue = EdgeDuration::value_type;
using TemporalProfileCoeffValue = engine::temporal::TemporalProfileCoefficient;
using TemporalFunctionID = TemporalProfileID;
using TemporalFunctionBucketValue = TemporalProfileBucketValue;
using TemporalFunctionDurationValue = TemporalProfileDurationValue;
using TemporalFunctionCoeffValue = engine::temporal::TemporalFunctionCoefficient;
using TemporalFunctionFlags = std::uint8_t;

inline constexpr TemporalProfileID INVALID_TEMPORAL_PROFILE_ID =
    std::numeric_limits<TemporalProfileID>::max();
inline constexpr TemporalFunctionID INVALID_TEMPORAL_FUNCTION_ID = INVALID_TEMPORAL_PROFILE_ID;
inline constexpr std::uint32_t TEMPORAL_PROFILE_ENCODING_VERSION_DENSE = 1;
inline constexpr std::uint32_t TEMPORAL_PROFILE_ENCODING_VERSION_DCT = 2;
inline constexpr std::uint32_t TEMPORAL_FUNCTION_ENCODING_VERSION_DENSE =
    TEMPORAL_PROFILE_ENCODING_VERSION_DENSE;
inline constexpr std::uint32_t TEMPORAL_FUNCTION_ENCODING_VERSION_DCT =
    TEMPORAL_PROFILE_ENCODING_VERSION_DCT;
inline constexpr TemporalFunctionFlags TEMPORAL_FUNCTION_FLAG_NONE = 0;
inline constexpr TemporalFunctionFlags TEMPORAL_FUNCTION_FLAG_FIFO_VALID = 1U << 0;
inline constexpr TemporalFunctionFlags TEMPORAL_FUNCTION_FLAG_USED_FALLBACK_COEFF_COUNT = 1U << 1;

struct TemporalProfileMeta
{
    std::uint32_t bucket_size_minutes = 0;
    std::uint32_t week_bucket_count = 0;
    std::uint32_t encoding_version = TEMPORAL_PROFILE_ENCODING_VERSION_DENSE;
};
using TemporalFunctionMeta = TemporalProfileMeta;

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
using TemporalFunctionIndex = TemporalProfileIndex;

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
using TemporalFunctionIndexView = TemporalProfileIndexView;

struct TemporalProfileStorage
{
    TemporalProfileMeta meta;
    std::vector<std::uint64_t> profile_offsets;
    std::vector<std::uint32_t> profile_sizes;
    std::vector<TemporalProfileDurationValue> min_durations;
    std::vector<TemporalProfileDurationValue> freeflow_durations;
    std::vector<TemporalFunctionFlags> profile_flags;
    std::vector<TemporalProfileBucketValue> values;
    std::vector<TemporalProfileCoeffValue> coeffs;

    std::uint32_t GetBucketSizeMinutes() const { return meta.bucket_size_minutes; }
    std::uint32_t GetWeekBucketCount() const { return meta.week_bucket_count; }
    std::uint32_t GetEncodingVersion() const { return meta.encoding_version; }

    bool HasProfile(const TemporalProfileID id) const { return id < profile_offsets.size(); }

    EdgeDuration GetMinDuration(const TemporalProfileID id) const;
    EdgeDuration GetFreeFlowDuration(const TemporalProfileID id) const;
    TemporalFunctionFlags GetFlags(const TemporalProfileID id) const;
    bool IsFIFOValid(const TemporalProfileID id) const;
    bool UsedFallbackCoeffCount(const TemporalProfileID id) const;
    EdgeDuration GetDuration(const TemporalProfileID id, const std::uint32_t week_bucket) const;
};
using TemporalFunctionStorage = TemporalProfileStorage;

struct TemporalProfileStorageView
{
    util::vector_view<std::uint64_t> profile_offsets;
    util::vector_view<std::uint32_t> profile_sizes;
    std::uint32_t *bucket_size_minutes = nullptr;
    std::uint32_t *week_bucket_count = nullptr;
    std::uint32_t *encoding_version = nullptr;
    util::vector_view<TemporalProfileDurationValue> min_durations;
    util::vector_view<TemporalProfileDurationValue> freeflow_durations;
    util::vector_view<TemporalFunctionFlags> profile_flags;
    util::vector_view<TemporalProfileBucketValue> values;
    util::vector_view<TemporalProfileCoeffValue> coeffs;

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

    EdgeDuration GetMinDuration(const TemporalProfileID id) const;
    EdgeDuration GetFreeFlowDuration(const TemporalProfileID id) const;
    TemporalFunctionFlags GetFlags(const TemporalProfileID id) const;
    bool IsFIFOValid(const TemporalProfileID id) const;
    bool UsedFallbackCoeffCount(const TemporalProfileID id) const;
    EdgeDuration GetDuration(const TemporalProfileID id, const std::uint32_t week_bucket) const;
};
using TemporalFunctionStorageView = TemporalProfileStorageView;

namespace detail
{
template <typename StorageT>
inline EdgeDuration GetTemporalProfileMinDuration(const StorageT &storage, const TemporalProfileID id)
{
    if (!storage.HasProfile(id))
    {
        return INVALID_EDGE_DURATION;
    }

    if (id < storage.min_durations.size())
    {
        return EdgeDuration{storage.min_durations[id]};
    }

    if (storage.GetEncodingVersion() != TEMPORAL_PROFILE_ENCODING_VERSION_DENSE ||
        storage.values.empty() || id >= storage.profile_sizes.size())
    {
        return INVALID_EDGE_DURATION;
    }

    const auto offset = static_cast<std::size_t>(storage.profile_offsets[id]);
    const auto size = static_cast<std::size_t>(storage.profile_sizes[id]);
    if (size == 0 || offset + size > storage.values.size())
    {
        return INVALID_EDGE_DURATION;
    }

    const auto min_value =
        *std::min_element(storage.values.begin() + offset, storage.values.begin() + offset + size);
    return EdgeDuration{min_value};
}

template <typename StorageT>
inline EdgeDuration
GetTemporalProfileDuration(const StorageT &storage,
                          const TemporalProfileID id,
                          const std::uint32_t week_bucket)
{
    if (!storage.HasProfile(id) || week_bucket >= storage.GetWeekBucketCount() ||
        id >= storage.profile_sizes.size())
    {
        return INVALID_EDGE_DURATION;
    }

    const auto offset = static_cast<std::size_t>(storage.profile_offsets[id]);
    const auto size = static_cast<std::size_t>(storage.profile_sizes[id]);

    switch (storage.GetEncodingVersion())
    {
    case TEMPORAL_PROFILE_ENCODING_VERSION_DENSE:
        if (week_bucket >= size || storage.values.empty() || offset + size > storage.values.size())
        {
            return INVALID_EDGE_DURATION;
        }
        return EdgeDuration{storage.values[offset + week_bucket]};
    case TEMPORAL_PROFILE_ENCODING_VERSION_DCT:
        if (size == 0 || storage.coeffs.empty() || offset + size > storage.coeffs.size())
        {
            return INVALID_EDGE_DURATION;
        }
        return engine::temporal::DecodeTemporalProfileBucket(storage.GetWeekBucketCount(),
                                                             storage.coeffs.data() + offset,
                                                             static_cast<std::uint32_t>(size),
                                                             week_bucket,
                                                             GetTemporalProfileMinDuration(
                                                                 storage, id));
    default:
        return INVALID_EDGE_DURATION;
    }
}

template <typename StorageT>
inline EdgeDuration GetTemporalProfileFreeFlowDuration(const StorageT &storage,
                                                       const TemporalProfileID id)
{
    if (!storage.HasProfile(id))
    {
        return INVALID_EDGE_DURATION;
    }

    if (id < storage.freeflow_durations.size())
    {
        return EdgeDuration{storage.freeflow_durations[id]};
    }

    return GetTemporalProfileMinDuration(storage, id);
}

template <typename StorageT>
inline TemporalFunctionFlags GetTemporalProfileFlags(const StorageT &storage,
                                                     const TemporalProfileID id)
{
    if (!storage.HasProfile(id) || id >= storage.profile_flags.size())
    {
        return TEMPORAL_FUNCTION_FLAG_NONE;
    }

    return storage.profile_flags[id];
}
} // namespace detail

inline EdgeDuration TemporalProfileStorage::GetMinDuration(const TemporalProfileID id) const
{
    return detail::GetTemporalProfileMinDuration(*this, id);
}

inline EdgeDuration TemporalProfileStorage::GetFreeFlowDuration(const TemporalProfileID id) const
{
    return detail::GetTemporalProfileFreeFlowDuration(*this, id);
}

inline TemporalFunctionFlags TemporalProfileStorage::GetFlags(const TemporalProfileID id) const
{
    return detail::GetTemporalProfileFlags(*this, id);
}

inline bool TemporalProfileStorage::IsFIFOValid(const TemporalProfileID id) const
{
    return (GetFlags(id) & TEMPORAL_FUNCTION_FLAG_FIFO_VALID) != 0;
}

inline bool TemporalProfileStorage::UsedFallbackCoeffCount(const TemporalProfileID id) const
{
    return (GetFlags(id) & TEMPORAL_FUNCTION_FLAG_USED_FALLBACK_COEFF_COUNT) != 0;
}

inline EdgeDuration TemporalProfileStorage::GetDuration(const TemporalProfileID id,
                                                        const std::uint32_t week_bucket) const
{
    return detail::GetTemporalProfileDuration(*this, id, week_bucket);
}

inline EdgeDuration TemporalProfileStorageView::GetMinDuration(const TemporalProfileID id) const
{
    return detail::GetTemporalProfileMinDuration(*this, id);
}

inline EdgeDuration TemporalProfileStorageView::GetFreeFlowDuration(const TemporalProfileID id) const
{
    return detail::GetTemporalProfileFreeFlowDuration(*this, id);
}

inline TemporalFunctionFlags TemporalProfileStorageView::GetFlags(const TemporalProfileID id) const
{
    return detail::GetTemporalProfileFlags(*this, id);
}

inline bool TemporalProfileStorageView::IsFIFOValid(const TemporalProfileID id) const
{
    return (GetFlags(id) & TEMPORAL_FUNCTION_FLAG_FIFO_VALID) != 0;
}

inline bool TemporalProfileStorageView::UsedFallbackCoeffCount(const TemporalProfileID id) const
{
    return (GetFlags(id) & TEMPORAL_FUNCTION_FLAG_USED_FALLBACK_COEFF_COUNT) != 0;
}

inline EdgeDuration TemporalProfileStorageView::GetDuration(const TemporalProfileID id,
                                                            const std::uint32_t week_bucket) const
{
    return detail::GetTemporalProfileDuration(*this, id, week_bucket);
}

} // namespace osrm::customizer

#endif
