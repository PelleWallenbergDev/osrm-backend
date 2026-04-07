#ifndef OSRM_ENGINE_TEMPORAL_TRAFFIC_HPP
#define OSRM_ENGINE_TEMPORAL_TRAFFIC_HPP

#include "util/typedefs.hpp"

#include <algorithm>
#include <cstdint>
#include <ctime>
#include <numeric>
#include <vector>

namespace osrm::engine::temporal
{

struct TemporalGeometryStep
{
    PackedGeometryID geometry_id = SPECIAL_GEOMETRYID;
    bool forward = true;
};

struct TemporalStepResult
{
    PackedGeometryID geometry_id = SPECIAL_GEOMETRYID;
    bool forward = true;
    std::uint32_t week_bucket = 0;
    EdgeDuration duration = EdgeDuration{0};
    bool used_temporal = false;
};

struct TemporalPathEvaluation
{
    EdgeDuration total_duration = EdgeDuration{0};
    std::time_t arrival_timestamp = 0;
    std::vector<TemporalStepResult> steps;
};

namespace detail
{
inline std::tm utcTime(const std::time_t timestamp)
{
    std::tm result{};
#ifdef _WIN32
    gmtime_s(&result, &timestamp);
#else
    gmtime_r(&timestamp, &result);
#endif
    return result;
}
} // namespace detail

inline std::uint32_t TimestampToWeekBucket(const std::time_t timestamp,
                                           const std::uint32_t bucket_size_minutes)
{
    if (bucket_size_minutes == 0)
    {
        return 0;
    }

    const auto utc = detail::utcTime(timestamp);
    const auto monday_based_day = (utc.tm_wday + 6) % 7;
    const auto minutes_since_week_start =
        monday_based_day * 24 * 60 + utc.tm_hour * 60 + utc.tm_min;
    return static_cast<std::uint32_t>(minutes_since_week_start / bucket_size_minutes);
}

template <typename FacadeT>
EdgeDuration GetStaticGeometryDuration(const FacadeT &facade,
                                       const PackedGeometryID geometry_id,
                                       const bool forward)
{
    if (forward)
    {
        const auto durations = facade.GetUncompressedForwardDurations(geometry_id);
        return alias_cast<EdgeDuration>(
            std::accumulate(durations.begin(), durations.end(), SegmentDuration{0}));
    }

    const auto durations = facade.GetUncompressedReverseDurations(geometry_id);
    return alias_cast<EdgeDuration>(
        std::accumulate(durations.begin(), durations.end(), SegmentDuration{0}));
}

template <typename FacadeT>
TemporalPathEvaluation EvaluateGeometryPath(const FacadeT &facade,
                                            const std::vector<TemporalGeometryStep> &steps,
                                            const std::time_t departure_timestamp)
{
    TemporalPathEvaluation evaluation;
    evaluation.arrival_timestamp = departure_timestamp;
    evaluation.steps.reserve(steps.size());

    const auto bucket_size_minutes = facade.GetTemporalBucketSizeMinutes();
    const auto week_bucket_count = facade.GetTemporalWeekBucketCount();

    for (const auto &step : steps)
    {
        TemporalStepResult result;
        result.geometry_id = step.geometry_id;
        result.forward = step.forward;

        const auto static_duration = GetStaticGeometryDuration(facade, step.geometry_id, step.forward);

        if (bucket_size_minutes > 0 && week_bucket_count > 0)
        {
            result.week_bucket =
                std::min(TimestampToWeekBucket(evaluation.arrival_timestamp, bucket_size_minutes),
                         week_bucket_count - 1);

            const auto temporal_duration =
                step.forward ? facade.GetTemporalForwardDuration(step.geometry_id, result.week_bucket)
                             : facade.GetTemporalReverseDuration(step.geometry_id, result.week_bucket);

            if (temporal_duration != INVALID_EDGE_DURATION)
            {
                result.duration = temporal_duration;
                result.used_temporal = true;
            }
            else
            {
                result.duration = static_duration;
            }
        }
        else
        {
            result.duration = static_duration;
        }

        evaluation.total_duration += result.duration;
        evaluation.arrival_timestamp += from_alias<std::int32_t>(result.duration) / 10;
        evaluation.steps.push_back(result);
    }

    return evaluation;
}

} // namespace osrm::engine::temporal

#endif
