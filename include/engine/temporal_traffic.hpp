#ifndef OSRM_ENGINE_TEMPORAL_TRAFFIC_HPP
#define OSRM_ENGINE_TEMPORAL_TRAFFIC_HPP

#include "engine/internal_route_result.hpp"
#include "engine/phantom_node.hpp"

#include "util/integer_range.hpp"
#include "util/typedefs.hpp"

#include <algorithm>
#include <cmath>
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

struct TemporalGeometryDuration
{
    std::uint32_t week_bucket = 0;
    EdgeDuration duration = EdgeDuration{0};
    bool used_temporal = false;
};

struct TemporalLegEvaluation
{
    std::vector<PathData> path_data;
    PhantomNode source_phantom;
    PhantomNode target_phantom;
    std::time_t arrival_timestamp = 0;
    bool used_temporal = false;
};

struct TemporalRouteEvaluation
{
    EdgeDuration total_duration = EdgeDuration{0};
    std::time_t arrival_timestamp = 0;
    bool used_temporal = false;
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

inline EdgeDuration ScaleDurationProportionally(const EdgeDuration value,
                                                const EdgeDuration static_total,
                                                const EdgeDuration scaled_total)
{
    const auto static_total_raw = from_alias<std::int64_t>(static_total);
    if (static_total_raw <= 0)
    {
        return EdgeDuration{0};
    }

    const auto value_raw = from_alias<std::int64_t>(value);
    const auto scaled_total_raw = from_alias<std::int64_t>(scaled_total);
    return to_alias<EdgeDuration>(
        std::llround(static_cast<long double>(value_raw) * scaled_total_raw / static_total_raw));
}

inline std::vector<EdgeDuration> ScaleDurationsToTotal(const std::vector<EdgeDuration> &durations,
                                                       const EdgeDuration scaled_total)
{
    std::vector<EdgeDuration> scaled(durations.size(), EdgeDuration{0});
    if (durations.empty())
    {
        return scaled;
    }

    const auto total_raw = from_alias<std::int64_t>(scaled_total);
    if (total_raw <= 0)
    {
        return scaled;
    }

    const auto static_total_raw =
        std::accumulate(durations.begin(),
                        durations.end(),
                        std::int64_t{0},
                        [](const std::int64_t sum, const EdgeDuration duration)
                        { return sum + from_alias<std::int64_t>(duration); });

    if (static_total_raw <= 0)
    {
        return scaled;
    }

    struct FractionalPart
    {
        std::size_t index = 0;
        long double remainder = 0.;
    };

    std::vector<FractionalPart> fractional_parts;
    fractional_parts.reserve(durations.size());

    std::int64_t assigned_total = 0;
    for (std::size_t index = 0; index < durations.size(); ++index)
    {
        const auto duration_raw = from_alias<std::int64_t>(durations[index]);
        const auto exact = static_cast<long double>(duration_raw) * total_raw / static_total_raw;
        const auto base = static_cast<std::int64_t>(std::floor(exact));
        scaled[index] = to_alias<EdgeDuration>(base);
        assigned_total += base;
        fractional_parts.push_back({index, exact - base});
    }

    auto remaining = total_raw - assigned_total;
    std::sort(fractional_parts.begin(),
              fractional_parts.end(),
              [](const FractionalPart &lhs, const FractionalPart &rhs)
              { return lhs.remainder > rhs.remainder; });

    for (const auto &part : fractional_parts)
    {
        if (remaining <= 0)
        {
            break;
        }

        scaled[part.index] += EdgeDuration{1};
        --remaining;
    }

    return scaled;
}

inline EdgeDuration GetTraversalDuration(const PhantomNode &phantom, const bool traversed_in_reverse)
{
    return traversed_in_reverse ? phantom.reverse_duration : phantom.forward_duration;
}

inline void SetTraversalDuration(PhantomNode &phantom,
                                 const bool traversed_in_reverse,
                                 const EdgeDuration duration)
{
    if (traversed_in_reverse)
    {
        phantom.reverse_duration = duration;
    }
    else
    {
        phantom.forward_duration = duration;
    }
}

inline EdgeDuration GetRouteLegDuration(const std::vector<PathData> &route_data,
                                        const PhantomNode &source_phantom,
                                        const PhantomNode &target_phantom,
                                        const bool target_traversed_in_reverse)
{
    auto duration = std::accumulate(route_data.begin(),
                                    route_data.end(),
                                    EdgeDuration{0},
                                    [](const EdgeDuration sum, const PathData &data)
                                    { return sum + data.duration_until_turn; });

    duration += detail::GetTraversalDuration(target_phantom, target_traversed_in_reverse);

    if (route_data.empty())
    {
        duration = std::max(duration -
                                detail::GetTraversalDuration(source_phantom,
                                                             target_traversed_in_reverse),
                            EdgeDuration{0});
    }

    return duration;
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

inline std::time_t AdvanceTimestamp(const std::time_t timestamp, const EdgeDuration duration)
{
    return timestamp + from_alias<std::int64_t>(duration) / 10;
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
TemporalGeometryDuration GetGeometryDurationAtTimestamp(const FacadeT &facade,
                                                        const PackedGeometryID geometry_id,
                                                        const bool forward,
                                                        const std::time_t timestamp)
{
    TemporalGeometryDuration result;
    result.duration = GetStaticGeometryDuration(facade, geometry_id, forward);

    const auto bucket_size_minutes = facade.GetTemporalBucketSizeMinutes();
    const auto week_bucket_count = facade.GetTemporalWeekBucketCount();
    if (bucket_size_minutes == 0 || week_bucket_count == 0)
    {
        return result;
    }

    result.week_bucket =
        std::min(TimestampToWeekBucket(timestamp, bucket_size_minutes), week_bucket_count - 1);

    const auto temporal_duration =
        forward ? facade.GetTemporalForwardDuration(geometry_id, result.week_bucket)
                : facade.GetTemporalReverseDuration(geometry_id, result.week_bucket);

    if (temporal_duration != INVALID_EDGE_DURATION)
    {
        result.duration = temporal_duration;
        result.used_temporal = true;
    }

    return result;
}

template <typename FacadeT>
TemporalLegEvaluation EvaluateRouteLeg(const FacadeT &facade,
                                       const std::vector<PathData> &path_data,
                                       const PhantomNode &source_phantom,
                                       const PhantomNode &target_phantom,
                                       const bool source_traversed_in_reverse,
                                       const bool target_traversed_in_reverse,
                                       const std::time_t departure_timestamp)
{
    TemporalLegEvaluation evaluation;
    evaluation.path_data = path_data;
    evaluation.source_phantom = source_phantom;
    evaluation.target_phantom = target_phantom;
    evaluation.arrival_timestamp = departure_timestamp;

    const auto target_node_id = target_traversed_in_reverse
                                    ? target_phantom.reverse_segment_id.id
                                    : target_phantom.forward_segment_id.id;
    const auto target_geometry = facade.GetGeometryIndex(target_node_id);
    const auto static_target_duration =
        detail::GetTraversalDuration(target_phantom, target_traversed_in_reverse);

    if (evaluation.path_data.empty())
    {
        const auto source_node_id = source_traversed_in_reverse
                                        ? source_phantom.reverse_segment_id.id
                                        : source_phantom.forward_segment_id.id;
        const auto source_geometry = facade.GetGeometryIndex(source_node_id);
        const auto geometry_duration = GetGeometryDurationAtTimestamp(
            facade, source_geometry.id, source_geometry.forward, evaluation.arrival_timestamp);
        const auto static_total =
            GetStaticGeometryDuration(facade, source_geometry.id, source_geometry.forward);

        const auto static_source_duration =
            detail::GetTraversalDuration(source_phantom, source_traversed_in_reverse);
        const auto adjusted_source_duration = detail::ScaleDurationProportionally(
            static_source_duration, static_total, geometry_duration.duration);
        const auto adjusted_target_duration = detail::ScaleDurationProportionally(
            static_target_duration, static_total, geometry_duration.duration);

        detail::SetTraversalDuration(
            evaluation.source_phantom, source_traversed_in_reverse, adjusted_source_duration);
        detail::SetTraversalDuration(
            evaluation.target_phantom, target_traversed_in_reverse, adjusted_target_duration);

        evaluation.arrival_timestamp = AdvanceTimestamp(
            evaluation.arrival_timestamp,
            std::max(adjusted_target_duration - adjusted_source_duration, EdgeDuration{0}));
        evaluation.used_temporal = geometry_duration.used_temporal;
        return evaluation;
    }

    bool target_duration_adjusted = false;
    for (std::size_t group_begin = 0; group_begin < evaluation.path_data.size();)
    {
        std::size_t group_end = group_begin + 1;
        while (group_end < evaluation.path_data.size() &&
               evaluation.path_data[group_end].from_edge_based_node ==
                   evaluation.path_data[group_begin].from_edge_based_node)
        {
            ++group_end;
        }

        const auto geometry_index =
            facade.GetGeometryIndex(evaluation.path_data[group_begin].from_edge_based_node);
        const auto geometry_duration = GetGeometryDurationAtTimestamp(
            facade, geometry_index.id, geometry_index.forward, evaluation.arrival_timestamp);
        const auto static_total =
            GetStaticGeometryDuration(facade, geometry_index.id, geometry_index.forward);

        std::vector<EdgeDuration> static_parts;
        static_parts.reserve(group_end - group_begin + 1);

        for (auto index = group_begin; index < group_end; ++index)
        {
            static_parts.push_back(std::max(evaluation.path_data[index].duration_until_turn -
                                                evaluation.path_data[index].duration_of_turn,
                                            EdgeDuration{0}));
        }

        const auto is_last_group = group_end == evaluation.path_data.size();
        const auto include_target_duration =
            is_last_group && target_geometry.id == geometry_index.id &&
            target_geometry.forward == geometry_index.forward;
        if (include_target_duration)
        {
            static_parts.push_back(static_target_duration);
        }

        const auto static_subset_total =
            std::accumulate(static_parts.begin(), static_parts.end(), EdgeDuration{0});
        const auto scaled_subset_total = geometry_duration.used_temporal
                                             ? detail::ScaleDurationProportionally(
                                                   static_subset_total,
                                                   static_total,
                                                   geometry_duration.duration)
                                             : static_subset_total;
        const auto scaled_parts = detail::ScaleDurationsToTotal(static_parts, scaled_subset_total);

        EdgeDuration traversed_duration = EdgeDuration{0};
        for (auto index = group_begin; index < group_end; ++index)
        {
            const auto scaled_segment_duration = scaled_parts[index - group_begin];
            evaluation.path_data[index].duration_until_turn =
                scaled_segment_duration + evaluation.path_data[index].duration_of_turn;
            traversed_duration += evaluation.path_data[index].duration_until_turn;
        }

        if (include_target_duration)
        {
            const auto scaled_target_duration = scaled_parts.back();
            detail::SetTraversalDuration(
                evaluation.target_phantom, target_traversed_in_reverse, scaled_target_duration);
            traversed_duration += scaled_target_duration;
            target_duration_adjusted = true;
        }

        evaluation.arrival_timestamp =
            AdvanceTimestamp(evaluation.arrival_timestamp, traversed_duration);
        evaluation.used_temporal = evaluation.used_temporal || geometry_duration.used_temporal;
        group_begin = group_end;
    }

    if (!target_duration_adjusted)
    {
        const auto geometry_duration = GetGeometryDurationAtTimestamp(
            facade, target_geometry.id, target_geometry.forward, evaluation.arrival_timestamp);
        const auto static_total =
            GetStaticGeometryDuration(facade, target_geometry.id, target_geometry.forward);
        const auto adjusted_target_duration = detail::ScaleDurationProportionally(
            static_target_duration, static_total, geometry_duration.duration);

        detail::SetTraversalDuration(
            evaluation.target_phantom, target_traversed_in_reverse, adjusted_target_duration);
        evaluation.arrival_timestamp =
            AdvanceTimestamp(evaluation.arrival_timestamp, adjusted_target_duration);
        evaluation.used_temporal = evaluation.used_temporal || geometry_duration.used_temporal;
    }

    return evaluation;
}

template <typename FacadeT>
TemporalPathEvaluation EvaluateGeometryPath(const FacadeT &facade,
                                            const std::vector<TemporalGeometryStep> &steps,
                                            const std::time_t departure_timestamp)
{
    TemporalPathEvaluation evaluation;
    evaluation.arrival_timestamp = departure_timestamp;
    evaluation.steps.reserve(steps.size());

    for (const auto &step : steps)
    {
        TemporalStepResult result;
        result.geometry_id = step.geometry_id;
        result.forward = step.forward;

        const auto geometry_duration = GetGeometryDurationAtTimestamp(
            facade, step.geometry_id, step.forward, evaluation.arrival_timestamp);
        result.week_bucket = geometry_duration.week_bucket;
        result.duration = geometry_duration.duration;
        result.used_temporal = geometry_duration.used_temporal;

        evaluation.total_duration += result.duration;
        evaluation.arrival_timestamp = AdvanceTimestamp(evaluation.arrival_timestamp, result.duration);
        evaluation.steps.push_back(result);
    }

    return evaluation;
}

template <typename FacadeT>
TemporalRouteEvaluation EvaluateRoute(const FacadeT &facade,
                                      const InternalRouteResult &route,
                                      const std::time_t departure_timestamp)
{
    TemporalRouteEvaluation evaluation;
    evaluation.arrival_timestamp = departure_timestamp;

    for (auto leg_index : util::irange<std::size_t>(0UL, route.leg_endpoints.size()))
    {
        const auto leg_evaluation = EvaluateRouteLeg(facade,
                                                     route.unpacked_path_segments[leg_index],
                                                     route.leg_endpoints[leg_index].source_phantom,
                                                     route.leg_endpoints[leg_index].target_phantom,
                                                     route.source_traversed_in_reverse[leg_index],
                                                     route.target_traversed_in_reverse[leg_index],
                                                     evaluation.arrival_timestamp);

        evaluation.total_duration += detail::GetRouteLegDuration(leg_evaluation.path_data,
                                                                 leg_evaluation.source_phantom,
                                                                 leg_evaluation.target_phantom,
                                                                 route.target_traversed_in_reverse
                                                                     [leg_index]);
        evaluation.arrival_timestamp = leg_evaluation.arrival_timestamp;
        evaluation.used_temporal = evaluation.used_temporal || leg_evaluation.used_temporal;
    }

    return evaluation;
}

template <typename FacadeT>
bool RerankRoutesByTemporalDuration(const FacadeT &facade,
                                    InternalManyRoutesResult &routes,
                                    const std::time_t departure_timestamp)
{
    struct TemporalCandidateScore
    {
        std::size_t index = 0;
        EdgeDuration duration = INVALID_EDGE_DURATION;
        bool valid = false;
        bool used_temporal = false;
    };

    std::vector<TemporalCandidateScore> scores;
    scores.reserve(routes.routes.size());

    for (auto index : util::irange<std::size_t>(0UL, routes.routes.size()))
    {
        const auto &route = routes.routes[index];
        if (!route.is_valid())
        {
            scores.push_back({index, INVALID_EDGE_DURATION, false, false});
            continue;
        }

        const auto evaluation = EvaluateRoute(facade, route, departure_timestamp);
        scores.push_back({index, evaluation.total_duration, true, evaluation.used_temporal});
    }

    const auto any_temporal = std::any_of(
        scores.begin(), scores.end(), [](const auto &score) { return score.used_temporal; });
    if (!any_temporal)
    {
        return false;
    }

    std::stable_sort(scores.begin(),
                     scores.end(),
                     [](const TemporalCandidateScore &lhs, const TemporalCandidateScore &rhs)
                     {
                         if (lhs.valid != rhs.valid)
                         {
                             return lhs.valid > rhs.valid;
                         }

                         if (!lhs.valid)
                         {
                             return false;
                         }

                         return lhs.duration < rhs.duration;
                     });

    std::vector<InternalRouteResult> reordered_routes;
    reordered_routes.reserve(routes.routes.size());
    for (const auto &score : scores)
    {
        reordered_routes.push_back(std::move(routes.routes[score.index]));
    }

    routes.routes = std::move(reordered_routes);
    return true;
}

} // namespace osrm::engine::temporal

#endif
