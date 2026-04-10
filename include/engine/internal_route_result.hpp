#ifndef RAW_ROUTE_DATA_H
#define RAW_ROUTE_DATA_H

#include "extractor/class_data.hpp"
#include "extractor/travel_mode.hpp"

#include "guidance/turn_bearing.hpp"
#include "guidance/turn_instruction.hpp"

#include "engine/phantom_node.hpp"

#include "util/coordinate.hpp"
#include "util/guidance/entry_class.hpp"
#include "util/guidance/turn_lanes.hpp"
#include "util/integer_range.hpp"
#include "util/typedefs.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace osrm::engine
{

struct TemporalAsymmetricSearchDiagnostics
{
    std::uint64_t endpoint_pairs_tried = 0;
    std::uint64_t endpoint_pairs_with_static_upper_bound = 0;
    std::uint64_t static_upper_bound_pair_match_count = 0;
    std::uint64_t static_upper_bound_source_mismatch_count = 0;
    std::uint64_t static_upper_bound_target_mismatch_count = 0;
    std::uint64_t static_upper_bound_direction_mismatch_count = 0;
    std::uint64_t static_upper_bound_route_endpoint_unavailable_count = 0;
    std::uint64_t reverse_lower_bound_source_invalid = 0;
    std::uint64_t queue_exhausted_without_target = 0;
    std::uint64_t pruned_by_initial_upper_bound = 0;
    std::uint64_t pruned_by_best_upper_bound = 0;
    std::uint64_t invalid_duration_relaxations = 0;
    std::uint64_t target_reached = 0;
    std::uint64_t best_candidate_rejected = 0;
    std::uint64_t expanded_nodes = 0;
    std::uint64_t relaxation_attempts = 0;
    std::uint64_t relaxation_improvements = 0;
    std::uint64_t source_first_pop_pruned_by_initial_upper_bound = 0;
    std::int64_t min_initial_upper_bound_prune_margin =
        std::numeric_limits<std::int64_t>::max();
    std::int64_t max_initial_upper_bound_prune_margin =
        std::numeric_limits<std::int64_t>::min();
    std::int64_t first_static_upper_bound_mismatch_directed_source_node = -1;
    std::int64_t first_static_upper_bound_mismatch_directed_target_node = -1;
    std::int64_t first_static_upper_bound_mismatch_route_source_node = -1;
    std::int64_t first_static_upper_bound_mismatch_route_target_node = -1;

    void RecordInitialUpperBoundPruneMargin(const std::int64_t margin)
    {
        min_initial_upper_bound_prune_margin =
            std::min(min_initial_upper_bound_prune_margin, margin);
        max_initial_upper_bound_prune_margin =
            std::max(max_initial_upper_bound_prune_margin, margin);
    }

    void RecordStaticUpperBoundDirectionCheck(const NodeID directed_source_node,
                                              const NodeID directed_target_node,
                                              const NodeID route_source_node,
                                              const NodeID route_target_node)
    {
        const auto directed_source_raw = static_cast<std::int64_t>(directed_source_node);
        const auto directed_target_raw = static_cast<std::int64_t>(directed_target_node);
        const auto route_source_raw = static_cast<std::int64_t>(route_source_node);
        const auto route_target_raw = static_cast<std::int64_t>(route_target_node);

        const bool source_matches = directed_source_raw == route_source_raw;
        const bool target_matches = directed_target_raw == route_target_raw;

        if (source_matches && target_matches)
        {
            ++static_upper_bound_pair_match_count;
            return;
        }

        if (!source_matches)
        {
            ++static_upper_bound_source_mismatch_count;
        }
        if (!target_matches)
        {
            ++static_upper_bound_target_mismatch_count;
        }
        ++static_upper_bound_direction_mismatch_count;

        if (first_static_upper_bound_mismatch_directed_source_node < 0)
        {
            first_static_upper_bound_mismatch_directed_source_node = directed_source_raw;
            first_static_upper_bound_mismatch_directed_target_node = directed_target_raw;
            first_static_upper_bound_mismatch_route_source_node = route_source_raw;
            first_static_upper_bound_mismatch_route_target_node = route_target_raw;
        }
    }

    std::int64_t MinInitialUpperBoundPruneMarginOrSentinel() const
    {
        return min_initial_upper_bound_prune_margin == std::numeric_limits<std::int64_t>::max()
                   ? -1
                   : min_initial_upper_bound_prune_margin;
    }

    std::int64_t MaxInitialUpperBoundPruneMarginOrSentinel() const
    {
        return max_initial_upper_bound_prune_margin == std::numeric_limits<std::int64_t>::min()
                   ? -1
                   : max_initial_upper_bound_prune_margin;
    }

    void Merge(const TemporalAsymmetricSearchDiagnostics &other)
    {
        endpoint_pairs_tried += other.endpoint_pairs_tried;
        endpoint_pairs_with_static_upper_bound += other.endpoint_pairs_with_static_upper_bound;
        static_upper_bound_pair_match_count += other.static_upper_bound_pair_match_count;
        static_upper_bound_source_mismatch_count += other.static_upper_bound_source_mismatch_count;
        static_upper_bound_target_mismatch_count += other.static_upper_bound_target_mismatch_count;
        static_upper_bound_direction_mismatch_count +=
            other.static_upper_bound_direction_mismatch_count;
        static_upper_bound_route_endpoint_unavailable_count +=
            other.static_upper_bound_route_endpoint_unavailable_count;
        reverse_lower_bound_source_invalid += other.reverse_lower_bound_source_invalid;
        queue_exhausted_without_target += other.queue_exhausted_without_target;
        pruned_by_initial_upper_bound += other.pruned_by_initial_upper_bound;
        pruned_by_best_upper_bound += other.pruned_by_best_upper_bound;
        invalid_duration_relaxations += other.invalid_duration_relaxations;
        target_reached += other.target_reached;
        best_candidate_rejected += other.best_candidate_rejected;
        expanded_nodes += other.expanded_nodes;
        relaxation_attempts += other.relaxation_attempts;
        relaxation_improvements += other.relaxation_improvements;
        source_first_pop_pruned_by_initial_upper_bound +=
            other.source_first_pop_pruned_by_initial_upper_bound;
        if (other.min_initial_upper_bound_prune_margin !=
            std::numeric_limits<std::int64_t>::max())
        {
            min_initial_upper_bound_prune_margin =
                std::min(min_initial_upper_bound_prune_margin,
                         other.min_initial_upper_bound_prune_margin);
        }
        if (other.max_initial_upper_bound_prune_margin !=
            std::numeric_limits<std::int64_t>::min())
        {
            max_initial_upper_bound_prune_margin =
                std::max(max_initial_upper_bound_prune_margin,
                         other.max_initial_upper_bound_prune_margin);
        }
        if (first_static_upper_bound_mismatch_directed_source_node < 0 &&
            other.first_static_upper_bound_mismatch_directed_source_node >= 0)
        {
            first_static_upper_bound_mismatch_directed_source_node =
                other.first_static_upper_bound_mismatch_directed_source_node;
            first_static_upper_bound_mismatch_directed_target_node =
                other.first_static_upper_bound_mismatch_directed_target_node;
            first_static_upper_bound_mismatch_route_source_node =
                other.first_static_upper_bound_mismatch_route_source_node;
            first_static_upper_bound_mismatch_route_target_node =
                other.first_static_upper_bound_mismatch_route_target_node;
        }
    }
};

struct TemporalRouteEvaluationDiagnostics
{
    std::uint64_t route_geometries_with_temporal_profiles = 0;
    std::uint64_t route_geometries_missing_temporal_profiles = 0;
    std::uint64_t route_plausibility_rejections = 0;
    std::uint64_t route_steps_used_temporal = 0;

    void Merge(const TemporalRouteEvaluationDiagnostics &other)
    {
        route_geometries_with_temporal_profiles +=
            other.route_geometries_with_temporal_profiles;
        route_geometries_missing_temporal_profiles +=
            other.route_geometries_missing_temporal_profiles;
        route_plausibility_rejections += other.route_plausibility_rejections;
        route_steps_used_temporal += other.route_steps_used_temporal;
    }
};

struct PathData
{
    // from edge-based-node id
    NodeID from_edge_based_node;
    // the internal OSRM id of the OSM node id that is the via node of the turn
    NodeID turn_via_node;
    // weight that is traveled on the segment until the turn is reached
    // including the turn weight, if one exists
    EdgeWeight weight_until_turn;
    // If this segment immediately precedes a turn, then duration_of_turn
    // will contain the weight of the turn.  Otherwise it will be 0.
    EdgeWeight weight_of_turn;
    // duration that is traveled on the segment until the turn is reached,
    // including a turn if the segment precedes one.
    EdgeDuration duration_until_turn;
    // If this segment immediately precedes a turn, then duration_of_turn
    // will contain the duration of the turn.  Otherwise it will be 0.
    EdgeDuration duration_of_turn;
    // Source of the speed value on this road segment
    DatasourceID datasource_id;
    // If segment precedes a turn, ID of the turn itself
    std::optional<EdgeID> turn_edge;
};

struct InternalRouteResult
{
    std::vector<std::vector<PathData>> unpacked_path_segments;
    std::vector<PhantomEndpoints> leg_endpoints;
    std::vector<bool> source_traversed_in_reverse;
    std::vector<bool> target_traversed_in_reverse;
    EdgeWeight shortest_path_weight = INVALID_EDGE_WEIGHT;
    std::optional<TemporalAsymmetricSearchDiagnostics> temporal_asymmetric_debug;

    bool is_valid() const { return INVALID_EDGE_WEIGHT != shortest_path_weight; }

    bool is_via_leg(const std::size_t leg) const
    {
        return (leg != unpacked_path_segments.size() - 1);
    }

    // Note: includes duration for turns, except for at start and end node.
    EdgeDuration duration() const
    {
        EdgeDuration ret{0};

        for (const auto &leg : unpacked_path_segments)
            for (const auto &segment : leg)
                ret += segment.duration_until_turn;

        return ret;
    }
};

struct InternalManyRoutesResult
{
    InternalManyRoutesResult() = default;
    InternalManyRoutesResult(InternalRouteResult route) : routes{std::move(route)} {}
    InternalManyRoutesResult(std::vector<InternalRouteResult> routes_) : routes{std::move(routes_)}
    {
    }

    std::vector<InternalRouteResult> routes;
};

inline InternalRouteResult CollapseInternalRouteResult(const InternalRouteResult &leggy_result,
                                                       const std::vector<bool> &is_waypoint)
{
    BOOST_ASSERT(leggy_result.is_valid());
    BOOST_ASSERT(is_waypoint[0]);     // first and last coords
    BOOST_ASSERT(is_waypoint.back()); // should always be waypoints
    // Nothing to collapse! return result as is
    if (leggy_result.unpacked_path_segments.size() == 1)
        return leggy_result;

    BOOST_ASSERT(leggy_result.leg_endpoints.size() > 1);

    InternalRouteResult collapsed;
    collapsed.shortest_path_weight = leggy_result.shortest_path_weight;
    collapsed.temporal_asymmetric_debug = leggy_result.temporal_asymmetric_debug;
    for (auto i : util::irange<std::size_t>(0, leggy_result.unpacked_path_segments.size()))
    {
        if (is_waypoint[i])
        {
            // start another leg vector
            collapsed.unpacked_path_segments.push_back(leggy_result.unpacked_path_segments[i]);
            // save new phantom node pair
            collapsed.leg_endpoints.push_back(leggy_result.leg_endpoints[i]);
            // save data about phantom nodes
            collapsed.source_traversed_in_reverse.push_back(
                leggy_result.source_traversed_in_reverse[i]);
            collapsed.target_traversed_in_reverse.push_back(
                leggy_result.target_traversed_in_reverse[i]);
        }
        else
        // no new leg, collapse the next segment into the last leg
        {
            BOOST_ASSERT(!collapsed.unpacked_path_segments.empty());
            auto &last_segment = collapsed.unpacked_path_segments.back();
            BOOST_ASSERT(!collapsed.leg_endpoints.empty());
            collapsed.leg_endpoints.back().target_phantom =
                leggy_result.leg_endpoints[i].target_phantom;
            collapsed.target_traversed_in_reverse.back() =
                leggy_result.target_traversed_in_reverse[i];
            // copy path segments into current leg
            if (!leggy_result.unpacked_path_segments[i].empty())
            {
                auto old_size = last_segment.size();
                last_segment.insert(last_segment.end(),
                                    leggy_result.unpacked_path_segments[i].begin(),
                                    leggy_result.unpacked_path_segments[i].end());

                // The first segment of the unpacked path is missing the weight of the
                // source phantom.  We need to add those values back so that the total
                // edge weight is correct
                last_segment[old_size].weight_until_turn +=

                    leggy_result.source_traversed_in_reverse[i]
                        ? leggy_result.leg_endpoints[i].source_phantom.reverse_weight
                        : leggy_result.leg_endpoints[i].source_phantom.forward_weight;

                last_segment[old_size].duration_until_turn +=
                    leggy_result.source_traversed_in_reverse[i]
                        ? leggy_result.leg_endpoints[i].source_phantom.reverse_duration
                        : leggy_result.leg_endpoints[i].source_phantom.forward_duration;
            }
        }
    }

    BOOST_ASSERT(collapsed.leg_endpoints.size() == collapsed.unpacked_path_segments.size());
    return collapsed;
}
} // namespace osrm::engine

#endif // RAW_ROUTE_DATA_H
