#include "extractor/link_map.hpp"

#include "util/log.hpp"

#include <boost/functional/hash.hpp>

#include <algorithm>
#include <limits>
#include <map>
#include <optional>
#include <unordered_map>
#include <utility>

namespace osrm::extractor
{
namespace
{

struct OrderedNodePair final
{
    OSMNodeID from;
    OSMNodeID to;

    bool operator==(const OrderedNodePair &rhs) const
    {
        return from == rhs.from && to == rhs.to;
    }
};

struct OrderedNodePairHash final
{
    std::size_t operator()(const OrderedNodePair &pair) const
    {
        std::size_t seed = 0;
        boost::hash_combine(seed, static_cast<std::uint64_t>(pair.from));
        boost::hash_combine(seed, static_cast<std::uint64_t>(pair.to));
        return seed;
    }
};

struct GeometryPairOccurrence final
{
    PackedGeometryID geometry_id = 0;
    std::uint32_t segment_offset = 0;
    std::uint8_t geometry_direction = LINK_MAP_DIRECTION_FORWARD;
};

using PairOccurrenceMap =
    std::unordered_map<OrderedNodePair, std::vector<GeometryPairOccurrence>, OrderedNodePairHash>;

bool CanContinue(const GeometryPairOccurrence &lhs, const GeometryPairOccurrence &rhs)
{
    if (lhs.geometry_id != rhs.geometry_id || lhs.geometry_direction != rhs.geometry_direction)
    {
        return false;
    }

    if (lhs.geometry_direction == LINK_MAP_DIRECTION_FORWARD)
    {
        return rhs.segment_offset == lhs.segment_offset + 1;
    }

    return rhs.segment_offset + 1 == lhs.segment_offset;
}

LinkMapEntry MakeEntry(const GeometryPairOccurrence &begin, const GeometryPairOccurrence &end)
{
    if (begin.geometry_direction == LINK_MAP_DIRECTION_FORWARD)
    {
        return {begin.geometry_id,
                begin.geometry_direction,
                begin.segment_offset,
                end.segment_offset + 1};
    }

    return {begin.geometry_id,
            begin.geometry_direction,
            end.segment_offset,
            begin.segment_offset + 1};
}

std::vector<OrderedNodePair> BuildWayPairs(const WayNodeStorage &way_node_storage,
                                           const std::size_t way_index,
                                           const std::uint8_t direction)
{
    std::vector<OrderedNodePair> pairs;
    const auto nodes = way_node_storage.GetNodes(way_index);
    if (nodes.size() < 2)
    {
        return pairs;
    }

    pairs.reserve(nodes.size() - 1);
    if (direction == LINK_MAP_DIRECTION_FORWARD)
    {
        for (auto index = std::size_t{0}; index + 1 < nodes.size(); ++index)
        {
            if (nodes[index] != nodes[index + 1])
            {
                pairs.push_back({nodes[index], nodes[index + 1]});
            }
        }
    }
    else
    {
        for (auto index = nodes.size() - 1; index > 0; --index)
        {
            if (nodes[index] != nodes[index - 1])
            {
                pairs.push_back({nodes[index], nodes[index - 1]});
            }
        }
    }

    return pairs;
}

struct ResolvedWayDirection final
{
    std::vector<GeometryPairOccurrence> occurrences;
    bool ambiguous = false;
};

std::optional<ResolvedWayDirection> ResolveWayDirection(const std::vector<OrderedNodePair> &pairs,
                                                        const PairOccurrenceMap &occurrences)
{
    if (pairs.empty())
    {
        return std::nullopt;
    }

    std::vector<const std::vector<GeometryPairOccurrence> *> candidate_sets;
    candidate_sets.reserve(pairs.size());

    bool ambiguous = false;
    for (const auto &pair : pairs)
    {
        const auto found = occurrences.find(pair);
        if (found == occurrences.end())
        {
            return std::nullopt;
        }

        candidate_sets.push_back(&found->second);
        ambiguous = ambiguous || found->second.size() > 1;
    }

    std::vector<std::vector<int>> scores(candidate_sets.size());
    std::vector<std::vector<std::size_t>> backpointers(candidate_sets.size());

    for (auto index = std::size_t{0}; index < candidate_sets.size(); ++index)
    {
        const auto candidate_count = candidate_sets[index]->size();
        scores[index].assign(candidate_count, std::numeric_limits<int>::min());
        backpointers[index].assign(candidate_count, 0);
    }

    std::fill(scores.front().begin(), scores.front().end(), 0);

    for (auto pair_index = std::size_t{1}; pair_index < candidate_sets.size(); ++pair_index)
    {
        const auto &current = *candidate_sets[pair_index];
        const auto &previous = *candidate_sets[pair_index - 1];

        for (auto current_index = std::size_t{0}; current_index < current.size(); ++current_index)
        {
            for (auto previous_index = std::size_t{0}; previous_index < previous.size();
                 ++previous_index)
            {
                auto score = scores[pair_index - 1][previous_index];
                if (score == std::numeric_limits<int>::min())
                {
                    continue;
                }

                if (CanContinue(previous[previous_index], current[current_index]))
                {
                    score += 2;
                }

                if (score > scores[pair_index][current_index])
                {
                    scores[pair_index][current_index] = score;
                    backpointers[pair_index][current_index] = previous_index;
                }
            }
        }
    }

    const auto &last_scores = scores.back();
    const auto best_it = std::max_element(last_scores.begin(), last_scores.end());
    if (best_it == last_scores.end() || *best_it == std::numeric_limits<int>::min())
    {
        return std::nullopt;
    }

    auto best_index = static_cast<std::size_t>(best_it - last_scores.begin());
    std::vector<GeometryPairOccurrence> resolved(candidate_sets.size());
    for (auto pair_index = candidate_sets.size(); pair_index > 0; --pair_index)
    {
        resolved[pair_index - 1] = candidate_sets[pair_index - 1]->at(best_index);
        if (pair_index > 1)
        {
            best_index = backpointers[pair_index - 1][best_index];
        }
    }

    return ResolvedWayDirection{std::move(resolved), ambiguous};
}

std::vector<LinkMapEntry> CollapseOccurrences(const std::vector<GeometryPairOccurrence> &resolved)
{
    std::vector<LinkMapEntry> entries;
    if (resolved.empty())
    {
        return entries;
    }

    auto range_begin = resolved.front();
    auto previous = resolved.front();
    for (auto index = std::size_t{1}; index < resolved.size(); ++index)
    {
        const auto &current = resolved[index];
        if (!CanContinue(previous, current))
        {
            entries.push_back(MakeEntry(range_begin, previous));
            range_begin = current;
        }
        previous = current;
    }

    entries.push_back(MakeEntry(range_begin, previous));
    return entries;
}

} // namespace

LinkMapStorage BuildLinkMap(const WayNodeStorage &way_node_storage,
                            const SegmentDataContainer &segment_data,
                            const PackedOSMIDs &osm_node_ids)
{
    PairOccurrenceMap pair_occurrences;
    pair_occurrences.reserve(segment_data.GetNumberOfSegments() * 2);

    for (PackedGeometryID geometry_id = 0; geometry_id < segment_data.GetNumberOfGeometries();
         ++geometry_id)
    {
        const auto nodes = segment_data.GetForwardGeometry(geometry_id);
        if (nodes.size() < 2)
        {
            continue;
        }

        for (auto segment_offset = std::uint32_t{0}; segment_offset + 1 < nodes.size();
             ++segment_offset)
        {
            const auto from = osm_node_ids[nodes[segment_offset]];
            const auto to = osm_node_ids[nodes[segment_offset + 1]];
            if (from == to)
            {
                continue;
            }

            pair_occurrences[{from, to}].push_back(
                {geometry_id, segment_offset, LINK_MAP_DIRECTION_FORWARD});
            pair_occurrences[{to, from}].push_back(
                {geometry_id, segment_offset, LINK_MAP_DIRECTION_REVERSE});
        }
    }

    for (auto &[pair, occurrences] : pair_occurrences)
    {
        std::sort(occurrences.begin(),
                  occurrences.end(),
                  [](const auto &lhs, const auto &rhs)
                  {
                      return std::tie(lhs.geometry_id, lhs.geometry_direction, lhs.segment_offset) <
                             std::tie(rhs.geometry_id, rhs.geometry_direction, rhs.segment_offset);
                  });
    }

    std::map<LinkMapKey, std::vector<LinkMapEntry>> keyed_entries;
    std::size_t matched_directions = 0;
    std::size_t unmatched_directions = 0;
    std::size_t ambiguous_directions = 0;

    for (auto way_index = std::size_t{0}; way_index < way_node_storage.size(); ++way_index)
    {
        const auto way_id = static_cast<std::uint64_t>(way_node_storage.way_ids[way_index]);
        for (const auto direction : {LINK_MAP_DIRECTION_FORWARD, LINK_MAP_DIRECTION_REVERSE})
        {
            const auto pairs = BuildWayPairs(way_node_storage, way_index, direction);
            if (pairs.empty())
            {
                continue;
            }

            const auto resolved = ResolveWayDirection(pairs, pair_occurrences);
            if (!resolved)
            {
                ++unmatched_directions;
                continue;
            }

            ++matched_directions;
            ambiguous_directions += resolved->ambiguous ? 1 : 0;
            keyed_entries[{way_id, direction}] = CollapseOccurrences(resolved->occurrences);
        }
    }

    LinkMapStorage storage;
    storage.offsets.push_back(0);

    for (const auto &[key, entries] : keyed_entries)
    {
        storage.way_ids.push_back({key.way_id, key.direction});
        for (const auto &entry : entries)
        {
            storage.geometry_ids.push_back(entry.geometry_id);
            storage.geometry_directions.push_back(entry.geometry_direction);
            storage.segment_begins.push_back(entry.segment_begin);
            storage.segment_ends.push_back(entry.segment_end);
        }
        storage.offsets.push_back(storage.geometry_ids.size());
    }

    util::Log() << "Built link map for " << matched_directions << " directed way(s)";
    if (unmatched_directions > 0)
    {
        util::Log(logWARNING) << unmatched_directions
                              << " directed way(s) could not be mapped to compressed geometry";
    }
    if (ambiguous_directions > 0)
    {
        util::Log(logWARNING) << ambiguous_directions
                              << " directed way(s) required ambiguous pair resolution";
    }

    return storage;
}

} // namespace osrm::extractor
