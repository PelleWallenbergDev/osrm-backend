#include "updater/hit_fcd_source.hpp"

#include "util/coordinate_calculation.hpp"
#include "util/exception.hpp"
#include "util/exception_utils.hpp"
#include "util/log.hpp"

#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/numeric/conversion/cast.hpp>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <limits>
#include <string_view>
#include <unordered_map>

namespace osrm::updater
{
namespace
{

struct TimestampParts final
{
    int year = 0;
    unsigned month = 0;
    unsigned day = 0;
    unsigned hour = 0;
    unsigned minute = 0;
};

struct SpeedAggregate final
{
    double weighted_speed_sum = 0.;
    std::uint64_t sample_count = 0;
    std::uint8_t source = 0;
};

struct DurationAggregate final
{
    double weighted_duration_sum = 0.;
    std::uint64_t sample_count = 0;
    std::uint8_t source = 0;
};

template <typename ValueT> void hashCombine(std::size_t &seed, const ValueT value)
{
    seed ^= std::hash<ValueT>{}(value) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
}

struct HitFCDKeyHash final
{
    std::size_t operator()(const HitFCDKey &key) const
    {
        std::size_t seed = 0;
        hashCombine(seed, key.way_id);
        hashCombine(seed, key.direction);
        hashCombine(seed, key.week_bucket);
        return seed;
    }
};

struct TemporalBucketKeyHash final
{
    std::size_t operator()(const TemporalBucketKey &key) const
    {
        std::size_t seed = 0;
        hashCombine(seed, key.from);
        hashCombine(seed, key.to);
        hashCombine(seed, key.direction);
        hashCombine(seed, key.week_bucket);
        return seed;
    }
};

std::string makeLineMessage(const std::string &path,
                            const std::size_t line_number,
                            const std::string &message)
{
    return path + ":" + std::to_string(line_number) + ": " + message;
}

std::string_view trim(const std::string_view value)
{
    const auto first = value.find_first_not_of(" \t\r");
    if (first == std::string_view::npos)
    {
        return {};
    }

    const auto last = value.find_last_not_of(" \t\r");
    return value.substr(first, last - first + 1);
}

std::vector<std::string_view> splitLine(const std::string_view line, const char delimiter)
{
    std::vector<std::string_view> fields;
    std::size_t begin = 0;
    while (begin <= line.size())
    {
        const auto end = line.find(delimiter, begin);
        if (end == std::string_view::npos)
        {
            fields.push_back(trim(line.substr(begin)));
            break;
        }

        fields.push_back(trim(line.substr(begin, end - begin)));
        begin = end + 1;
    }

    return fields;
}

bool isDigitsOnly(const std::string_view value)
{
    return !value.empty() &&
           std::all_of(value.begin(), value.end(), [](const auto ch) { return ch >= '0' && ch <= '9'; });
}

template <typename IntegerT>
IntegerT parseInteger(const std::string_view value,
                      const std::string &path,
                      const std::size_t line_number,
                      const std::string &field_name)
{
    const auto trimmed = trim(value);
    IntegerT result{};
    const auto *first = trimmed.data();
    const auto *last = trimmed.data() + trimmed.size();
    const auto parsed = std::from_chars(first, last, result);
    if (parsed.ec != std::errc{} || parsed.ptr != last)
    {
        throw util::exception(makeLineMessage(path,
                                              line_number,
                                              "invalid " + field_name + " value \"" +
                                                  std::string(trimmed) + "\"") +
                              SOURCE_REF);
    }
    return result;
}

double parseDouble(const std::string_view value,
                   const std::string &path,
                   const std::size_t line_number,
                   const std::string &field_name)
{
    const auto trimmed = trim(value);
    try
    {
        std::size_t parsed_length = 0;
        const auto result = std::stod(std::string(trimmed), &parsed_length);
        if (parsed_length != trimmed.size())
        {
            throw util::exception(makeLineMessage(path,
                                                  line_number,
                                                  "invalid " + field_name + " value \"" +
                                                      std::string(trimmed) + "\"") +
                                  SOURCE_REF);
        }
        return result;
    }
    catch (const util::exception &)
    {
        throw;
    }
    catch (const std::exception &)
    {
        throw util::exception(makeLineMessage(path,
                                              line_number,
                                              "invalid " + field_name + " value \"" +
                                                  std::string(trimmed) + "\"") +
                              SOURCE_REF);
    }
}

TimestampParts parseTimestamp(const std::string_view timestamp,
                             const std::string &path,
                             const std::size_t line_number)
{
    const auto trimmed = trim(timestamp);
    if (trimmed.size() < 19 || trimmed[4] != '-' || trimmed[7] != '-' || trimmed[10] != ' ' ||
        trimmed[13] != ':' || trimmed[16] != ':')
    {
        throw util::exception(makeLineMessage(path,
                                              line_number,
                                              "invalid timestamp \"" + std::string(trimmed) +
                                                  "\"") +
                              SOURCE_REF);
    }

    TimestampParts result;
    result.year = parseInteger<int>(trimmed.substr(0, 4), path, line_number, "year");
    result.month = parseInteger<unsigned>(trimmed.substr(5, 2), path, line_number, "month");
    result.day = parseInteger<unsigned>(trimmed.substr(8, 2), path, line_number, "day");
    result.hour = parseInteger<unsigned>(trimmed.substr(11, 2), path, line_number, "hour");
    result.minute = parseInteger<unsigned>(trimmed.substr(14, 2), path, line_number, "minute");
    const auto seconds = parseInteger<unsigned>(trimmed.substr(17, 2), path, line_number, "second");

    if (result.month == 0 || result.month > 12 || result.day == 0 || result.day > 31 ||
        result.hour > 23 || result.minute > 59 || seconds > 59)
    {
        throw util::exception(makeLineMessage(path,
                                              line_number,
                                              "timestamp components are out of range in \"" +
                                                  std::string(trimmed) + "\"") +
                              SOURCE_REF);
    }

    try
    {
        boost::gregorian::date(result.year, result.month, result.day);
    }
    catch (const std::exception &)
    {
        throw util::exception(makeLineMessage(path,
                                              line_number,
                                              "timestamp date is invalid in \"" +
                                                  std::string(trimmed) + "\"") +
                              SOURCE_REF);
    }

    return result;
}

unsigned weekdayFromCivil(const int year, const unsigned month, const unsigned day)
{
    static constexpr int offsets[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    auto adjusted_year = year;
    if (month < 3)
    {
        adjusted_year -= 1;
    }

    return static_cast<unsigned>((
        adjusted_year + adjusted_year / 4 - adjusted_year / 100 + adjusted_year / 400 +
        offsets[month - 1] + static_cast<int>(day)) %
                                  7);
}

std::uint32_t timestampToWeekBucket(const TimestampParts &timestamp,
                                    const std::uint32_t bucket_size_minutes)
{
    const auto weekday = weekdayFromCivil(timestamp.year, timestamp.month, timestamp.day);
    const auto monday_based_day = (weekday + 6) % 7;
    const auto minutes_since_week_start =
        monday_based_day * 24U * 60U + timestamp.hour * 60U + timestamp.minute;
    return minutes_since_week_start / bucket_size_minutes;
}

std::uint8_t mapDirection(const std::uint32_t raw_direction,
                          const std::uint32_t forward_direction_value,
                          const std::uint32_t reverse_direction_value,
                          const std::string &path,
                          const std::size_t line_number)
{
    if (raw_direction == forward_direction_value)
    {
        return HIT_FCD_DIRECTION_FORWARD;
    }
    if (raw_direction == reverse_direction_value)
    {
        return HIT_FCD_DIRECTION_REVERSE;
    }

    throw util::exception(makeLineMessage(path,
                                          line_number,
                                          "unexpected Link_Direction value " +
                                              std::to_string(raw_direction)) +
                          SOURCE_REF);
}

const std::vector<double> &
getSegmentLengths(std::unordered_map<PackedGeometryID, std::vector<double>> &cache,
                  const extractor::SegmentDataContainer &segment_data,
                  const std::vector<util::Coordinate> &coordinates,
                  const PackedGeometryID geometry_id)
{
    const auto cached = cache.find(geometry_id);
    if (cached != cache.end())
    {
        return cached->second;
    }

    std::vector<double> lengths;
    const auto nodes = segment_data.GetForwardGeometry(geometry_id);
    lengths.reserve(nodes.size() > 0 ? nodes.size() - 1 : 0);
    for (auto segment_offset = std::size_t{0}; segment_offset + 1 < nodes.size(); ++segment_offset)
    {
        lengths.push_back(util::coordinate_calculation::greatCircleDistance(
            coordinates[nodes[segment_offset]], coordinates[nodes[segment_offset + 1]]));
    }

    return cache.try_emplace(geometry_id, std::move(lengths)).first->second;
}

SegmentDuration convertSpeedToDuration(const double speed_in_kmh, const double distance_in_meters)
{
    if (speed_in_kmh <= 0.)
    {
        return INVALID_SEGMENT_DURATION;
    }

    const auto speed_in_ms = speed_in_kmh / 3.6;
    const auto duration = distance_in_meters / speed_in_ms;
    auto segment_duration = std::max<SegmentDuration>(
        {1}, {boost::numeric_cast<SegmentDuration::value_type>(std::round(duration * 10.))});
    if (segment_duration >= INVALID_SEGMENT_DURATION)
    {
        util::Log(logWARNING) << "Clamping segment duration " << segment_duration << " to "
                              << MAX_SEGMENT_DURATION;
        segment_duration = MAX_SEGMENT_DURATION;
    }
    return segment_duration;
}

} // namespace

HitFCDLookupTable readHitFCDValues(const std::vector<std::string> &paths,
                                   const std::uint32_t bucket_size_minutes,
                                   const std::uint32_t forward_direction_value,
                                   const std::uint32_t reverse_direction_value)
{
    if (bucket_size_minutes == 0)
    {
        throw util::exception("HIT FCD import requires a non-zero bucket size" + SOURCE_REF);
    }

    std::unordered_map<HitFCDKey, SpeedAggregate, HitFCDKeyHash> aggregates;
    for (auto file_index = std::size_t{0}; file_index < paths.size(); ++file_index)
    {
        const auto &path = paths[file_index];
        std::ifstream input(path);
        if (!input)
        {
            throw util::exception("Could not open HIT FCD file " + path + SOURCE_REF);
        }

        std::size_t line_number = 0;
        std::size_t loaded_rows = 0;
        std::string line;
        while (std::getline(input, line))
        {
            ++line_number;
            const auto line_view = trim(line);
            if (line_view.empty())
            {
                continue;
            }

            const auto delimiter = line_view.find('\t') != std::string_view::npos ? '\t' : ',';
            const auto fields = splitLine(line_view, delimiter);
            if (fields.size() < 4)
            {
                throw util::exception(
                    makeLineMessage(path, line_number, "expected at least 4 delimited fields") +
                    SOURCE_REF);
            }

            if (!isDigitsOnly(fields[0]))
            {
                if (line_number == 1)
                {
                    continue;
                }

                throw util::exception(
                    makeLineMessage(path, line_number, "unexpected non-numeric Link_id field") +
                    SOURCE_REF);
            }

            const auto way_id = parseInteger<std::uint64_t>(fields[0], path, line_number, "Link_id");
            const auto raw_direction =
                parseInteger<std::uint32_t>(fields[1], path, line_number, "Link_Direction");
            const auto direction = mapDirection(raw_direction,
                                                forward_direction_value,
                                                reverse_direction_value,
                                                path,
                                                line_number);
            const auto timestamp = parseTimestamp(fields[2], path, line_number);
            const auto week_bucket = timestampToWeekBucket(timestamp, bucket_size_minutes);
            const auto speed_kmh = parseDouble(fields[3], path, line_number, "Speed");

            if (speed_kmh <= 0.)
            {
                continue;
            }

            std::uint64_t sample_count = 1;
            if (fields.size() >= 5 && !fields[4].empty())
            {
                sample_count =
                    std::max<std::uint64_t>(1,
                                            parseInteger<std::uint64_t>(fields[4],
                                                                        path,
                                                                        line_number,
                                                                        "UniqueEntries"));
            }

            auto &aggregate = aggregates[{way_id, direction, week_bucket}];
            aggregate.weighted_speed_sum += speed_kmh * static_cast<double>(sample_count);
            aggregate.sample_count += sample_count;
            aggregate.source = static_cast<std::uint8_t>(file_index + 1);
            ++loaded_rows;
        }

        util::Log() << "Loaded " << path << " with " << loaded_rows << " row(s)";
    }

    HitFCDLookupTable result;
    result.lookup.reserve(aggregates.size());
    for (const auto &[key, aggregate] : aggregates)
    {
        result.lookup.push_back(
            {key,
             {aggregate.weighted_speed_sum / static_cast<double>(aggregate.sample_count),
              aggregate.sample_count,
              aggregate.source}});
    }
    std::sort(result.lookup.begin(),
              result.lookup.end(),
              [](const auto &lhs, const auto &rhs) { return rhs.first < lhs.first; });

    util::Log() << "In total loaded " << paths.size() << " HIT FCD file(s) with a total of "
                << result.lookup.size() << " unique way bucket values";

    return result;
}

TemporalLookupTable buildTemporalLookupFromHitFCD(
    const HitFCDLookupTable &hit_fcd_lookup,
    const extractor::LinkMapStorage &link_map,
    const extractor::SegmentDataContainer &segment_data,
    const std::vector<util::Coordinate> &coordinates,
    const extractor::PackedOSMIDs &osm_node_ids)
{
    std::unordered_map<TemporalBucketKey, DurationAggregate, TemporalBucketKeyHash> aggregates;
    std::unordered_map<PackedGeometryID, std::vector<double>> length_cache;

    std::size_t matched_way_buckets = 0;
    std::size_t unmatched_way_buckets = 0;

    for (const auto &[way_bucket, observation] : hit_fcd_lookup.lookup)
    {
        const auto [entry_begin, entry_end] =
            link_map.FindRange(way_bucket.way_id, way_bucket.direction);
        if (entry_begin == entry_end)
        {
            ++unmatched_way_buckets;
            continue;
        }

        ++matched_way_buckets;
        for (auto entry_index = entry_begin; entry_index < entry_end; ++entry_index)
        {
            const auto geometry_id = link_map.geometry_ids[entry_index];
            const auto geometry_direction = link_map.geometry_directions[entry_index];
            const auto segment_begin = static_cast<std::size_t>(link_map.segment_begins[entry_index]);
            const auto segment_end = static_cast<std::size_t>(link_map.segment_ends[entry_index]);

            const auto nodes = segment_data.GetForwardGeometry(geometry_id);
            const auto &lengths = getSegmentLengths(length_cache, segment_data, coordinates, geometry_id);
            if (segment_end > lengths.size())
            {
                throw util::exception("Link map references segment range outside geometry " +
                                      std::to_string(geometry_id) + SOURCE_REF);
            }

            const auto temporal_direction = geometry_direction == extractor::LINK_MAP_DIRECTION_FORWARD
                                                ? TEMPORAL_DIRECTION_FORWARD
                                                : TEMPORAL_DIRECTION_REVERSE;

            for (auto segment_offset = segment_begin; segment_offset < segment_end; ++segment_offset)
            {
                const auto from = osm_node_ids[nodes[segment_offset]];
                const auto to = osm_node_ids[nodes[segment_offset + 1]];
                if (from == to)
                {
                    continue;
                }

                const auto duration =
                    convertSpeedToDuration(observation.speed_kmh, lengths[segment_offset]);
                if (duration == INVALID_SEGMENT_DURATION)
                {
                    continue;
                }

                auto &aggregate =
                    aggregates[{static_cast<std::uint64_t>(from),
                                static_cast<std::uint64_t>(to),
                                temporal_direction,
                                way_bucket.week_bucket}];
                aggregate.weighted_duration_sum +=
                    from_alias<SegmentDuration::value_type>(duration) *
                    static_cast<double>(observation.sample_count);
                aggregate.sample_count += observation.sample_count;
                aggregate.source = observation.source;
            }
        }
    }

    TemporalLookupTable result;
    result.lookup.reserve(aggregates.size());
    for (const auto &[key, aggregate] : aggregates)
    {
        TemporalBucketValue value{
            static_cast<std::int32_t>(
                std::llround(aggregate.weighted_duration_sum /
                             static_cast<double>(aggregate.sample_count)))};
        value.source = aggregate.source;
        result.lookup.push_back({key, value});
    }
    std::sort(result.lookup.begin(),
              result.lookup.end(),
              [](const auto &lhs, const auto &rhs) { return rhs.first < lhs.first; });

    util::Log() << "Mapped " << matched_way_buckets << " HIT FCD way bucket value(s) to "
                << result.lookup.size() << " temporal segment bucket(s)";
    if (unmatched_way_buckets > 0)
    {
        util::Log(logWARNING) << unmatched_way_buckets
                              << " HIT FCD way bucket value(s) could not be mapped to geometry";
    }

    return result;
}

} // namespace osrm::updater
