#include "updater/hit_fcd_source.hpp"

#include "extractor/link_map.hpp"
#include "util/coordinate.hpp"
#include "util/coordinate_calculation.hpp"

#include "../common/temporary_file.hpp"

#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>

BOOST_AUTO_TEST_SUITE(hit_fcd_source)

namespace
{

osrm::extractor::SegmentDataContainer makeSegmentData()
{
    using SegmentData = osrm::extractor::SegmentDataContainer;

    SegmentData::SegmentWeightVector forward_weights{
        SegmentWeight{0},
        SegmentWeight{1},
        SegmentWeight{1},
        SegmentWeight{1}};
    SegmentData::SegmentWeightVector reverse_weights{
        SegmentWeight{0},
        SegmentWeight{1},
        SegmentWeight{1},
        SegmentWeight{1}};
    SegmentData::SegmentDurationVector forward_durations{
        SegmentDuration{0},
        SegmentDuration{1},
        SegmentDuration{1},
        SegmentDuration{1}};
    SegmentData::SegmentDurationVector reverse_durations{
        SegmentDuration{0},
        SegmentDuration{1},
        SegmentDuration{1},
        SegmentDuration{1}};

    return {{0, 4},
            {0, 1, 2, 3},
            std::move(forward_weights),
            std::move(reverse_weights),
            std::move(forward_durations),
            std::move(reverse_durations),
            {0, 0, 0, 0},
            {0, 0, 0, 0}};
}

osrm::extractor::PackedOSMIDs makeOSMNodeIDs()
{
    osrm::extractor::PackedOSMIDs osm_node_ids;
    for (const auto node_id : {10ULL, 11ULL, 12ULL, 13ULL})
    {
        osm_node_ids.push_back(OSMNodeID{node_id});
    }
    return osm_node_ids;
}

std::vector<osrm::util::Coordinate> makeCoordinates()
{
    std::vector<osrm::util::Coordinate> coordinates(4);
    coordinates[0].lon = osrm::util::toFixed(osrm::util::UnsafeFloatLongitude{0.0000});
    coordinates[0].lat = osrm::util::toFixed(osrm::util::UnsafeFloatLatitude{0.0000});
    coordinates[1].lon = osrm::util::toFixed(osrm::util::UnsafeFloatLongitude{0.0010});
    coordinates[1].lat = osrm::util::toFixed(osrm::util::UnsafeFloatLatitude{0.0000});
    coordinates[2].lon = osrm::util::toFixed(osrm::util::UnsafeFloatLongitude{0.0020});
    coordinates[2].lat = osrm::util::toFixed(osrm::util::UnsafeFloatLatitude{0.0000});
    coordinates[3].lon = osrm::util::toFixed(osrm::util::UnsafeFloatLongitude{0.0030});
    coordinates[3].lat = osrm::util::toFixed(osrm::util::UnsafeFloatLatitude{0.0000});
    return coordinates;
}

template <typename LookupTableT> void sortLookupDescending(LookupTableT &lookup)
{
    std::sort(lookup.lookup.begin(),
              lookup.lookup.end(),
              [](const auto &lhs, const auto &rhs) { return rhs.first < lhs.first; });
}

} // namespace

BOOST_AUTO_TEST_CASE(read_hit_fcd_values_buckets_and_aggregates_samples)
{
    TemporaryFile file;
    {
        std::ofstream output(file.path);
        output << "Link_id\tLink_Direction\tTimestamp\tSpeed\tUniqueEntries\n";
        output << "4688548\t1\t2025-01-01 00:00:00.000\t11\t3\n";
        output << "4688548\t1\t2025-01-01 00:10:00.000\t13\t1\n";
        output << "4825719\t2\t2025-01-01 00:00:00.000\t47\t2\n";
    }

    auto lookup = osrm::updater::readHitFCDValues({file.path.string()}, 15, 1, 2);

    const auto forward =
        lookup({4688548, osrm::updater::HIT_FCD_DIRECTION_FORWARD, 192});
    BOOST_REQUIRE(forward);
    BOOST_CHECK_CLOSE(forward->speed_kmh, 11.5, 0.001);
    BOOST_CHECK_EQUAL(forward->sample_count, 4);

    const auto reverse =
        lookup({4825719, osrm::updater::HIT_FCD_DIRECTION_REVERSE, 192});
    BOOST_REQUIRE(reverse);
    BOOST_CHECK_CLOSE(reverse->speed_kmh, 47.0, 0.001);
    BOOST_CHECK_EQUAL(reverse->sample_count, 2);
}

BOOST_AUTO_TEST_CASE(build_temporal_lookup_from_hit_fcd_maps_directions)
{
    auto segment_data = makeSegmentData();
    auto osm_node_ids = makeOSMNodeIDs();
    const auto coordinates = makeCoordinates();
    osrm::extractor::LinkMapStorage link_map;
    link_map.way_ids = {{100, osrm::extractor::LINK_MAP_DIRECTION_FORWARD},
                        {100, osrm::extractor::LINK_MAP_DIRECTION_REVERSE}};
    link_map.offsets = {0, 1, 2};
    link_map.geometry_ids = {0, 0};
    link_map.geometry_directions = {osrm::extractor::LINK_MAP_DIRECTION_FORWARD,
                                    osrm::extractor::LINK_MAP_DIRECTION_REVERSE};
    link_map.segment_begins = {0, 0};
    link_map.segment_ends = {3, 3};

    osrm::updater::HitFCDLookupTable hit_lookup;
    hit_lookup.lookup = {
        {{100, osrm::updater::HIT_FCD_DIRECTION_FORWARD, 10}, {36.0, 2, 1}},
        {{100, osrm::updater::HIT_FCD_DIRECTION_REVERSE, 11}, {18.0, 1, 1}}};
    sortLookupDescending(hit_lookup);

    auto temporal_lookup = osrm::updater::buildTemporalLookupFromHitFCD(
        hit_lookup, link_map, segment_data, coordinates, osm_node_ids);

    const auto forward =
        temporal_lookup({10, 11, osrm::updater::TEMPORAL_DIRECTION_FORWARD, 10});
    BOOST_REQUIRE(forward);

    const auto reverse =
        temporal_lookup({10, 11, osrm::updater::TEMPORAL_DIRECTION_REVERSE, 11});
    BOOST_REQUIRE(reverse);

    const auto expected_length = osrm::util::coordinate_calculation::greatCircleDistance(
        coordinates[0], coordinates[1]);
    BOOST_CHECK_EQUAL(forward->duration_ds, static_cast<std::int32_t>(std::llround(expected_length)));
    BOOST_CHECK_EQUAL(reverse->duration_ds,
                      static_cast<std::int32_t>(std::llround(expected_length * 2.0)));
    BOOST_CHECK_GT(reverse->duration_ds, forward->duration_ds);
}

BOOST_AUTO_TEST_SUITE_END()
