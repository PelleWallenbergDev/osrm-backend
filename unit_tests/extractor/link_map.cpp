#include "extractor/files.hpp"
#include "extractor/link_map.hpp"

#include "../common/temporary_file.hpp"

#include <boost/test/unit_test.hpp>

#include <algorithm>

BOOST_AUTO_TEST_SUITE(link_map)

namespace
{

osrm::extractor::SegmentDataContainer makeSegmentData()
{
    using SegmentData = osrm::extractor::SegmentDataContainer;

    SegmentData::SegmentWeightVector forward_weights{
        SegmentWeight{0},
        SegmentWeight{1},
        SegmentWeight{1},
        SegmentWeight{1},
        SegmentWeight{0},
        SegmentWeight{1},
        SegmentWeight{1}};
    SegmentData::SegmentWeightVector reverse_weights{
        SegmentWeight{0},
        SegmentWeight{1},
        SegmentWeight{1},
        SegmentWeight{1},
        SegmentWeight{0},
        SegmentWeight{1},
        SegmentWeight{1}};
    SegmentData::SegmentDurationVector forward_durations{
        SegmentDuration{0},
        SegmentDuration{1},
        SegmentDuration{1},
        SegmentDuration{1},
        SegmentDuration{0},
        SegmentDuration{1},
        SegmentDuration{1}};
    SegmentData::SegmentDurationVector reverse_durations{
        SegmentDuration{0},
        SegmentDuration{1},
        SegmentDuration{1},
        SegmentDuration{1},
        SegmentDuration{0},
        SegmentDuration{1},
        SegmentDuration{1}};

    return {{0, 4, 7},
            {0, 1, 2, 3, 3, 4, 5},
            std::move(forward_weights),
            std::move(reverse_weights),
            std::move(forward_durations),
            std::move(reverse_durations),
            {0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0}};
}

osrm::extractor::PackedOSMIDs makeOSMNodeIDs()
{
    osrm::extractor::PackedOSMIDs osm_node_ids;
    for (const auto node_id : {10ULL, 11ULL, 12ULL, 13ULL, 14ULL, 15ULL})
    {
        osm_node_ids.push_back(OSMNodeID{node_id});
    }
    return osm_node_ids;
}

osrm::extractor::WayNodeStorage makeWayNodeStorage()
{
    osrm::extractor::WayNodeStorage storage;
    storage.way_ids = {OSMWayID{300}};
    storage.node_offsets = {0, 6};
    storage.node_ids = {
        OSMNodeID{10},
        OSMNodeID{11},
        OSMNodeID{12},
        OSMNodeID{13},
        OSMNodeID{14},
        OSMNodeID{15}};
    return storage;
}

} // namespace

BOOST_AUTO_TEST_CASE(build_link_map_spans_geometries_and_preserves_direction)
{
    const auto link_map = osrm::extractor::BuildLinkMap(
        makeWayNodeStorage(), makeSegmentData(), makeOSMNodeIDs());

    const auto [forward_begin, forward_end] =
        link_map.FindRange(300, osrm::extractor::LINK_MAP_DIRECTION_FORWARD);
    BOOST_REQUIRE_EQUAL(forward_end - forward_begin, 2);
    BOOST_CHECK_EQUAL(link_map.geometry_ids[forward_begin], 0);
    BOOST_CHECK_EQUAL(link_map.geometry_directions[forward_begin],
                      osrm::extractor::LINK_MAP_DIRECTION_FORWARD);
    BOOST_CHECK_EQUAL(link_map.segment_begins[forward_begin], 0);
    BOOST_CHECK_EQUAL(link_map.segment_ends[forward_begin], 3);
    BOOST_CHECK_EQUAL(link_map.geometry_ids[forward_begin + 1], 1);
    BOOST_CHECK_EQUAL(link_map.geometry_directions[forward_begin + 1],
                      osrm::extractor::LINK_MAP_DIRECTION_FORWARD);
    BOOST_CHECK_EQUAL(link_map.segment_begins[forward_begin + 1], 0);
    BOOST_CHECK_EQUAL(link_map.segment_ends[forward_begin + 1], 2);

    const auto [reverse_begin, reverse_end] =
        link_map.FindRange(300, osrm::extractor::LINK_MAP_DIRECTION_REVERSE);
    BOOST_REQUIRE_EQUAL(reverse_end - reverse_begin, 2);
    BOOST_CHECK_EQUAL(link_map.geometry_ids[reverse_begin], 1);
    BOOST_CHECK_EQUAL(link_map.geometry_directions[reverse_begin],
                      osrm::extractor::LINK_MAP_DIRECTION_REVERSE);
    BOOST_CHECK_EQUAL(link_map.segment_begins[reverse_begin], 0);
    BOOST_CHECK_EQUAL(link_map.segment_ends[reverse_begin], 2);
    BOOST_CHECK_EQUAL(link_map.geometry_ids[reverse_begin + 1], 0);
    BOOST_CHECK_EQUAL(link_map.geometry_directions[reverse_begin + 1],
                      osrm::extractor::LINK_MAP_DIRECTION_REVERSE);
    BOOST_CHECK_EQUAL(link_map.segment_begins[reverse_begin + 1], 0);
    BOOST_CHECK_EQUAL(link_map.segment_ends[reverse_begin + 1], 3);
}

BOOST_AUTO_TEST_CASE(link_map_roundtrips_through_tar_sidecar)
{
    TemporaryFile link_map_file;

    const auto reference =
        osrm::extractor::BuildLinkMap(makeWayNodeStorage(), makeSegmentData(), makeOSMNodeIDs());
    osrm::extractor::files::writeLinkMap(link_map_file.path, reference);

    osrm::extractor::LinkMapStorage loaded;
    osrm::extractor::files::readLinkMap(link_map_file.path, loaded);

    BOOST_REQUIRE_EQUAL(reference.way_ids.size(), loaded.way_ids.size());
    for (auto index = std::size_t{0}; index < reference.way_ids.size(); ++index)
    {
        BOOST_CHECK_EQUAL(reference.way_ids[index].first, loaded.way_ids[index].first);
        BOOST_CHECK_EQUAL(reference.way_ids[index].second, loaded.way_ids[index].second);
    }
    BOOST_CHECK_EQUAL_COLLECTIONS(reference.offsets.begin(),
                                  reference.offsets.end(),
                                  loaded.offsets.begin(),
                                  loaded.offsets.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(reference.geometry_ids.begin(),
                                  reference.geometry_ids.end(),
                                  loaded.geometry_ids.begin(),
                                  loaded.geometry_ids.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(reference.geometry_directions.begin(),
                                  reference.geometry_directions.end(),
                                  loaded.geometry_directions.begin(),
                                  loaded.geometry_directions.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(reference.segment_begins.begin(),
                                  reference.segment_begins.end(),
                                  loaded.segment_begins.begin(),
                                  loaded.segment_begins.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(reference.segment_ends.begin(),
                                  reference.segment_ends.end(),
                                  loaded.segment_ends.begin(),
                                  loaded.segment_ends.end());
}

BOOST_AUTO_TEST_SUITE_END()
