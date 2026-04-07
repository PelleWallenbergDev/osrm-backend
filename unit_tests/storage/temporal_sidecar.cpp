#include "customizer/temporal_files.hpp"
#include "storage/storage.hpp"
#include "storage/view_factory.hpp"

#include "../common/temporary_file.hpp"

#include <boost/test/unit_test.hpp>

#include <cstdint>
#include <memory>

BOOST_AUTO_TEST_SUITE(temporal_sidecar)

using namespace osrm;

BOOST_AUTO_TEST_CASE(read_write_temporal_sidecar_through_views)
{
    TemporaryFile index_file;
    TemporaryFile profiles_file;
    TemporaryFile meta_file;

    customizer::TemporalProfileIndex reference_index;
    reference_index.forward_profile_ids = {0, customizer::INVALID_TEMPORAL_PROFILE_ID};
    reference_index.reverse_profile_ids = {customizer::INVALID_TEMPORAL_PROFILE_ID, 1};

    customizer::TemporalProfileStorage reference_storage;
    reference_storage.meta.bucket_size_minutes = 15;
    reference_storage.meta.week_bucket_count = 672;
    reference_storage.meta.encoding_version = customizer::TEMPORAL_PROFILE_ENCODING_VERSION_DENSE;
    reference_storage.profile_offsets = {0, 2};
    reference_storage.profile_sizes = {2, 2};
    reference_storage.freeflow_speeds = {0, 0};
    reference_storage.constrained_speeds = {0, 0};
    reference_storage.values = {10, 11, 20, 21};

    customizer::files::writeTemporalProfileIndex(index_file.path, reference_index);
    customizer::files::writeTemporalProfiles(profiles_file.path, reference_storage);
    customizer::files::writeTemporalMeta(meta_file.path, reference_storage.meta);

    auto layout = std::make_unique<storage::ContiguousDataLayout>();
    storage::populateLayoutFromFile(index_file.path, *layout);
    storage::populateLayoutFromFile(profiles_file.path, *layout);
    storage::populateLayoutFromFile(meta_file.path, *layout);

    auto memory = std::make_unique<char[]>(layout->GetSizeOfLayout());
    std::vector<storage::SharedDataIndex::AllocatedRegion> regions;
    regions.push_back({memory.get(), std::move(layout)});
    storage::SharedDataIndex index(std::move(regions));

    auto index_view =
        storage::make_temporal_profile_index_view(index, "/common/temporal_profile_index");
    customizer::files::readTemporalProfileIndex(index_file.path, index_view);

    auto storage_view = storage::make_temporal_profile_storage_view(index, "/common/temporal_profiles");
    customizer::files::readTemporalProfiles(profiles_file.path, storage_view);
    customizer::files::readTemporalMeta(meta_file.path, storage_view);

    BOOST_CHECK(index_view.HasForwardProfile(0));
    BOOST_CHECK(!index_view.HasForwardProfile(1));
    BOOST_CHECK(index_view.HasReverseProfile(1));
    BOOST_CHECK_EQUAL(storage_view.GetBucketSizeMinutes(), 15);
    BOOST_CHECK_EQUAL(storage_view.GetWeekBucketCount(), 672);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(storage_view.GetDuration(0, 1)), 11);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(storage_view.GetDuration(1, 0)), 20);
}

BOOST_AUTO_TEST_SUITE_END()
