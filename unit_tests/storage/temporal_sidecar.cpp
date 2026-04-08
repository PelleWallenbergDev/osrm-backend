#include "customizer/temporal_files.hpp"
#include "engine/temporal/temporal_profile_decoder.hpp"
#include "storage/storage.hpp"
#include "storage/view_factory.hpp"

#include "../common/temporary_file.hpp"

#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

BOOST_AUTO_TEST_SUITE(temporal_sidecar)

using namespace osrm;

BOOST_AUTO_TEST_CASE(read_write_dense_temporal_sidecar_through_views)
{
    TemporaryFile index_file;
    TemporaryFile profiles_file;
    TemporaryFile meta_file;

    customizer::TemporalProfileIndex reference_index;
    reference_index.forward_profile_ids = {0, customizer::INVALID_TEMPORAL_PROFILE_ID};
    reference_index.reverse_profile_ids = {customizer::INVALID_TEMPORAL_PROFILE_ID, 1};

    customizer::TemporalProfileStorage reference_storage;
    reference_storage.meta.bucket_size_minutes = 15;
    reference_storage.meta.week_bucket_count = 4;
    reference_storage.meta.encoding_version = customizer::TEMPORAL_PROFILE_ENCODING_VERSION_DENSE;
    reference_storage.profile_offsets = {0, 4};
    reference_storage.profile_sizes = {4, 4};
    reference_storage.values = {10, 11, 12, 13, 20, 21, 22, 23};

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

    auto storage_view =
        storage::make_temporal_profile_storage_view(index, "/common/temporal_profiles");
    customizer::files::readTemporalProfiles(profiles_file.path, storage_view);
    customizer::files::readTemporalMeta(meta_file.path, storage_view);

    BOOST_CHECK(index_view.HasForwardProfile(0));
    BOOST_CHECK(!index_view.HasForwardProfile(1));
    BOOST_CHECK(index_view.HasReverseProfile(1));
    BOOST_CHECK_EQUAL(storage_view.GetBucketSizeMinutes(), 15);
    BOOST_CHECK_EQUAL(storage_view.GetWeekBucketCount(), 4);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(storage_view.GetDuration(0, 1)), 11);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(storage_view.GetDuration(1, 0)), 20);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(storage_view.GetMinDuration(0)), 10);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(storage_view.GetMinDuration(1)), 20);
}

BOOST_AUTO_TEST_CASE(read_write_dct_temporal_sidecar_through_views)
{
    TemporaryFile index_file;
    TemporaryFile profiles_file;
    TemporaryFile meta_file;

    const std::vector<customizer::TemporalProfileBucketValue> profile_a = {100, 110, 120, 110,
                                                                           100, 90,  80,  90};
    const std::vector<customizer::TemporalProfileBucketValue> profile_b = {40, 40, 40, 40,
                                                                           40, 40, 40, 40};
    const auto coeffs_a = engine::temporal::CompressTemporalProfile(
        profile_a, static_cast<std::uint32_t>(profile_a.size()));
    const auto coeffs_b = engine::temporal::CompressTemporalProfile(
        profile_b, static_cast<std::uint32_t>(profile_b.size()));

    customizer::TemporalProfileIndex reference_index;
    reference_index.forward_profile_ids = {0, customizer::INVALID_TEMPORAL_PROFILE_ID};
    reference_index.reverse_profile_ids = {customizer::INVALID_TEMPORAL_PROFILE_ID, 1};

    customizer::TemporalProfileStorage reference_storage;
    reference_storage.meta.bucket_size_minutes = 15;
    reference_storage.meta.week_bucket_count = 8;
    reference_storage.meta.encoding_version = customizer::TEMPORAL_PROFILE_ENCODING_VERSION_DCT;
    reference_storage.profile_offsets = {0, coeffs_a.size()};
    reference_storage.profile_sizes = {static_cast<std::uint32_t>(coeffs_a.size()),
                                       static_cast<std::uint32_t>(coeffs_b.size())};
    reference_storage.min_durations = {80, 40};
    reference_storage.freeflow_durations = {80, 40};
    reference_storage.coeffs = coeffs_a;
    reference_storage.coeffs.insert(
        reference_storage.coeffs.end(), coeffs_b.begin(), coeffs_b.end());

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

    auto storage_view =
        storage::make_temporal_profile_storage_view(index, "/common/temporal_profiles");
    customizer::files::readTemporalProfiles(profiles_file.path, storage_view);
    customizer::files::readTemporalMeta(meta_file.path, storage_view);

    BOOST_CHECK_EQUAL(storage_view.GetEncodingVersion(),
                      customizer::TEMPORAL_PROFILE_ENCODING_VERSION_DCT);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(storage_view.GetMinDuration(0)), 80);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(storage_view.GetMinDuration(1)), 40);

    for (const auto bucket : {0U, 2U, 5U, 7U})
    {
        const auto decoded_a = from_alias<std::int32_t>(storage_view.GetDuration(0, bucket));
        const auto decoded_b = from_alias<std::int32_t>(storage_view.GetDuration(1, bucket));
        BOOST_CHECK_SMALL(static_cast<double>(std::abs(decoded_a - profile_a[bucket])), 2.1);
        BOOST_CHECK_SMALL(static_cast<double>(std::abs(decoded_b - profile_b[bucket])), 1.1);
    }
}

BOOST_AUTO_TEST_SUITE_END()
