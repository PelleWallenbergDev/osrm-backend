#include "customizer/temporal_cell_files.hpp"
#include "storage/storage.hpp"
#include "storage/view_factory.hpp"

#include "../common/temporary_file.hpp"

#include <boost/test/unit_test.hpp>

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

BOOST_AUTO_TEST_SUITE(temporal_cell_sidecar)

using namespace osrm;

BOOST_AUTO_TEST_CASE(read_write_dense_temporal_cell_sidecar_through_views)
{
    TemporaryFile metrics_file;
    TemporaryFile profiles_file;
    TemporaryFile meta_file;

    customizer::TemporalCellMetric reference_metric;
    reference_metric.function_ids = {0,
                                     customizer::INVALID_TEMPORAL_FUNCTION_ID,
                                     1,
                                     0};
    reference_metric.min_durations = {EdgeDuration{7},
                                      INVALID_EDGE_DURATION,
                                      EdgeDuration{20},
                                      EdgeDuration{7}};

    std::unordered_map<std::string, std::vector<customizer::TemporalCellMetric>> metrics = {
        {"duration", {reference_metric}}};

    customizer::TemporalFunctionStorage reference_storage;
    reference_storage.meta.bucket_size_minutes = 5;
    reference_storage.meta.week_bucket_count = 4;
    reference_storage.meta.encoding_version = customizer::TEMPORAL_FUNCTION_ENCODING_VERSION_DENSE;
    reference_storage.profile_offsets = {0, 4};
    reference_storage.profile_sizes = {4, 4};
    reference_storage.min_durations = {7, 20};
    reference_storage.freeflow_durations = {7, 20};
    reference_storage.profile_flags = {
        static_cast<customizer::TemporalFunctionFlags>(
            customizer::TEMPORAL_FUNCTION_FLAG_FIFO_VALID),
        static_cast<customizer::TemporalFunctionFlags>(
            customizer::TEMPORAL_FUNCTION_FLAG_FIFO_VALID |
            customizer::TEMPORAL_FUNCTION_FLAG_USED_FALLBACK_COEFF_COUNT)};
    reference_storage.values = {7, 8, 9, 10, 20, 21, 22, 23};

    customizer::files::writeTemporalCellMetrics(metrics_file.path, metrics);
    customizer::files::writeTemporalCellStorage(profiles_file.path, reference_storage);
    customizer::files::writeTemporalCellMeta(meta_file.path, reference_storage.meta);

    auto layout = std::make_unique<storage::ContiguousDataLayout>();
    storage::populateLayoutFromFile(metrics_file.path, *layout);
    storage::populateLayoutFromFile(profiles_file.path, *layout);
    storage::populateLayoutFromFile(meta_file.path, *layout);

    auto memory = std::make_unique<char[]>(layout->GetSizeOfLayout());
    std::vector<storage::SharedDataIndex::AllocatedRegion> regions;
    regions.push_back({memory.get(), std::move(layout)});
    storage::SharedDataIndex index(std::move(regions));

    auto metric_views =
        storage::make_temporal_cell_metric_view(index, "/mld/temporal_metrics/duration");
    std::unordered_map<std::string, std::vector<customizer::TemporalCellMetricView>> metric_targets =
        {{"duration", std::move(metric_views)}};
    customizer::files::readTemporalCellMetrics(metrics_file.path, metric_targets);

    auto storage_view =
        storage::make_temporal_function_storage_view(index, "/mld/temporal_metric_storage");
    customizer::files::readTemporalCellStorage(profiles_file.path, storage_view);
    customizer::files::readTemporalCellMeta(meta_file.path, storage_view);

    BOOST_REQUIRE_EQUAL(metric_targets.at("duration").size(), 1);
    const auto &metric_view = metric_targets.at("duration").front();
    BOOST_CHECK_EQUAL(metric_view.function_ids.size(), 4);
    BOOST_CHECK_EQUAL(metric_view.function_ids[0], 0U);
    BOOST_CHECK_EQUAL(metric_view.function_ids[1], customizer::INVALID_TEMPORAL_FUNCTION_ID);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(metric_view.min_durations[2]), 20);

    BOOST_CHECK_EQUAL(storage_view.GetBucketSizeMinutes(), 5U);
    BOOST_CHECK_EQUAL(storage_view.GetWeekBucketCount(), 4U);
    BOOST_CHECK_EQUAL(storage_view.GetEncodingVersion(),
                      customizer::TEMPORAL_FUNCTION_ENCODING_VERSION_DENSE);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(storage_view.GetDuration(0, 2)), 9);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(storage_view.GetDuration(1, 3)), 23);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(storage_view.GetMinDuration(0)), 7);
    BOOST_CHECK_EQUAL(from_alias<std::int32_t>(storage_view.GetFreeFlowDuration(1)), 20);
    BOOST_CHECK(storage_view.IsFIFOValid(0));
    BOOST_CHECK(storage_view.UsedFallbackCoeffCount(1));
    BOOST_CHECK_EQUAL(storage_view.GetFlags(1),
                      static_cast<customizer::TemporalFunctionFlags>(
                          customizer::TEMPORAL_FUNCTION_FLAG_FIFO_VALID |
                          customizer::TEMPORAL_FUNCTION_FLAG_USED_FALLBACK_COEFF_COUNT));
}

BOOST_AUTO_TEST_SUITE_END()
