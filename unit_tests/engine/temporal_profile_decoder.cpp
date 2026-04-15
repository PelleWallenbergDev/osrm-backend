#include "engine/temporal/temporal_profile_decoder.hpp"

#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

BOOST_AUTO_TEST_SUITE(temporal_profile_decoder)

namespace
{
float MaxAbsError(const std::vector<EdgeDuration::value_type> &lhs,
                  const std::vector<EdgeDuration::value_type> &rhs)
{
    float error = 0.0F;
    for (std::size_t index = 0; index < lhs.size(); ++index)
    {
        error = std::max(error, std::abs(static_cast<float>(lhs[index] - rhs[index])));
    }
    return error;
}

float MeanAbsError(const std::vector<EdgeDuration::value_type> &lhs,
                   const std::vector<EdgeDuration::value_type> &rhs)
{
    float error = 0.0F;
    for (std::size_t index = 0; index < lhs.size(); ++index)
    {
        error += std::abs(static_cast<float>(lhs[index] - rhs[index]));
    }
    return error / static_cast<float>(lhs.size());
}
} // namespace

BOOST_AUTO_TEST_CASE(lossy_round_trip_smooth_profile_stays_within_tolerance)
{
    constexpr std::uint32_t bucket_count = 672;
    constexpr std::uint32_t coeff_count = 200;

    std::vector<EdgeDuration::value_type> profile(bucket_count, 0);
    for (std::uint32_t bucket = 0; bucket < bucket_count; ++bucket)
    {
        const auto t = static_cast<float>(bucket) / static_cast<float>(bucket_count);
        profile[bucket] = static_cast<EdgeDuration::value_type>(
            std::lround(120.0F - 25.0F * std::sin(2.0F * 3.14159F * 7.0F * t) -
                        15.0F * std::sin(2.0F * 3.14159F * 14.0F * t)));
    }

    const auto compressed =
        osrm::engine::temporal::CompressTemporalProfile(profile, coeff_count);
    const auto reconstructed = osrm::engine::temporal::ExpandTemporalProfile(
        bucket_count, compressed.data(), static_cast<std::uint32_t>(compressed.size()), EdgeDuration{80});

    BOOST_CHECK_LE(MeanAbsError(profile, reconstructed), 1.0F);
    BOOST_CHECK_LE(MaxAbsError(profile, reconstructed), 2.0F);
}

BOOST_AUTO_TEST_CASE(constant_profile_round_trips_cleanly)
{
    constexpr std::uint32_t bucket_count = 672;
    constexpr std::uint32_t coeff_count = 200;

    std::vector<EdgeDuration::value_type> profile(bucket_count, 123);
    const auto compressed =
        osrm::engine::temporal::CompressTemporalProfile(profile, coeff_count);

    for (const auto bucket : {0U, 1U, 123U, 671U})
    {
        const auto reconstructed = osrm::engine::temporal::DecodeTemporalProfileBucket(
            bucket_count,
            compressed.data(),
            static_cast<std::uint32_t>(compressed.size()),
            bucket,
            EdgeDuration{123});
        BOOST_CHECK_SMALL(static_cast<double>(std::abs(
                              osrm::from_alias<std::int32_t>(reconstructed) - 123)),
                          1.1);
    }
}

BOOST_AUTO_TEST_CASE(single_bucket_decode_matches_expanded_profile)
{
    constexpr std::uint32_t bucket_count = 672;
    constexpr std::uint32_t coeff_count = 200;

    std::vector<EdgeDuration::value_type> profile(bucket_count, 0);
    for (std::uint32_t bucket = 0; bucket < bucket_count; ++bucket)
    {
        profile[bucket] = static_cast<EdgeDuration::value_type>(90 + (bucket % 17));
    }

    const auto compressed =
        osrm::engine::temporal::CompressTemporalProfile(profile, coeff_count);
    const auto expanded = osrm::engine::temporal::ExpandTemporalProfile(
        bucket_count, compressed.data(), static_cast<std::uint32_t>(compressed.size()), EdgeDuration{90});

    for (const auto bucket : {0U, 96U, 288U, 511U, 671U})
    {
        const auto decoded = osrm::engine::temporal::DecodeTemporalProfileBucket(
            bucket_count,
            compressed.data(),
            static_cast<std::uint32_t>(compressed.size()),
            bucket,
            EdgeDuration{90});
        BOOST_CHECK_EQUAL(osrm::from_alias<std::int32_t>(decoded), expanded[bucket]);
    }
}

BOOST_AUTO_TEST_CASE(decoded_bucket_respects_minimum_duration_floor)
{
    const std::array<osrm::engine::temporal::TemporalProfileCoefficient, 4> coefficients = {0, 0, 0, 0};

    const auto decoded = osrm::engine::temporal::DecodeTemporalProfileBucket(
        8, coefficients.data(), static_cast<std::uint32_t>(coefficients.size()), 3, EdgeDuration{15});

    BOOST_CHECK_EQUAL(osrm::from_alias<std::int32_t>(decoded), 15);
}

BOOST_AUTO_TEST_CASE(adaptive_compression_promotes_sharp_profiles_when_needed)
{
    constexpr std::uint32_t bucket_count = 672;
    constexpr std::uint32_t preferred_coeff_count = 200;

    std::vector<EdgeDuration::value_type> profile(bucket_count, 0);
    for (std::uint32_t bucket = 0; bucket < bucket_count; ++bucket)
    {
        if ((bucket / 8) % 2 == 0)
        {
            profile[bucket] = 45;
        }
        else
        {
            profile[bucket] = 240;
        }
    }

    const auto lossy = osrm::engine::temporal::CompressTemporalProfile(profile, preferred_coeff_count);
    const auto lossy_expanded = osrm::engine::temporal::ExpandTemporalProfile(
        bucket_count, lossy.data(), static_cast<std::uint32_t>(lossy.size()), EdgeDuration{45});
    const auto [lossy_mean_error, lossy_max_error] =
        osrm::engine::temporal::ComputeTemporalProfileError(profile, lossy_expanded);

    const auto adaptive =
        osrm::engine::temporal::CompressTemporalProfileAdaptive(profile, preferred_coeff_count);
    const auto adaptive_expanded = osrm::engine::temporal::ExpandTemporalProfile(
        bucket_count, adaptive.data(), static_cast<std::uint32_t>(adaptive.size()), EdgeDuration{45});
    const auto [adaptive_mean_error, adaptive_max_error] =
        osrm::engine::temporal::ComputeTemporalProfileError(profile, adaptive_expanded);

    BOOST_CHECK_GT(lossy_mean_error, 1.0F);
    BOOST_CHECK_GT(lossy_max_error, 2.0F);
    BOOST_CHECK_EQUAL(adaptive.size(), bucket_count);
    BOOST_CHECK_LE(adaptive_mean_error, 1.0F);
    BOOST_CHECK_LE(adaptive_max_error, 2.0F);
}

BOOST_AUTO_TEST_CASE(adaptive_compression_reports_when_fallback_coeff_count_is_used)
{
    constexpr std::uint32_t bucket_count = 672;
    constexpr std::uint32_t preferred_coeff_count = 200;

    std::vector<EdgeDuration::value_type> profile(bucket_count, 0);
    for (std::uint32_t bucket = 0; bucket < bucket_count; ++bucket)
    {
        profile[bucket] = ((bucket / 8) % 2 == 0) ? 45 : 240;
    }

    const auto adaptive = osrm::engine::temporal::CompressTemporalProfileAdaptiveWithStatus(
        profile, preferred_coeff_count);

    BOOST_CHECK(adaptive.used_fallback_coeff_count);
    BOOST_CHECK_EQUAL(adaptive.coefficients.size(), bucket_count);
}

BOOST_AUTO_TEST_CASE(fifo_detection_accepts_monotone_arrival_profiles)
{
    const std::vector<EdgeDuration::value_type> fifo_profile = {1500, 1400, 1600, 1500};
    BOOST_CHECK(osrm::engine::temporal::IsTemporalFunctionFIFO(fifo_profile, 5));
}

BOOST_AUTO_TEST_CASE(fifo_detection_rejects_profiles_with_backward_arrival_jumps)
{
    const std::vector<EdgeDuration::value_type> non_fifo_profile = {5000, 1, 5000, 1};
    BOOST_CHECK(!osrm::engine::temporal::IsTemporalFunctionFIFO(non_fifo_profile, 5));
}

BOOST_AUTO_TEST_SUITE_END()
