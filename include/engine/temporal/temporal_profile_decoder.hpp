#ifndef OSRM_ENGINE_TEMPORAL_TEMPORAL_PROFILE_DECODER_HPP
#define OSRM_ENGINE_TEMPORAL_TEMPORAL_PROFILE_DECODER_HPP

#include "util/typedefs.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

namespace osrm::engine::temporal
{

using TemporalProfileCoefficient = std::int32_t;

namespace detail
{
inline const std::vector<float> &
GetTemporalProfileCosineLUT(const std::uint32_t week_bucket_count,
                            const std::uint32_t coeff_count)
{
    using CacheKey = std::pair<std::uint32_t, std::uint32_t>;

    static std::mutex cache_mutex;
    static std::map<CacheKey, std::shared_ptr<std::vector<float>>> cache;

    const CacheKey key{week_bucket_count, coeff_count};

    std::lock_guard lock(cache_mutex);
    const auto cached = cache.find(key);
    if (cached != cache.end())
    {
        return *cached->second;
    }

    auto lut = std::make_shared<std::vector<float>>(
        static_cast<std::size_t>(week_bucket_count) * coeff_count);

    const auto pi_over_bucket_count =
        static_cast<float>(std::acos(-1.0)) / static_cast<float>(week_bucket_count);

    auto *out = lut->data();
    for (std::uint32_t bucket = 0; bucket < week_bucket_count; ++bucket)
    {
        for (std::uint32_t k = 0; k < coeff_count; ++k)
        {
            *out++ = std::cos(pi_over_bucket_count * (bucket + 0.5F) * k);
        }
    }

    return *cache.emplace(key, std::move(lut)).first->second;
}

inline std::int32_t clampDecodedDuration(const long double decoded_duration,
                                         const EdgeDuration minimum_duration)
{
    const auto minimum_duration_raw =
        std::max<std::int64_t>(1, from_alias<std::int64_t>(minimum_duration));
    const auto maximum_duration_raw =
        static_cast<std::int64_t>(std::numeric_limits<EdgeDuration::value_type>::max()) - 1;

    return static_cast<std::int32_t>(std::clamp<std::int64_t>(
        std::llround(decoded_duration), minimum_duration_raw, maximum_duration_raw));
}
} // namespace detail

inline std::vector<TemporalProfileCoefficient>
CompressTemporalProfile(const std::vector<EdgeDuration::value_type> &profile,
                        std::uint32_t coeff_count)
{
    if (profile.empty() || coeff_count == 0)
    {
        return {};
    }

    coeff_count = std::min<std::uint32_t>(coeff_count, static_cast<std::uint32_t>(profile.size()));

    const auto &lut =
        detail::GetTemporalProfileCosineLUT(static_cast<std::uint32_t>(profile.size()), coeff_count);
    const auto normalization =
        std::sqrt(2.0L / static_cast<long double>(profile.size()));
    constexpr long double one_over_sqrt_two = 0.70710678118654752440L;

    std::vector<long double> coefficients(coeff_count, 0.0L);
    for (std::uint32_t bucket = 0; bucket < profile.size(); ++bucket)
    {
        const auto *row = lut.data() + static_cast<std::size_t>(bucket) * coeff_count;
        const auto duration = static_cast<long double>(profile[bucket]);

        for (std::uint32_t k = 0; k < coeff_count; ++k)
        {
            coefficients[k] += static_cast<long double>(row[k]) * duration;
        }
    }

    coefficients[0] *= one_over_sqrt_two;

    std::vector<TemporalProfileCoefficient> compressed(coeff_count, 0);
    for (std::uint32_t k = 0; k < coeff_count; ++k)
    {
        const auto quantized = std::llround(normalization * coefficients[k]);
        compressed[k] = static_cast<TemporalProfileCoefficient>(std::clamp<long long>(
            quantized,
            std::numeric_limits<TemporalProfileCoefficient>::min(),
            std::numeric_limits<TemporalProfileCoefficient>::max()));
    }

    return compressed;
}

inline std::vector<EdgeDuration::value_type>
ExpandTemporalProfile(const std::uint32_t week_bucket_count,
                      const TemporalProfileCoefficient *coefficients,
                      const std::uint32_t coeff_count,
                      const EdgeDuration minimum_duration = EdgeDuration{1});

inline std::pair<float, float>
ComputeTemporalProfileError(const std::vector<EdgeDuration::value_type> &reference_profile,
                            const std::vector<EdgeDuration::value_type> &decoded_profile)
{
    assert(reference_profile.size() == decoded_profile.size());

    float max_abs_error = 0.0F;
    float mean_abs_error = 0.0F;

    for (std::size_t bucket = 0; bucket < reference_profile.size(); ++bucket)
    {
        const auto error = std::abs(static_cast<float>(reference_profile[bucket] -
                                                      decoded_profile[bucket]));
        max_abs_error = std::max(max_abs_error, error);
        mean_abs_error += error;
    }

    mean_abs_error /= static_cast<float>(reference_profile.size());
    return {mean_abs_error, max_abs_error};
}

inline std::vector<TemporalProfileCoefficient>
CompressTemporalProfileAdaptive(const std::vector<EdgeDuration::value_type> &profile,
                                const std::uint32_t preferred_coeff_count,
                                const float max_mean_abs_error = 1.0F,
                                const float max_bucket_abs_error = 2.0F)
{
    if (profile.empty() || preferred_coeff_count == 0)
    {
        return {};
    }

    const auto minimum_duration =
        EdgeDuration{*std::min_element(profile.begin(), profile.end())};
    const auto initial_coeff_count =
        std::min<std::uint32_t>(preferred_coeff_count, static_cast<std::uint32_t>(profile.size()));

    const auto compressed = CompressTemporalProfile(profile, initial_coeff_count);
    if (initial_coeff_count == profile.size())
    {
        return compressed;
    }

    const auto expanded = ExpandTemporalProfile(static_cast<std::uint32_t>(profile.size()),
                                                compressed.data(),
                                                static_cast<std::uint32_t>(compressed.size()),
                                                minimum_duration);
    const auto [mean_abs_error, max_abs_error] =
        ComputeTemporalProfileError(profile, expanded);

    if (mean_abs_error <= max_mean_abs_error && max_abs_error <= max_bucket_abs_error)
    {
        return compressed;
    }

    return CompressTemporalProfile(profile, static_cast<std::uint32_t>(profile.size()));
}

inline EdgeDuration
DecodeTemporalProfileBucket(const std::uint32_t week_bucket_count,
                            const TemporalProfileCoefficient *coefficients,
                            const std::uint32_t coeff_count,
                            const std::uint32_t week_bucket,
                            const EdgeDuration minimum_duration = EdgeDuration{1})
{
    if (coefficients == nullptr || coeff_count == 0 || week_bucket_count == 0 ||
        week_bucket >= week_bucket_count)
    {
        return INVALID_EDGE_DURATION;
    }

    const auto &lut = detail::GetTemporalProfileCosineLUT(week_bucket_count, coeff_count);
    const auto *row = lut.data() + static_cast<std::size_t>(week_bucket) * coeff_count;
    const auto normalization = std::sqrt(2.0L / static_cast<long double>(week_bucket_count));
    constexpr long double one_over_sqrt_two = 0.70710678118654752440L;

    long double decoded_duration =
        static_cast<long double>(coefficients[0]) * one_over_sqrt_two;
    for (std::uint32_t k = 1; k < coeff_count; ++k)
    {
        decoded_duration += static_cast<long double>(coefficients[k]) *
                            static_cast<long double>(row[k]);
    }

    decoded_duration *= normalization;
    return EdgeDuration{
        detail::clampDecodedDuration(decoded_duration, minimum_duration)};
}

inline std::vector<EdgeDuration::value_type>
ExpandTemporalProfile(const std::uint32_t week_bucket_count,
                      const TemporalProfileCoefficient *coefficients,
                      const std::uint32_t coeff_count,
                      const EdgeDuration minimum_duration)
{
    std::vector<EdgeDuration::value_type> expanded;
    expanded.reserve(week_bucket_count);

    for (std::uint32_t week_bucket = 0; week_bucket < week_bucket_count; ++week_bucket)
    {
        const auto duration = DecodeTemporalProfileBucket(
            week_bucket_count, coefficients, coeff_count, week_bucket, minimum_duration);
        expanded.push_back(from_alias<EdgeDuration::value_type>(duration));
    }

    return expanded;
}

} // namespace osrm::engine::temporal

#endif
