#include "updater/temporal_source.hpp"

#include "updater/csv_file_parser.hpp"

#include <boost/fusion/adapted/std_pair.hpp>
#include <boost/fusion/include/adapt_adt.hpp>
#include <boost/spirit/include/qi.hpp>

// clang-format off
BOOST_FUSION_ADAPT_STRUCT(osrm::updater::TemporalBucketKey,
                         (decltype(osrm::updater::TemporalBucketKey::from), from)
                         (decltype(osrm::updater::TemporalBucketKey::to), to)
                         (decltype(osrm::updater::TemporalBucketKey::direction), direction)
                         (decltype(osrm::updater::TemporalBucketKey::week_bucket), week_bucket))
BOOST_FUSION_ADAPT_STRUCT(osrm::updater::TemporalBucketValue,
                          (decltype(osrm::updater::TemporalBucketValue::duration_ds), duration_ds))
// clang-format on

namespace
{
namespace qi = boost::spirit::qi;
}

namespace osrm::updater
{

TemporalLookupTable readTemporalValues(const std::vector<std::string> &paths)
{
    qi::symbols<char, std::uint8_t> direction_parser;
    direction_parser.add("0", TEMPORAL_DIRECTION_FORWARD)("1", TEMPORAL_DIRECTION_REVERSE)(
        "fwd",
        TEMPORAL_DIRECTION_FORWARD)("forward", TEMPORAL_DIRECTION_FORWARD)(
        "rev",
        TEMPORAL_DIRECTION_REVERSE)("reverse", TEMPORAL_DIRECTION_REVERSE);

    CSVFilesParser<TemporalBucketKey, TemporalBucketValue> parser(
        1,
        qi::ulong_long >> ',' >> qi::ulong_long >> ',' >>
            qi::no_case[direction_parser] >> ',' >> qi::uint_,
        qi::int_);

    auto result = parser(paths);
    const auto found_inconsistency =
        std::find_if(std::begin(result.lookup),
                     std::end(result.lookup),
                     [](const auto &entry) { return entry.first.from == entry.first.to; });
    if (found_inconsistency != std::end(result.lookup))
    {
        util::Log(logWARNING) << "Empty temporal segment in CSV with node " +
                                     std::to_string(found_inconsistency->first.from);
    }

    return result;
}

} // namespace osrm::updater
