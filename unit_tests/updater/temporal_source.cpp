#include "updater/temporal_source.hpp"

#include "../common/temporary_file.hpp"

#include <boost/test/unit_test.hpp>

#include <fstream>

BOOST_AUTO_TEST_SUITE(temporal_source)

using namespace osrm::updater;

BOOST_AUTO_TEST_CASE(read_temporal_values_supports_direction_tokens_and_precedence)
{
    TemporaryFile first_file;
    TemporaryFile second_file;

    {
        std::ofstream output(first_file.path);
        output << "1,2,fwd,0,100\n";
        output << "1,2,rev,0,130\n";
    }

    {
        std::ofstream output(second_file.path);
        output << "1,2,0,0,90\n";
        output << "1,2,1,1,140\n";
    }

    const auto lookup =
        readTemporalValues({first_file.path.string(), second_file.path.string()});

    const auto forward = lookup({1, 2, TEMPORAL_DIRECTION_FORWARD, 0});
    BOOST_REQUIRE(forward);
    BOOST_CHECK_EQUAL(forward->duration_ds, 90);
    BOOST_CHECK_EQUAL(forward->source, 2);

    const auto reverse = lookup({1, 2, TEMPORAL_DIRECTION_REVERSE, 0});
    BOOST_REQUIRE(reverse);
    BOOST_CHECK_EQUAL(reverse->duration_ds, 130);
    BOOST_CHECK_EQUAL(reverse->source, 1);

    const auto reverse_bucket_1 = lookup({1, 2, TEMPORAL_DIRECTION_REVERSE, 1});
    BOOST_REQUIRE(reverse_bucket_1);
    BOOST_CHECK_EQUAL(reverse_bucket_1->duration_ds, 140);
}

BOOST_AUTO_TEST_SUITE_END()
