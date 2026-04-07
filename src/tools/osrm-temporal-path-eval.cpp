#include "extractor/files.hpp"
#include "engine/datafacade.hpp"
#include "engine/datafacade/process_memory_allocator.hpp"
#include "engine/temporal_traffic.hpp"
#include "osrm/exception.hpp"
#include "osrm/storage_config.hpp"
#include "util/log.hpp"
#include "util/meminfo.hpp"
#include "util/version.hpp"

#include <boost/program_options.hpp>

#include <algorithm>
#include <cctype>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

using namespace osrm;

namespace
{

enum class return_code : unsigned
{
    ok,
    fail,
    exit
};

std::string lowerCopy(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

std::optional<engine::temporal::TemporalGeometryStep> parseGeometryToken(const std::string &token)
{
    if (token.empty())
    {
        return std::nullopt;
    }

    const auto separator = token.find(':');
    const auto id_token = separator == std::string::npos ? token : token.substr(0, separator);
    const auto direction_token =
        separator == std::string::npos ? std::string{"fwd"} : lowerCopy(token.substr(separator + 1));

    auto geometry_id = static_cast<PackedGeometryID>(std::stoul(id_token));
    const auto forward = direction_token == "fwd" || direction_token == "forward" ||
                         direction_token == "0";
    const auto reverse = direction_token == "rev" || direction_token == "reverse" ||
                         direction_token == "1";

    if (!forward && !reverse)
    {
        throw util::exception("Invalid geometry direction '" + direction_token +
                              "'. Expected fwd/rev" + SOURCE_REF);
    }

    return engine::temporal::TemporalGeometryStep{geometry_id, forward};
}

std::vector<engine::temporal::TemporalGeometryStep>
parseGeometryPath(const std::vector<std::string> &tokens)
{
    std::vector<engine::temporal::TemporalGeometryStep> steps;

    for (const auto &token : tokens)
    {
        std::size_t start = 0;
        while (start < token.size())
        {
            const auto comma = token.find(',', start);
            const auto piece = token.substr(start, comma == std::string::npos ? comma : comma - start);
            if (const auto parsed = parseGeometryToken(piece))
            {
                steps.push_back(*parsed);
            }

            if (comma == std::string::npos)
            {
                break;
            }
            start = comma + 1;
        }
    }

    return steps;
}

return_code parseArguments(int argc,
                           char *argv[],
                           std::string &verbosity,
                           std::filesystem::path &base_path,
                           std::time_t &departure_timestamp,
                           std::vector<std::string> &geometry_path_tokens)
{
    boost::program_options::options_description generic_options("Options");
    generic_options.add_options()("version,v", "Show version")("help,h", "Show this help message")(
        "verbosity,l",
        boost::program_options::value<std::string>(&verbosity)->default_value("INFO"),
        std::string("Log verbosity level: " + util::LogPolicy::GetLevels()).c_str());

    boost::program_options::options_description config_options("Configuration");
    config_options.add_options()(
        "departure-timestamp",
        boost::program_options::value<std::time_t>(&departure_timestamp)->required(),
        "Departure timestamp as seconds since the Unix epoch (UTC)")(
        "geometry-path",
        boost::program_options::value<std::vector<std::string>>(&geometry_path_tokens)
            ->multitoken()
            ->required(),
        "Ordered geometry path as <packed_geometry_id>:<fwd|rev> tokens");

    boost::program_options::options_description hidden_options("Hidden options");
    hidden_options.add_options()(
        "input,i", boost::program_options::value<std::filesystem::path>(&base_path), "Input base path");

    boost::program_options::positional_options_description positional_options;
    positional_options.add("input", 1);

    boost::program_options::options_description cmdline_options;
    cmdline_options.add(generic_options).add(config_options).add(hidden_options);

    const auto *executable = argv[0];
    boost::program_options::options_description visible_options(
        std::filesystem::path(executable).filename().string() + " <input.osrm> [options]");
    visible_options.add(generic_options).add(config_options);

    boost::program_options::variables_map option_variables;
    try
    {
        boost::program_options::store(boost::program_options::command_line_parser(argc, argv)
                                          .options(cmdline_options)
                                          .positional(positional_options)
                                          .run(),
                                      option_variables);
    }
    catch (const boost::program_options::error &e)
    {
        util::Log(logERROR) << e.what();
        return return_code::fail;
    }

    if (option_variables.count("version") > 0)
    {
        std::cout << OSRM_VERSION << std::endl;
        return return_code::exit;
    }

    if (option_variables.count("help") > 0)
    {
        std::cout << visible_options;
        return return_code::exit;
    }

    try
    {
        boost::program_options::notify(option_variables);
    }
    catch (const boost::program_options::error &e)
    {
        util::Log(logERROR) << e.what();
        std::cout << visible_options;
        return return_code::fail;
    }

    if (option_variables.count("input") == 0)
    {
        std::cout << visible_options;
        return return_code::fail;
    }

    return return_code::ok;
}

} // namespace

int main(int argc, char *argv[])
try
{
    util::LogPolicy::GetInstance().Unmute();

    std::string verbosity;
    std::filesystem::path base_path;
    std::time_t departure_timestamp = 0;
    std::vector<std::string> geometry_path_tokens;

    const auto result =
        parseArguments(argc, argv, verbosity, base_path, departure_timestamp, geometry_path_tokens);

    if (return_code::fail == result)
    {
        return EXIT_FAILURE;
    }

    if (return_code::exit == result)
    {
        return EXIT_SUCCESS;
    }

    util::LogPolicy::GetInstance().SetLevel(verbosity);

    const auto steps = parseGeometryPath(geometry_path_tokens);
    if (steps.empty())
    {
        util::Log(logERROR) << "Geometry path must not be empty";
        return EXIT_FAILURE;
    }

    storage::StorageConfig storage_config(base_path);
    if (!storage_config.IsValid())
    {
        return EXIT_FAILURE;
    }

    extractor::ProfileProperties profile_properties;
    extractor::files::readProfileProperties(storage_config.GetPath(".osrm.properties"),
                                            profile_properties);

    auto allocator = std::make_shared<engine::datafacade::ProcessMemoryAllocator>(storage_config);
    engine::DataFacade<engine::datafacade::MLD> facade(
        allocator, profile_properties.GetWeightName(), 0);

    const auto evaluation =
        engine::temporal::EvaluateGeometryPath(facade, steps, departure_timestamp);

    std::cout << "departure_timestamp=" << departure_timestamp << "\n";
    std::cout << "arrival_timestamp=" << evaluation.arrival_timestamp << "\n";
    std::cout << "bucket_size_minutes=" << facade.GetTemporalBucketSizeMinutes() << "\n";
    std::cout << "week_bucket_count=" << facade.GetTemporalWeekBucketCount() << "\n";
    std::cout << "total_duration_ds=" << from_alias<std::int32_t>(evaluation.total_duration) << "\n";
    std::cout << "segments=" << evaluation.steps.size() << "\n";

    for (const auto &step : evaluation.steps)
    {
        std::cout << step.geometry_id << ":" << (step.forward ? "fwd" : "rev")
                  << " bucket=" << step.week_bucket
                  << " duration_ds=" << from_alias<std::int32_t>(step.duration)
                  << " source=" << (step.used_temporal ? "temporal" : "static") << "\n";
    }

    util::DumpMemoryStats();
    return EXIT_SUCCESS;
}
catch (const osrm::RuntimeError &e)
{
    util::DumpMemoryStats();
    util::Log(logERROR) << e.what();
    return e.GetCode();
}
catch (const util::exception &e)
{
    util::DumpMemoryStats();
    util::Log(logERROR) << e.what();
    return EXIT_FAILURE;
}
catch (const std::bad_alloc &e)
{
    util::DumpMemoryStats();
    util::Log(logERROR) << "[exception] " << e.what();
    util::Log(logERROR) << "Please provide more memory or consider using a larger swapfile";
    return EXIT_FAILURE;
}
#ifdef _WIN32
catch (const std::exception &e)
{
    util::Log(logERROR) << "[exception] " << e.what() << std::endl;
    return EXIT_FAILURE;
}
#endif
