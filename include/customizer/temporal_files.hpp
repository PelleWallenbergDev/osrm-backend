#ifndef OSRM_CUSTOMIZER_TEMPORAL_FILES_HPP
#define OSRM_CUSTOMIZER_TEMPORAL_FILES_HPP

#include "customizer/temporal_profiles.hpp"

#include "storage/serialization.hpp"
#include "storage/tar.hpp"

#include <filesystem>
#include <iterator>
#include <string>
#include <unordered_set>
#include <vector>

namespace osrm::customizer::files
{
namespace detail
{
inline std::string makePath(const std::string &prefix, const std::string &leaf)
{
    return prefix + "/" + leaf;
}

inline std::unordered_set<std::string> listTarEntries(const std::filesystem::path &path)
{
    storage::tar::FileReader reader{path, storage::tar::FileReader::VerifyFingerprint};
    std::vector<storage::tar::FileReader::FileEntry> entries;
    reader.List(std::back_inserter(entries));

    std::unordered_set<std::string> names;
    names.reserve(entries.size());
    for (const auto &entry : entries)
    {
        names.insert(entry.name);
    }
    return names;
}

template <typename T>
inline void readIfPresent(const std::filesystem::path &path,
                          const std::unordered_set<std::string> &entries,
                          const std::string &name,
                          T &target)
{
    if (!entries.contains(name))
    {
        return;
    }

    storage::tar::FileReader reader{path, storage::tar::FileReader::VerifyFingerprint};
    storage::serialization::read(reader, name, target);
}
} // namespace detail

inline void writeTemporalFunctionIndex(const std::filesystem::path &path,
                                       const TemporalFunctionIndex &index,
                                       const std::string &prefix)
{
    storage::tar::FileWriter writer{path, storage::tar::FileWriter::GenerateFingerprint};

    storage::serialization::write(
        writer, detail::makePath(prefix, "forward_profile_ids"), index.forward_profile_ids);
    storage::serialization::write(
        writer, detail::makePath(prefix, "reverse_profile_ids"), index.reverse_profile_ids);
}

inline void readTemporalFunctionIndex(const std::filesystem::path &path,
                                      TemporalFunctionIndex &index,
                                      const std::string &prefix)
{
    storage::tar::FileReader reader{path, storage::tar::FileReader::VerifyFingerprint};

    storage::serialization::read(
        reader, detail::makePath(prefix, "forward_profile_ids"), index.forward_profile_ids);
    storage::serialization::read(
        reader, detail::makePath(prefix, "reverse_profile_ids"), index.reverse_profile_ids);
}

inline void readTemporalFunctionIndex(const std::filesystem::path &path,
                                      TemporalFunctionIndexView &index,
                                      const std::string &prefix)
{
    storage::tar::FileReader reader{path, storage::tar::FileReader::VerifyFingerprint};

    storage::serialization::read(
        reader, detail::makePath(prefix, "forward_profile_ids"), index.forward_profile_ids);
    storage::serialization::read(
        reader, detail::makePath(prefix, "reverse_profile_ids"), index.reverse_profile_ids);
}

template <typename StorageT>
inline void writeTemporalFunctionStorage(const std::filesystem::path &path,
                                         const StorageT &profiles,
                                         const std::string &prefix)
{
    storage::tar::FileWriter writer{path, storage::tar::FileWriter::GenerateFingerprint};

    storage::serialization::write(
        writer, detail::makePath(prefix, "profile_offsets"), profiles.profile_offsets);
    storage::serialization::write(
        writer, detail::makePath(prefix, "profile_sizes"), profiles.profile_sizes);

    if (!profiles.min_durations.empty())
    {
        storage::serialization::write(
            writer, detail::makePath(prefix, "min_durations"), profiles.min_durations);
    }

    if (!profiles.freeflow_durations.empty())
    {
        storage::serialization::write(
            writer, detail::makePath(prefix, "freeflow_durations"), profiles.freeflow_durations);
    }

    if (!profiles.profile_flags.empty())
    {
        storage::serialization::write(
            writer, detail::makePath(prefix, "profile_flags"), profiles.profile_flags);
    }

    if (!profiles.values.empty())
    {
        storage::serialization::write(writer, detail::makePath(prefix, "values"), profiles.values);
    }

    if (!profiles.coeffs.empty())
    {
        storage::serialization::write(writer, detail::makePath(prefix, "coeffs"), profiles.coeffs);
    }
}

template <typename StorageT>
inline void readTemporalFunctionStorage(const std::filesystem::path &path,
                                        StorageT &profiles,
                                        const std::string &prefix)
{
    storage::tar::FileReader reader{path, storage::tar::FileReader::VerifyFingerprint};

    storage::serialization::read(
        reader, detail::makePath(prefix, "profile_offsets"), profiles.profile_offsets);
    storage::serialization::read(
        reader, detail::makePath(prefix, "profile_sizes"), profiles.profile_sizes);

    const auto entries = detail::listTarEntries(path);
    detail::readIfPresent(
        path, entries, detail::makePath(prefix, "min_durations"), profiles.min_durations);
    detail::readIfPresent(
        path, entries, detail::makePath(prefix, "freeflow_durations"), profiles.freeflow_durations);
    detail::readIfPresent(
        path, entries, detail::makePath(prefix, "profile_flags"), profiles.profile_flags);
    detail::readIfPresent(path, entries, detail::makePath(prefix, "values"), profiles.values);
    detail::readIfPresent(path, entries, detail::makePath(prefix, "coeffs"), profiles.coeffs);
}

inline void writeTemporalFunctionMeta(const std::filesystem::path &path,
                                      const TemporalFunctionMeta &meta,
                                      const std::string &prefix)
{
    storage::tar::FileWriter writer{path, storage::tar::FileWriter::GenerateFingerprint};

    writer.WriteElementCount64(detail::makePath(prefix, "bucket_size_minutes"), 1);
    writer.WriteFrom(detail::makePath(prefix, "bucket_size_minutes"), meta.bucket_size_minutes);

    writer.WriteElementCount64(detail::makePath(prefix, "week_bucket_count"), 1);
    writer.WriteFrom(detail::makePath(prefix, "week_bucket_count"), meta.week_bucket_count);

    writer.WriteElementCount64(detail::makePath(prefix, "encoding_version"), 1);
    writer.WriteFrom(detail::makePath(prefix, "encoding_version"), meta.encoding_version);
}

inline void readTemporalFunctionMeta(const std::filesystem::path &path,
                                     TemporalFunctionMeta &meta,
                                     const std::string &prefix)
{
    storage::tar::FileReader reader{path, storage::tar::FileReader::VerifyFingerprint};

    reader.ReadInto(detail::makePath(prefix, "bucket_size_minutes"), meta.bucket_size_minutes);
    reader.ReadInto(detail::makePath(prefix, "week_bucket_count"), meta.week_bucket_count);
    reader.ReadInto(detail::makePath(prefix, "encoding_version"), meta.encoding_version);
}

template <typename StorageT>
inline void readTemporalFunctionMeta(const std::filesystem::path &path,
                                     StorageT &profiles,
                                     const std::string &prefix)
{
    storage::tar::FileReader reader{path, storage::tar::FileReader::VerifyFingerprint};

    reader.ReadInto(detail::makePath(prefix, "bucket_size_minutes"), *profiles.bucket_size_minutes);
    reader.ReadInto(detail::makePath(prefix, "week_bucket_count"), *profiles.week_bucket_count);
    reader.ReadInto(detail::makePath(prefix, "encoding_version"), *profiles.encoding_version);
}

inline void writeTemporalProfileIndex(const std::filesystem::path &path,
                                      const TemporalProfileIndex &index)
{
    writeTemporalFunctionIndex(path, index, "/common/temporal_profile_index");
}

inline void readTemporalProfileIndex(const std::filesystem::path &path, TemporalProfileIndex &index)
{
    readTemporalFunctionIndex(path, index, "/common/temporal_profile_index");
}

inline void readTemporalProfileIndex(const std::filesystem::path &path,
                                     TemporalProfileIndexView &index)
{
    readTemporalFunctionIndex(path, index, "/common/temporal_profile_index");
}

inline void writeTemporalProfiles(const std::filesystem::path &path,
                                  const TemporalProfileStorage &profiles)
{
    writeTemporalFunctionStorage(path, profiles, "/common/temporal_profiles");
}

inline void readTemporalProfiles(const std::filesystem::path &path,
                                 TemporalProfileStorage &profiles)
{
    readTemporalFunctionStorage(path, profiles, "/common/temporal_profiles");
}

inline void readTemporalProfiles(const std::filesystem::path &path,
                                 TemporalProfileStorageView &profiles)
{
    readTemporalFunctionStorage(path, profiles, "/common/temporal_profiles");
}

inline void writeTemporalMeta(const std::filesystem::path &path, const TemporalProfileMeta &meta)
{
    writeTemporalFunctionMeta(path, meta, "/common/temporal_profiles");
}

inline void readTemporalMeta(const std::filesystem::path &path, TemporalProfileMeta &meta)
{
    readTemporalFunctionMeta(path, meta, "/common/temporal_profiles");
}

inline void readTemporalMeta(const std::filesystem::path &path, TemporalProfileStorage &profiles)
{
    readTemporalMeta(path, profiles.meta);
}

inline void readTemporalMeta(const std::filesystem::path &path,
                             TemporalProfileStorageView &profiles)
{
    readTemporalFunctionMeta(path, profiles, "/common/temporal_profiles");
}

} // namespace osrm::customizer::files

#endif
