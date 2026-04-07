#ifndef OSRM_CUSTOMIZER_TEMPORAL_FILES_HPP
#define OSRM_CUSTOMIZER_TEMPORAL_FILES_HPP

#include "customizer/temporal_profiles.hpp"

#include "storage/serialization.hpp"
#include "storage/tar.hpp"

namespace osrm::customizer::files
{

inline void writeTemporalProfileIndex(const std::filesystem::path &path,
                                      const TemporalProfileIndex &index)
{
    storage::tar::FileWriter writer{path, storage::tar::FileWriter::GenerateFingerprint};

    storage::serialization::write(writer,
                                  "/common/temporal_profile_index/forward_profile_ids",
                                  index.forward_profile_ids);
    storage::serialization::write(writer,
                                  "/common/temporal_profile_index/reverse_profile_ids",
                                  index.reverse_profile_ids);
}

inline void readTemporalProfileIndex(const std::filesystem::path &path, TemporalProfileIndex &index)
{
    storage::tar::FileReader reader{path, storage::tar::FileReader::VerifyFingerprint};

    storage::serialization::read(reader,
                                 "/common/temporal_profile_index/forward_profile_ids",
                                 index.forward_profile_ids);
    storage::serialization::read(reader,
                                 "/common/temporal_profile_index/reverse_profile_ids",
                                 index.reverse_profile_ids);
}

inline void readTemporalProfileIndex(const std::filesystem::path &path,
                                     TemporalProfileIndexView &index)
{
    storage::tar::FileReader reader{path, storage::tar::FileReader::VerifyFingerprint};

    storage::serialization::read(reader,
                                 "/common/temporal_profile_index/forward_profile_ids",
                                 index.forward_profile_ids);
    storage::serialization::read(reader,
                                 "/common/temporal_profile_index/reverse_profile_ids",
                                 index.reverse_profile_ids);
}

inline void writeTemporalProfiles(const std::filesystem::path &path,
                                  const TemporalProfileStorage &profiles)
{
    storage::tar::FileWriter writer{path, storage::tar::FileWriter::GenerateFingerprint};

    storage::serialization::write(
        writer, "/common/temporal_profiles/profile_offsets", profiles.profile_offsets);
    storage::serialization::write(
        writer, "/common/temporal_profiles/profile_sizes", profiles.profile_sizes);
    storage::serialization::write(
        writer, "/common/temporal_profiles/freeflow_speeds", profiles.freeflow_speeds);
    storage::serialization::write(
        writer, "/common/temporal_profiles/constrained_speeds", profiles.constrained_speeds);
    storage::serialization::write(writer, "/common/temporal_profiles/values", profiles.values);
}

inline void readTemporalProfiles(const std::filesystem::path &path,
                                 TemporalProfileStorage &profiles)
{
    storage::tar::FileReader reader{path, storage::tar::FileReader::VerifyFingerprint};

    storage::serialization::read(reader,
                                 "/common/temporal_profiles/profile_offsets",
                                 profiles.profile_offsets);
    storage::serialization::read(
        reader, "/common/temporal_profiles/profile_sizes", profiles.profile_sizes);
    storage::serialization::read(
        reader, "/common/temporal_profiles/freeflow_speeds", profiles.freeflow_speeds);
    storage::serialization::read(
        reader, "/common/temporal_profiles/constrained_speeds", profiles.constrained_speeds);
    storage::serialization::read(reader, "/common/temporal_profiles/values", profiles.values);
}

inline void readTemporalProfiles(const std::filesystem::path &path,
                                 TemporalProfileStorageView &profiles)
{
    storage::tar::FileReader reader{path, storage::tar::FileReader::VerifyFingerprint};

    storage::serialization::read(reader,
                                 "/common/temporal_profiles/profile_offsets",
                                 profiles.profile_offsets);
    storage::serialization::read(
        reader, "/common/temporal_profiles/profile_sizes", profiles.profile_sizes);
    storage::serialization::read(
        reader, "/common/temporal_profiles/freeflow_speeds", profiles.freeflow_speeds);
    storage::serialization::read(
        reader, "/common/temporal_profiles/constrained_speeds", profiles.constrained_speeds);
    storage::serialization::read(reader, "/common/temporal_profiles/values", profiles.values);
}

inline void writeTemporalMeta(const std::filesystem::path &path, const TemporalProfileMeta &meta)
{
    storage::tar::FileWriter writer{path, storage::tar::FileWriter::GenerateFingerprint};

    writer.WriteElementCount64("/common/temporal_profiles/bucket_size_minutes", 1);
    writer.WriteFrom("/common/temporal_profiles/bucket_size_minutes", meta.bucket_size_minutes);

    writer.WriteElementCount64("/common/temporal_profiles/week_bucket_count", 1);
    writer.WriteFrom("/common/temporal_profiles/week_bucket_count", meta.week_bucket_count);

    writer.WriteElementCount64("/common/temporal_profiles/encoding_version", 1);
    writer.WriteFrom("/common/temporal_profiles/encoding_version", meta.encoding_version);
}

inline void readTemporalMeta(const std::filesystem::path &path, TemporalProfileMeta &meta)
{
    storage::tar::FileReader reader{path, storage::tar::FileReader::VerifyFingerprint};

    reader.ReadInto("/common/temporal_profiles/bucket_size_minutes", meta.bucket_size_minutes);
    reader.ReadInto("/common/temporal_profiles/week_bucket_count", meta.week_bucket_count);
    reader.ReadInto("/common/temporal_profiles/encoding_version", meta.encoding_version);
}

inline void readTemporalMeta(const std::filesystem::path &path, TemporalProfileStorage &profiles)
{
    readTemporalMeta(path, profiles.meta);
}

inline void readTemporalMeta(const std::filesystem::path &path,
                             TemporalProfileStorageView &profiles)
{
    storage::tar::FileReader reader{path, storage::tar::FileReader::VerifyFingerprint};

    reader.ReadInto("/common/temporal_profiles/bucket_size_minutes", *profiles.bucket_size_minutes);
    reader.ReadInto("/common/temporal_profiles/week_bucket_count", *profiles.week_bucket_count);
    reader.ReadInto("/common/temporal_profiles/encoding_version", *profiles.encoding_version);
}

} // namespace osrm::customizer::files

#endif
