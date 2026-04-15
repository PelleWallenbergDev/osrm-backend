#ifndef OSRM_CUSTOMIZER_TEMPORAL_CELL_FILES_HPP
#define OSRM_CUSTOMIZER_TEMPORAL_CELL_FILES_HPP

#include "customizer/temporal_cell_serialization.hpp"
#include "customizer/temporal_files.hpp"
#include "storage/tar.hpp"

#include <filesystem>
#include <string>
#include <unordered_map>

namespace osrm::customizer::files
{

template <typename TemporalCellMetricT>
inline void readTemporalCellMetrics(
    const std::filesystem::path &path,
    std::unordered_map<std::string, std::vector<TemporalCellMetricT>> &metrics)
{
    static_assert(std::is_same<TemporalCellMetricView, TemporalCellMetricT>::value ||
                      std::is_same<TemporalCellMetric, TemporalCellMetricT>::value,
                  "");

    storage::tar::FileReader reader{path, storage::tar::FileReader::VerifyFingerprint};

    for (auto &[metric_name, metric_exclude_classes] : metrics)
    {
        const auto prefix = "/mld/temporal_metrics/" + metric_name + "/exclude";
        const auto num_exclude_classes = reader.ReadElementCount64(prefix);
        metric_exclude_classes.resize(num_exclude_classes);

        std::size_t id = 0;
        for (auto &metric : metric_exclude_classes)
        {
            serialization::read(reader, prefix + "/" + std::to_string(id++), metric);
        }
    }
}

template <typename TemporalCellMetricT>
inline void writeTemporalCellMetrics(
    const std::filesystem::path &path,
    const std::unordered_map<std::string, std::vector<TemporalCellMetricT>> &metrics)
{
    static_assert(std::is_same<TemporalCellMetricView, TemporalCellMetricT>::value ||
                      std::is_same<TemporalCellMetric, TemporalCellMetricT>::value,
                  "");

    storage::tar::FileWriter writer{path, storage::tar::FileWriter::GenerateFingerprint};

    for (const auto &[metric_name, metric_exclude_classes] : metrics)
    {
        const auto prefix = "/mld/temporal_metrics/" + metric_name + "/exclude";
        writer.WriteElementCount64(prefix, metric_exclude_classes.size());

        std::size_t id = 0;
        for (const auto &exclude_metric : metric_exclude_classes)
        {
            serialization::write(writer, prefix + "/" + std::to_string(id++), exclude_metric);
        }
    }
}

template <typename StorageT>
inline void writeTemporalCellStorage(const std::filesystem::path &path, const StorageT &storage)
{
    writeTemporalFunctionStorage(path, storage, "/mld/temporal_metric_storage");
}

template <typename StorageT>
inline void readTemporalCellStorage(const std::filesystem::path &path, StorageT &storage)
{
    readTemporalFunctionStorage(path, storage, "/mld/temporal_metric_storage");
}

inline void writeTemporalCellMeta(const std::filesystem::path &path,
                                  const TemporalFunctionMeta &meta)
{
    writeTemporalFunctionMeta(path, meta, "/mld/temporal_metric_storage");
}

inline void readTemporalCellMeta(const std::filesystem::path &path, TemporalFunctionMeta &meta)
{
    readTemporalFunctionMeta(path, meta, "/mld/temporal_metric_storage");
}

inline void readTemporalCellMeta(const std::filesystem::path &path, TemporalFunctionStorage &storage)
{
    readTemporalCellMeta(path, storage.meta);
}

inline void readTemporalCellMeta(const std::filesystem::path &path,
                                 TemporalFunctionStorageView &storage)
{
    readTemporalFunctionMeta(path, storage, "/mld/temporal_metric_storage");
}

} // namespace osrm::customizer::files

#endif
