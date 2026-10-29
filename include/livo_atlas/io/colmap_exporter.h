#ifndef LIVO_ATLAS_IO_COLMAP_EXPORTER_H_
#define LIVO_ATLAS_IO_COLMAP_EXPORTER_H_

#include <cstdint>
#include <filesystem>

namespace livo_atlas {
struct ColmapExportOptions {
  std::uint64_t pose_version = 0;
  std::uint32_t max_points_per_image = 2000;
};
struct ColmapExportSummary {
  std::uint64_t images = 0;
  std::uint64_t points = 0;
};

// Reads a stopped keyframe dataset and writes a new, independent export.
// Version zero uses odom; positive versions use their frozen map-frame prefix.
// A successful export has export.yml. Failures retain an incomplete directory
// without this marker; existing outputs and source assets are never replaced.
ColmapExportSummary ExportColmap(const std::filesystem::path& dataset,
                                 const std::filesystem::path& output,
                                 const ColmapExportOptions& options = {});
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_IO_COLMAP_EXPORTER_H_
