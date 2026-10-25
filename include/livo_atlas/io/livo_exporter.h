#ifndef LIVO_ATLAS_IO_LIVO_EXPORTER_H_
#define LIVO_ATLAS_IO_LIVO_EXPORTER_H_

#include <filesystem>
#include <memory>
#include <string>

#include "livo_atlas/frontend/calibration.h"
#include "livo_atlas/frontend/frontend_result.h"
#include "livo_atlas/sensor/point_types.h"

namespace livo_atlas {

struct ExportOptions {
  std::filesystem::path directory;
  std::string sequence_name = "01";
  FrontendMode mode = FrontendMode::kLidarInertialVisual;
  Calibration calibration;
  bool trajectory = false;
  bool pcd = false;
  bool images = false;
  bool colmap = false;
  int pcd_interval = -1;
  int image_interval = 1;
  int pcd_type = 0;
  double voxel_size = 0.5;
};

// Validates all options without touching the filesystem. Throws on error.
void ValidateExportOptions(const ExportOptions& options);

class LivoExporter {
 public:
  explicit LivoExporter(const ExportOptions& options);
  ~LivoExporter();
  LivoExporter(const LivoExporter&) = delete;
  LivoExporter& operator=(const LivoExporter&) = delete;

  void Write(const FrontendResult& result,
             const ColoredCloud::ConstPtr& colored_cloud);
  // Explicit completion reports I/O failures. Destruction only closes streams.
  void Finish();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_IO_LIVO_EXPORTER_H_
