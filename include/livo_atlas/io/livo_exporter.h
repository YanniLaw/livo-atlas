#ifndef LIVO_ATLAS_IO_LIVO_EXPORTER_H_
#define LIVO_ATLAS_IO_LIVO_EXPORTER_H_

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "livo_atlas/frontend/calibration.h"
#include "livo_atlas/frontend/frontend_result.h"
#include "livo_atlas/sensor/point_types.h"

namespace livo_atlas {

class Camera;

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
  // Keep the path with each stream so write and close errors identify it.
  struct TextFile {
    std::filesystem::path path;
    std::ofstream stream;
  };

  static void Check(const TextFile& file);
  static void Open(const std::filesystem::path& path, TextFile* file);
  static void Close(TextFile* file);

  std::filesystem::path Path(const std::filesystem::path& relative) const;
  void InitializeColmap();
  void WriteColmap(const FrontendResult& result, const cv::Mat& image);
  void WriteProducts(const FrontendResult& result,
                     const ColoredCloud::ConstPtr& colored_cloud);
  void WriteAccumulatedCloud();

  ExportOptions options_;
  std::unique_ptr<Camera> colmap_camera_;
  TextFile pre_;
  TextFile out_;
  TextFile trajectory_;
  TextFile imu_;
  TextFile lidar_poses_;
  TextFile image_poses_;
  TextFile points_;
  TextFile colmap_poses_;
  ColoredCloud::Ptr colored_{new ColoredCloud};
  PointCloud plain_;
  int scan_count_ = 0;
  int image_count_ = 0;
  int colmap_count_ = 1;
  bool closed_ = false;
  bool failed_ = false;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_IO_LIVO_EXPORTER_H_
