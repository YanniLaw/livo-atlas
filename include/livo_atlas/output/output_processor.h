#ifndef LIVO_ATLAS_OUTPUT_OUTPUT_PROCESSOR_H_
#define LIVO_ATLAS_OUTPUT_OUTPUT_PROCESSOR_H_

#include <memory>
#include <opencv2/core/mat.hpp>

#include "livo_atlas/frontend/calibration.h"
#include "livo_atlas/frontend/frontend_result.h"

namespace livo_atlas {

using ColoredCloud = pcl::PointCloud<pcl::PointXYZRGB>;

struct OutputProcessorOptions {
  Calibration calibration;
  FrontendMode mode = FrontendMode::kLidarInertialVisual;
  int scan_interval = 1;
  double minimum_color_distance = 0.01;
  bool colmap = false;
};

struct PreparedOutput {
  bool has_cloud = false;
  ColoredCloud::ConstPtr colored_cloud;
  cv::Mat colmap_image;
};

class OutputProcessor {
 public:
  explicit OutputProcessor(const OutputProcessorOptions& options);
  ~OutputProcessor();
  OutputProcessor(const OutputProcessor&) = delete;
  OutputProcessor& operator=(const OutputProcessor&) = delete;

  PreparedOutput Process(const FrontendResult& result);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_OUTPUT_OUTPUT_PROCESSOR_H_
