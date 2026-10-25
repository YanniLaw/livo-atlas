#ifndef LIVO_ATLAS_MAPPING_CLOUD_COLORIZER_H_
#define LIVO_ATLAS_MAPPING_CLOUD_COLORIZER_H_

#include <memory>

#include "livo_atlas/camera/camera_config.h"
#include "livo_atlas/frontend/frontend_result.h"
#include "livo_atlas/sensor/point_types.h"

namespace livo_atlas {

struct CloudColorizerOptions {
  CameraConfig camera;
  FrontendMode mode = FrontendMode::kLidarInertialVisual;
  int scan_interval = 1;
  double minimum_color_distance = 0.01;
};

class CloudColorizer {
 public:
  explicit CloudColorizer(const CloudColorizerOptions& options);
  ~CloudColorizer();
  CloudColorizer(const CloudColorizer&) = delete;
  CloudColorizer& operator=(const CloudColorizer&) = delete;

  // Null means no effective input cloud. A non-null empty cloud preserves
  // the publication cadence while color observations accumulate.
  ColoredCloud::ConstPtr Process(const FrontendResult& result);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_MAPPING_CLOUD_COLORIZER_H_
