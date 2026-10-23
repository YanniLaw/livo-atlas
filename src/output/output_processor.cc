#include "livo_atlas/output/output_processor.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "livo_atlas/camera/pinhole_camera.h"

#include "livo_atlas/camera/camera_factory.h"

namespace livo_atlas {
namespace {

Eigen::Vector3f SampleColor(const cv::Mat& image,
                          const Eigen::Vector2d& pixel) {
  const float u = pixel.x();
  const float v = pixel.y();
  const int x = std::floor(u);
  const int y = std::floor(v);
  const float sub_u = u - x;
  const float sub_v = v - y;
  const float top_left = (1.0 - sub_u) * (1.0 - sub_v);
  const float top_right = sub_u * (1.0 - sub_v);
  const float bottom_left = (1.0 - sub_u) * sub_v;
  const float bottom_right = sub_u * sub_v;
  const auto* top = image.ptr<unsigned char>(y) + x * 3;
  const auto* bottom = image.ptr<unsigned char>(y + 1) + x * 3;
  Eigen::Vector3f color;
  for (int channel = 0; channel < 3; ++channel) {
    color[channel] = top_left * top[channel] + top_right * top[channel + 3] +
                     bottom_left * bottom[channel] +
                     bottom_right * bottom[channel + 3];
  }
  return color;
}

}  // namespace

class OutputProcessor::Impl {
 public:
  explicit Impl(const OutputProcessorOptions& options) : options_(options) {
    if (options_.scan_interval <= 0) {
      throw std::invalid_argument("publish/pub_scan_num must be positive");
    }
    if (options_.colmap && options_.calibration.camera.model != "Pinhole") {
      throw std::invalid_argument("COLMAP output requires a Pinhole camera");
    }
    if (options_.mode == FrontendMode::kLidarInertialVisual ||
        options_.colmap) {
      camera_ = CreateCamera(options_.calibration.camera);
      if (!camera_) throw std::invalid_argument("Invalid output camera");
    }
  }

  PreparedOutput Process(const FrontendResult& result) {
    PreparedOutput output;
    if (!result.updated || !result.world_cloud || result.world_cloud->empty()) {
      return output;
    }
    output.has_cloud = true;
    ColoredCloud::Ptr colored(new ColoredCloud);
    ++scan_count_;
    if (result.kind == UpdateKind::kVisual) {
      if (result.color_image.type() != CV_8UC3 || result.color_image.empty() ||
          result.color_image.cols != camera_->width() ||
          result.color_image.rows != camera_->height()) {
        throw std::invalid_argument("Output image must match the BGR camera");
      }
      pending_ += *result.world_cloud;
      if (scan_count_ >= options_.scan_interval) {
        scan_count_ = 1;
        Colorize(result, colored.get());
      }
      if (options_.colmap) {
        auto* pinhole = static_cast<livo_atlas::PinholeCamera*>(camera_.get());
        pinhole->UndistortImage(result.color_image, &output.colmap_image);
      }
    }
    if (!colored->empty()) PointCloud().swap(pending_);
    output.colored_cloud = colored;
    return output;
  }

  void Colorize(const FrontendResult& result, ColoredCloud* colored) {
    colored->reserve(pending_.size());
    for (const auto& point : pending_.points) {
      const Eigen::Vector3d world(point.x, point.y, point.z);
      const Eigen::Vector3d local =
          result.camera_from_world_rotation * world +
          result.camera_from_world_translation;
      if (!local.allFinite() || local.z() <= 0) continue;
      const Eigen::Vector2d pixel = camera_->Project(local);
      if (!pixel.allFinite() || pixel.cwiseAbs().maxCoeff() >
                                    std::numeric_limits<int>::max()) {
        continue;
      }
      if (!camera_->IsInFrame(pixel.cast<int>(), 3)) continue;
      const Eigen::Vector3f color = SampleColor(result.color_image, pixel);
      pcl::PointXYZRGB output;
      output.x = point.x;
      output.y = point.y;
      output.z = point.z;
      output.r = color[2];
      output.g = color[1];
      output.b = color[0];
      if (local.norm() > options_.minimum_color_distance) {
        colored->push_back(output);
      }
    }
  }

  OutputProcessorOptions options_;
  std::unique_ptr<livo_atlas::Camera> camera_;
  PointCloud pending_;
  int scan_count_ = 1;
};

OutputProcessor::OutputProcessor(const OutputProcessorOptions& options)
    : impl_(std::make_unique<Impl>(options)) {}
OutputProcessor::~OutputProcessor() = default;
PreparedOutput OutputProcessor::Process(const FrontendResult& result) {
  return impl_->Process(result);
}

}  // namespace livo_atlas
