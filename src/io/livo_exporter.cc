#include "livo_atlas/io/livo_exporter.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Geometry>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <ostream>
#include <set>
#include <sstream>
#include <stdexcept>

#include "livo_atlas/camera/camera_config.h"
#include "livo_atlas/common/so3_math.h"

namespace livo_atlas {
namespace {

namespace fs = std::filesystem;

struct TextFile {
  fs::path path;
  std::ofstream stream;
};

void Check(const TextFile& file) {
  if (!file.stream) {
    throw std::runtime_error(file.path.string() + ": output stream failed");
  }
}

void Open(const fs::path& path, TextFile* file) {
  file->path = path;
  try {
    fs::create_directories(path.parent_path());
    file->stream.open(path);
    Check(*file);
  } catch (const std::exception& error) {
    throw std::runtime_error(path.string() + ": " + error.what());
  }
}

void Close(TextFile* file) {
  if (file->stream.is_open()) {
    file->stream.close();
    Check(*file);
  }
}

void WriteStateRow(std::ostream& stream, double time,
                   const FilterState& state) {
  const Eigen::Vector3d euler = RotationMatrixToEuler(state.rot_end);
  stream << std::setw(20) << time << " " << euler.transpose() * 57.3 << " "
         << state.pos_end.transpose() << " " << state.vel_end.transpose() << " "
         << state.bias_g.transpose() << " " << state.bias_a.transpose() << " "
         << Eigen::Vector3d(state.inv_expo_time, 0, 0).transpose();
}

void WritePose(std::ostream& stream, const FrontendResult& result,
               bool trailing_space = false) {
  const auto& state = result.estimate;
  const Eigen::Quaterniond orientation(state.rot_end);
  stream << std::fixed << std::setprecision(6) << result.timestamp << " "
         << state.pos_end[0] << " " << state.pos_end[1] << " "
         << state.pos_end[2] << " " << orientation.x() << " " << orientation.y()
         << " " << orientation.z() << " " << orientation.w();
  if (trailing_space) stream << " ";
  stream << std::endl;
}

std::string Timestamp(double time) {
  std::ostringstream text;
  text << std::fixed << std::setprecision(6) << time;
  return text.str();
}

void WriteImage(const fs::path& path, const cv::Mat& image) {
  try {
    if (!cv::imwrite(path.string(), image)) {
      throw std::runtime_error("Cannot write image");
    }
  } catch (const std::exception& error) {
    throw std::runtime_error(path.string() + ": " + error.what());
  }
}

template <typename Cloud>
void WriteCloud(const fs::path& path, const Cloud& cloud) {
  try {
    pcl::PCDWriter writer;
    if (writer.writeBinary(path.string(), cloud) != 0) {
      throw std::runtime_error("Cannot write PCD");
    }
  } catch (const std::exception& error) {
    throw std::runtime_error(path.string() + ": " + error.what());
  }
}

}  // namespace

void ValidateExportOptions(const ExportOptions& options) {
  if (options.directory.empty()) {
    throw std::invalid_argument("output/directory must not be empty");
  }
  const auto& sequence = options.sequence_name;
  if (sequence.empty() || sequence == "." || sequence == ".." ||
      sequence.find_first_of("/\\") != std::string::npos) {
    throw std::invalid_argument("evo/seq_name must be a single filename");
  }
  if (options.images && options.image_interval <= 0) {
    throw std::invalid_argument("image_save/interval must be positive");
  }
  if (options.pcd) {
    if (options.pcd_interval == 0) {
      throw std::invalid_argument("pcd_save/interval must not be zero");
    }
    if (options.pcd_type != 0 && options.pcd_type != 1) {
      throw std::invalid_argument("pcd_save/type must be 0 or 1");
    }
    if (!std::isfinite(options.voxel_size) || options.voxel_size <= 0) {
      throw std::invalid_argument("pcd_save/filter_size_pcd must be positive");
    }
  }
  if (options.colmap) {
    if (options.calibration.camera.model != "Pinhole") {
      throw std::invalid_argument(
          "pcd_save/colmap_output_en requires a Pinhole camera");
    }
    const std::string error = ValidateCameraConfig(options.calibration.camera);
    if (!error.empty()) {
      throw std::invalid_argument("COLMAP camera: " + error);
    }
  }
}

class LivoExporter::Impl {
 public:
  explicit Impl(const ExportOptions& options) : options_(options) {
    ValidateExportOptions(options_);
    Open(Path("mat_pre.txt"), &pre_);
    Open(Path("mat_out.txt"), &out_);
    if (options_.trajectory) {
      Open(Path("result") / (options_.sequence_name + ".txt"), &trajectory_);
    }
    if (options_.pcd) Open(Path("pcd/lidar_poses.txt"), &lidar_poses_);
    if (options_.images) Open(Path("image/image_poses.txt"), &image_poses_);
    if (options_.colmap) InitializeColmap();
  }

  fs::path Path(const fs::path& relative) const {
    return options_.directory / relative;
  }

  void InitializeColmap() {
    if (options_.pcd) {
      // Preserve the original COLMAP script's reconstruction reset.
      fs::remove_all(Path("Colmap/images"));
      fs::remove_all(Path("Colmap/sparse/0"));
    }
    fs::create_directories(Path("Colmap/images"));
    Open(Path("Colmap/sparse/0/points3D.txt"), &points_);
    Open(Path("Colmap/sparse/0/images.txt"), &colmap_poses_);
    colmap_poses_.stream
        << "# Image list with two lines of data per image:\n"
        << "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, "
           "NAME\n"
        << "#   POINTS2D[] as (X, Y, POINT3D_ID)\n";
    Check(colmap_poses_);
    TextFile camera;
    Open(Path("Colmap/sparse/0/cameras.txt"), &camera);
    const auto& config = options_.calibration.camera;
    camera.stream << "# Camera list with one line of data per camera:\n"
                  << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n"
                  << "1 PINHOLE "
                  << static_cast<int>(config.width * config.scale) << " "
                  << static_cast<int>(config.height * config.scale) << " "
                  << std::fixed << std::setprecision(6)
                  << config.fx * config.scale << " " << config.fy * config.scale
                  << " " << config.cx * config.scale << " "
                  << config.cy * config.scale << std::endl;
    Close(&camera);
  }

  void Write(const FrontendResult& result, const PreparedOutput& prepared) {
    if (closed_) throw std::logic_error("Output writer is closed");
    try {
      WriteStateRow(pre_.stream, result.relative_time, result.prediction);
      pre_.stream << std::endl;
      Check(pre_);
      if (result.imu_initialized && !imu_.stream.is_open()) {
        Open(Path("imu.txt"), &imu_);
      }
      for (const auto& diagnostic : result.imu_diagnostics) {
        imu_.stream << std::setw(10) << diagnostic.relative_time << " "
                    << diagnostic.angular_velocity.transpose() << " "
                    << diagnostic.acceleration.transpose() << std::endl;
        Check(imu_);
      }
      if (!result.updated) return;
      if (result.kind != UpdateKind::kVisual && options_.trajectory) {
        WritePose(trajectory_.stream, result);
        Check(trajectory_);
      }
      if (result.kind == UpdateKind::kVisual && options_.colmap) {
        WriteColmap(result, prepared.colmap_image);
      }
      if (prepared.has_cloud) WriteProducts(result, prepared);
      for (const auto& debug : result.debug_images) {
        fs::create_directories(Path("ref_cur_combine"));
        WriteImage(Path("ref_cur_combine") / debug.name, debug.image);
      }
      WriteStateRow(out_.stream, result.relative_time, result.estimate);
      out_.stream << " " << result.undistorted_cloud->size() << std::endl;
      Check(out_);
    } catch (const std::exception& error) {
      closed_ = true;
      failed_ = true;
      throw std::runtime_error(options_.directory.string() + ": " +
                               error.what());
    }
  }

  void WriteColmap(const FrontendResult& result, const cv::Mat& image) {
    std::ostringstream name;
    name << std::setw(5) << std::setfill('0') << colmap_count_ << ".png";
    WriteImage(Path("Colmap/images") / name.str(), image);
    const Eigen::Quaterniond orientation(
        result.camera_from_world_rotation.toRotationMatrix());
    const auto& translation = result.camera_from_world_translation;
    colmap_poses_.stream << colmap_count_ << " " << std::fixed
                         << std::setprecision(6) << orientation.w() << " "
                         << orientation.x() << " " << orientation.y() << " "
                         << orientation.z() << " " << translation.x() << " "
                         << translation.y() << " " << translation.z() << " 1 "
                         << name.str() << std::endl
                         << "0.0 0.0 -1" << std::endl;
    Check(colmap_poses_);
    ++colmap_count_;
  }

  void WriteProducts(const FrontendResult& result,
                     const PreparedOutput& prepared) {
    const bool visual = result.kind == UpdateKind::kVisual;
    const std::string name = Timestamp(result.timestamp);
    if (options_.pcd) {
      if (options_.pcd_type == 0) {
        if (options_.mode == FrontendMode::kLidarInertialVisual) {
          if (prepared.colored_cloud) *colored_ += *prepared.colored_cloud;
        } else {
          plain_ += *result.world_cloud;
        }
        if (!visual) ++scan_count_;
      } else if (!visual) {
        PointCloud body(result.undistorted_cloud->size(), 1);
        const auto& extrinsics = options_.calibration.imu_from_lidar;
        for (std::size_t i = 0; i < body.size(); ++i) {
          const auto& point = result.undistorted_cloud->points[i];
          const Eigen::Vector3d position =
              extrinsics.rotation * Eigen::Vector3d(point.x, point.y, point.z) +
              extrinsics.translation;
          body[i].x = position.x();
          body[i].y = position.y();
          body[i].z = position.z();
          body[i].intensity = point.intensity;
        }
        plain_ += body;
        ++scan_count_;
      }
      const int interval = options_.pcd_type == 1 ? 1 : options_.pcd_interval;
      if ((!colored_->empty() || !plain_.empty()) && interval > 0 &&
          scan_count_ >= interval) {
        const auto path = Path("pcd") / (name + ".pcd");
        if (!colored_->empty()) {
          WriteCloud(path, *colored_);
          ColoredCloud().swap(*colored_);
        }
        if (!plain_.empty()) {
          WriteCloud(path, plain_);
          PointCloud().swap(plain_);
        }
        scan_count_ = 0;
      }
      if (!visual) {
        WritePose(lidar_poses_.stream, result, true);
        Check(lidar_poses_);
      }
    }
    if (options_.images && visual &&
        ++image_count_ >= options_.image_interval) {
      WriteImage(Path("image") / (name + ".png"), result.color_image);
      WritePose(image_poses_.stream, result);
      Check(image_poses_);
      image_count_ = 0;
    }
  }

  void Finish() {
    if (failed_) {
      throw std::runtime_error("Output writer previously failed: " +
                               options_.directory.string());
    }
    if (closed_) return;
    closed_ = true;
    try {
      if (options_.pcd && options_.pcd_type == 0 && options_.pcd_interval < 0 &&
          (!colored_->empty() || !plain_.empty())) {
        WriteAccumulatedCloud();
      }
      for (auto* stream : {&pre_, &out_, &trajectory_, &imu_, &lidar_poses_,
                           &image_poses_, &points_, &colmap_poses_}) {
        Close(stream);
      }
    } catch (const std::exception& error) {
      failed_ = true;
      throw std::runtime_error(options_.directory.string() + ": " +
                               error.what());
    }
  }

  void WriteAccumulatedCloud() {
    const auto raw = Path("pcd/all_raw_points.pcd");
    if (options_.mode != FrontendMode::kLidarInertialVisual) {
      WriteCloud(raw, plain_);
      return;
    }
    ColoredCloud downsampled;
    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setInputCloud(colored_);
    filter.setLeafSize(options_.voxel_size, options_.voxel_size,
                       options_.voxel_size);
    filter.filter(downsampled);
    WriteCloud(raw, *colored_);
    WriteCloud(Path("pcd/all_downsampled_points.pcd"), downsampled);
    if (!options_.colmap) return;
    points_.stream << "# 3D point list with one line of data per point\n"
                   << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
    Check(points_);
    for (std::size_t i = 0; i < downsampled.size(); ++i) {
      const auto& point = downsampled[i];
      points_.stream << i << " " << std::fixed << std::setprecision(6)
                     << point.x << " " << point.y << " " << point.z << " "
                     << static_cast<int>(point.r) << " "
                     << static_cast<int>(point.g) << " "
                     << static_cast<int>(point.b) << " 0" << std::endl;
      Check(points_);
    }
  }

  ExportOptions options_;
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

LivoExporter::LivoExporter(const ExportOptions& options)
    : impl_(std::make_unique<Impl>(options)) {}
LivoExporter::~LivoExporter() = default;
void LivoExporter::Write(const FrontendResult& result,
                         const PreparedOutput& prepared) {
  impl_->Write(result, prepared);
}
void LivoExporter::Finish() { impl_->Finish(); }

}  // namespace livo_atlas
