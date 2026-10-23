#include "livo_atlas/io/result_writer.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "livo_atlas/core/so3_math.h"

namespace livo_atlas {
namespace {

namespace fs = std::filesystem;

void Open(const fs::path& path, std::ofstream* stream) {
  try {
    fs::create_directories(path.parent_path());
    stream->exceptions(std::ios::failbit | std::ios::badbit);
    stream->open(path);
  } catch (const std::exception& error) {
    throw std::runtime_error(path.string() + ": " + error.what());
  }
}

void WriteStateRow(std::ostream& stream, double time, const State& state) {
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
         << state.pos_end[2] << " " << orientation.x() << " "
         << orientation.y() << " " << orientation.z() << " "
         << orientation.w();
  if (trailing_space) stream << " ";
  stream << std::endl;
}

std::string Timestamp(double time) {
  std::ostringstream text;
  text << std::fixed << std::setprecision(6) << time;
  return text.str();
}

void WriteImage(const fs::path& path, const cv::Mat& image) {
  if (!cv::imwrite(path.string(), image)) {
    throw std::runtime_error("Cannot write image: " + path.string());
  }
}

template <typename Cloud>
void WriteCloud(const fs::path& path, const Cloud& cloud) {
  pcl::PCDWriter writer;
  if (writer.writeBinary(path.string(), cloud) != 0) {
    throw std::runtime_error("Cannot write PCD: " + path.string());
  }
}

}  // namespace

void ValidateOutputOptions(const OutputOptions& options) {
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
  if (options.colmap && options.calibration.camera.model != "Pinhole") {
    throw std::invalid_argument(
        "pcd_save/colmap_output_en requires a Pinhole camera");
  }
}

class ResultWriter::Impl {
 public:
  explicit Impl(const OutputOptions& options) : options_(options) {
    ValidateOutputOptions(options_);
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
    fs::create_directories(Path("Colmap/images"));
    Open(Path("Colmap/sparse/0/points3D.txt"), &points_);
    Open(Path("Colmap/sparse/0/images.txt"), &colmap_poses_);
    colmap_poses_ << "# Image list with two lines of data per image:\n"
                  << "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, "
                     "NAME\n"
                  << "#   POINTS2D[] as (X, Y, POINT3D_ID)\n";
    std::ofstream camera;
    Open(Path("Colmap/sparse/0/cameras.txt"), &camera);
    const auto& config = options_.calibration.camera;
    camera << "# Camera list with one line of data per camera:\n"
           << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n"
           << "1 PINHOLE " << static_cast<int>(config.width * config.scale)
           << " " << static_cast<int>(config.height * config.scale) << " "
           << std::fixed << std::setprecision(6) << config.fx * config.scale
           << " " << config.fy * config.scale << " " << config.cx * config.scale
           << " " << config.cy * config.scale << std::endl;
    camera.close();
  }

  void Write(const FrontendResult& result, const PreparedOutput& prepared) {
    if (closed_) throw std::logic_error("Output writer is closed");
    try {
      WriteStateRow(pre_, result.relative_time, result.prediction);
      pre_ << std::endl;
      if (result.imu_initialized && !imu_.is_open()) {
        Open(Path("imu.txt"), &imu_);
      }
      for (const auto& diagnostic : result.imu_diagnostics) {
        imu_ << std::setw(10) << diagnostic.relative_time << " "
             << diagnostic.angular_velocity.transpose() << " "
             << diagnostic.acceleration.transpose() << std::endl;
      }
      if (!result.updated) return;
      if (result.kind != UpdateKind::kVisual && options_.trajectory) {
        WritePose(trajectory_, result);
      }
      if (result.kind == UpdateKind::kVisual && options_.colmap) {
        WriteColmap(result, prepared.colmap_image);
      }
      if (prepared.has_cloud) WriteProducts(result, prepared);
      for (const auto& debug : result.debug_images) {
        fs::create_directories(Path("ref_cur_combine"));
        WriteImage(Path("ref_cur_combine") / debug.name, debug.image);
      }
      WriteStateRow(out_, result.relative_time, result.estimate);
      out_ << " " << result.undistorted_cloud->size() << std::endl;
    } catch (const std::exception& error) {
      closed_ = true;
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
    colmap_poses_ << colmap_count_ << " " << std::fixed << std::setprecision(6)
                  << orientation.w() << " " << orientation.x() << " "
                  << orientation.y() << " " << orientation.z() << " "
                  << translation.x() << " " << translation.y() << " "
                  << translation.z() << " 1 " << name.str() << std::endl
                  << "0.0 0.0 -1" << std::endl;
    ++colmap_count_;
  }

  void WriteProducts(const FrontendResult& result,
                     const PreparedOutput& prepared) {
    const bool visual = result.kind == UpdateKind::kVisual;
    const std::string name = Timestamp(result.timestamp);
    if (options_.pcd) {
      if (options_.pcd_type == 0) {
        if (options_.mode == FrontendMode::kLidarInertialVisual) {
          if (prepared.colored_cloud) colored_ += *prepared.colored_cloud;
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
      if ((!colored_.empty() || !plain_.empty()) && interval > 0 &&
          scan_count_ >= interval) {
        const auto path = Path("pcd") / (name + ".pcd");
        if (!colored_.empty()) {
          WriteCloud(path, colored_);
          ColoredCloud().swap(colored_);
        }
        if (!plain_.empty()) {
          WriteCloud(path, plain_);
          PointCloud().swap(plain_);
        }
        scan_count_ = 0;
      }
      if (!visual) WritePose(lidar_poses_, result, true);
    }
    if (options_.images && visual &&
        ++image_count_ >= options_.image_interval) {
      WriteImage(Path("image") / (name + ".png"), result.color_image);
      WritePose(image_poses_, result);
      image_count_ = 0;
    }
  }

  void Finish() {
    if (closed_) return;
    closed_ = true;
    try {
      if (options_.pcd && options_.pcd_type == 0 &&
          options_.pcd_interval < 0 && (!colored_.empty() || !plain_.empty())) {
        WriteAccumulatedCloud();
      }
      for (auto* stream : {&pre_, &out_, &trajectory_, &imu_, &lidar_poses_,
                           &image_poses_, &points_, &colmap_poses_}) {
        if (stream->is_open()) stream->close();
      }
    } catch (const std::exception& error) {
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
    ColoredCloud::Ptr input(new ColoredCloud(colored_));
    ColoredCloud downsampled;
    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setInputCloud(input);
    filter.setLeafSize(options_.voxel_size, options_.voxel_size,
                       options_.voxel_size);
    filter.filter(downsampled);
    WriteCloud(raw, colored_);
    WriteCloud(Path("pcd/all_downsampled_points.pcd"), downsampled);
    if (!options_.colmap) return;
    points_ << "# 3D point list with one line of data per point\n"
            << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
    for (std::size_t i = 0; i < downsampled.size(); ++i) {
      const auto& point = downsampled[i];
      points_ << i << " " << std::fixed << std::setprecision(6) << point.x
              << " " << point.y << " " << point.z << " "
              << static_cast<int>(point.r) << " " << static_cast<int>(point.g)
              << " " << static_cast<int>(point.b) << " 0" << std::endl;
    }
  }

  OutputOptions options_;
  std::ofstream pre_, out_, trajectory_, imu_, lidar_poses_, image_poses_;
  std::ofstream points_, colmap_poses_;
  ColoredCloud colored_;
  PointCloud plain_;
  int scan_count_ = 0;
  int image_count_ = 0;
  int colmap_count_ = 1;
  bool closed_ = false;
};

ResultWriter::ResultWriter(const OutputOptions& options)
    : impl_(std::make_unique<Impl>(options)) {}
ResultWriter::~ResultWriter() = default;
void ResultWriter::Write(const FrontendResult& result,
                         const PreparedOutput& prepared) {
  impl_->Write(result, prepared);
}
void ResultWriter::Finish() { impl_->Finish(); }

}  // namespace livo_atlas
