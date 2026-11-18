#include "livo_atlas/io/gaussian_render_writer.h"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <iomanip>
#include <locale>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "gaussian/render_protocol.h"
#include "mapping/pose_math.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

void Require(bool condition, const char* message) {
  if (!condition) throw std::invalid_argument(message);
}

void Number(std::ostream& out, double value) {
  Require(std::isfinite(value), "Nonfinite render metadata");
  out << value;
}

void Optional(std::ostream& out, const std::optional<double>& value) {
  if (value) Number(out, *value);
  else out << "null";
}

void Pose(std::ostream& out, const RigidTransform& pose) {
  pose_math::Validate(pose);
  out << '[';
  for (int i = 0; i < 16; ++i) {
    if (i) out << ',';
    Number(out, i / 4 == 3 ? (i == 15 ? 1 : 0)
                 : i % 4 == 3 ? pose.translation[i / 4]
                              : pose.rotation(i / 4, i % 4));
  }
  out << ']';
}

void Metrics(std::ostream& out, const GaussianRenderMetrics& m, int pixels) {
  Require(m.image_pixels >= 0 && m.image_pixels <= pixels &&
              m.covered_pixels >= 0 && m.covered_pixels <= m.image_pixels &&
              m.depth_pixels_available >= 0 &&
              m.depth_pixels_available <= m.image_pixels &&
              m.depth_pixels_used >= 0 &&
              m.depth_pixels_used <= m.depth_pixels_available &&
              m.depth_pixels_used <= m.covered_pixels &&
              m.rgb_l1.has_value() == (m.image_pixels > 0) &&
              m.rgb_mse.has_value() == (m.image_pixels > 0) &&
              m.alpha_coverage.has_value() == (m.image_pixels > 0) &&
              m.depth_l1_m.has_value() == (m.depth_pixels_used > 0) &&
              m.depth_rmse_m.has_value() == (m.depth_pixels_used > 0) &&
              m.rgb_perfect == (m.rgb_mse && *m.rgb_mse == 0),
          "Inconsistent render metrics");
  out << "{\"image_pixels\":" << m.image_pixels
      << ",\"covered_pixels\":" << m.covered_pixels
      << ",\"depth_pixels_available\":" << m.depth_pixels_available
      << ",\"depth_pixels_used\":" << m.depth_pixels_used
      << ",\"rgb_perfect\":" << (m.rgb_perfect ? "true" : "false");
  for (const auto& field :
       {std::pair<const char*, std::optional<double>>{"rgb_l1", m.rgb_l1},
        {"rgb_mse", m.rgb_mse}, {"rgb_psnr_db", m.rgb_psnr_db},
        {"alpha_coverage", m.alpha_coverage}, {"depth_l1_m", m.depth_l1_m},
        {"depth_rmse_m", m.depth_rmse_m}}) {
    out << ",\"" << field.first << "\":";
    Optional(out, field.second);
  }
  out << '}';
}

std::string Metadata(const GaussianRender& r) {
  const auto request = EncodeGaussianRenderRequest(r.request);
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(17)
      << "{\"schema\":1,\"kind\":\"livo_atlas.joint_render\","
         "\"training_view\":true,\"as_of\":" << request.header_json
      << ",\"width\":" << r.width << ",\"height\":" << r.height
      << ",\"calibration_id\":\"" << r.calibration_id << "\",\"K\":[";
  for (int i = 0; i < 9; ++i) {
    if (i) out << ',';
    Number(out, r.K(i / 3, i % 3));
  }
  out << ']';
  for (const auto& field :
       {std::pair<const char*, RigidTransform>{
            "parent_from_body", r.request.parent_from_body},
        {"camera_from_body", r.camera_from_body},
        {"camera_from_parent", r.camera_from_parent}}) {
    out << ",\"" << field.first << "\":";
    Pose(out, field.second);
  }
  for (const auto& field :
       {std::pair<const char*, double>{"capture_time", r.capture_time},
        {"state_timestamp", r.state_timestamp},
        {"cloud_timestamp", r.cloud_timestamp},
        {"inverse_exposure_time", r.inverse_exposure_time},
        {"accepted_at_unix_seconds", r.accepted_at_unix_seconds},
        {"accepted_at_steady_seconds", r.accepted_at_steady_seconds},
        {"latency_seconds", r.latency_seconds},
        {"worker_seconds", r.worker_seconds}}) {
    out << ",\"" << field.first << "\":";
    Number(out, field.second);
  }
  out << ",\"changed_pixels\":" << r.changed_pixels << ",\"single\":";
  Metrics(out, r.single, r.width * r.height);
  out << ",\"joint\":";
  Metrics(out, r.joint, r.width * r.height);
  for (const auto& field :
       {std::pair<const char*, std::uint64_t>{
            "gpu_peak_allocated_bytes", r.gpu_peak_allocated_bytes},
        {"gpu_peak_reserved_bytes", r.gpu_peak_reserved_bytes},
        {"gpu_allocated_bytes", r.gpu_allocated_bytes},
        {"gpu_reserved_bytes", r.gpu_reserved_bytes},
        {"worker_peak_rss_bytes", r.worker_peak_rss_bytes}})
    out << ",\"" << field.first << "\":\"" << field.second << '"';
  out << "}\n";
  const auto text = out.str();
  Require(text.size() <= 65536, "Render metadata exceeds 64 KiB");
  return text;
}

void WriteNew(const fs::path& path, const char* data, std::size_t size) {
  int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC |
                                  O_NOFOLLOW, 0600);
  if (fd < 0)
    throw std::system_error(errno, std::generic_category(), "open");
  try {
    for (std::size_t offset = 0; offset < size;) {
      const auto count = write(fd, data + offset, size - offset);
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0)
        throw std::system_error(errno, std::generic_category(), "write");
      offset += static_cast<std::size_t>(count);
    }
    const int result = close(fd);
    fd = -1;
    if (result)
      throw std::system_error(errno, std::generic_category(), "close");
  } catch (...) {
    if (fd >= 0) close(fd);
    throw;
  }
}

}  // namespace

void WriteGaussianRender(const fs::path& directory,
                         const GaussianRender& render) {
  const auto pixels =
      static_cast<std::int64_t>(render.width) * render.height;
  Require(render.width > 0 && render.height > 0 && pixels <= 1000000 &&
              render.rgb.size() == 3u * render.width * render.height,
          "Invalid render pixels");
  const auto metadata = Metadata(render);
  cv::Mat rgb(render.height, render.width, CV_8UC3,
              const_cast<std::uint8_t*>(render.rgb.data()));
  cv::Mat bgr;
  cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
  std::vector<std::uint8_t> png;
  Require(cv::imencode(".png", bgr, png) && png.size() <= 4 * 1024 * 1024,
          "Render PNG exceeds its 4 MiB budget");
  const auto stem = "joint-render-" + std::to_string(render.request.job_id);
  const auto image = directory / (stem + ".png");
  const auto json = directory / (stem + ".json");
  const auto temporary_image = directory / (stem + ".tmp.png");
  const auto temporary_json = directory / (stem + ".tmp.json");
  for (const auto& path : {image, json, temporary_image, temporary_json})
    Require(!fs::exists(fs::symlink_status(path)),
            "Render output already exists");
  try {
    WriteNew(temporary_image, reinterpret_cast<const char*>(png.data()),
             png.size());
    fs::rename(temporary_image, image);
    WriteNew(temporary_json, metadata.data(), metadata.size());
    fs::rename(temporary_json, json);  // Only complete pairs get a JSON marker.
  } catch (...) {
    std::error_code ignored;
    for (const auto& path : {temporary_image, temporary_json, image})
      fs::remove(path, ignored);
    throw;
  }
}

}  // namespace livo_atlas
