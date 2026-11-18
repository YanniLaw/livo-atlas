#ifndef LIVO_ATLAS_CORE_ONLINE_GAUSSIAN_OPTIONS_H_
#define LIVO_ATLAS_CORE_ONLINE_GAUSSIAN_OPTIONS_H_

#include <chrono>
#include <filesystem>

namespace livo_atlas {

struct OnlineGaussianOptions {
  bool enabled = false;
  bool persist_submaps = false;
  bool finalize_on_shutdown = false;
  bool joint_render_enabled = false;
  std::chrono::milliseconds joint_render_interval{1000};
  std::chrono::milliseconds joint_render_timeout{5000};
  std::chrono::milliseconds finalize_timeout{60000};
  std::filesystem::path socket_path;
  std::filesystem::path output_directory;
  std::chrono::milliseconds connect_timeout{10000};
  std::chrono::milliseconds transfer_timeout{5000};
  std::chrono::milliseconds job_timeout{30000};
  std::chrono::milliseconds shutdown_timeout{3000};
};

void ValidateOnlineGaussianOptions(const OnlineGaussianOptions& options);

// Read-only filesystem checks, performed after logical configuration validation
// and before creating either the keyframe dataset or Gaussian output.
void ValidateOnlineGaussianPaths(const OnlineGaussianOptions& options,
                                 const std::filesystem::path& dataset);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_CORE_ONLINE_GAUSSIAN_OPTIONS_H_
