#ifndef LIVO_ATLAS_CORE_ONLINE_GAUSSIAN_OPTIONS_H_
#define LIVO_ATLAS_CORE_ONLINE_GAUSSIAN_OPTIONS_H_

#include <chrono>
#include <filesystem>

#include "livo_atlas/gaussian/gaussian_render_policy.h"

namespace livo_atlas {

struct OnlineGaussianOptions {
  bool enabled = false;
  bool persist_submaps = false;
  bool finalize_on_shutdown = false;
  std::chrono::milliseconds finalize_timeout{60000};
  std::filesystem::path socket_path;
  std::filesystem::path output_directory;
  std::chrono::milliseconds connect_timeout{10000};
  std::chrono::milliseconds transfer_timeout{5000};
  std::chrono::milliseconds job_timeout{30000};
  std::chrono::milliseconds shutdown_timeout{3000};
  // Keep new defaulted members last for existing aggregate initializers.
  bool joint_render_enabled = false;
  std::chrono::milliseconds joint_render_interval{1000};
  std::chrono::milliseconds joint_render_timeout{5000};
  GaussianRenderPolicy joint_render_policy = GaussianRenderPolicy::kTemporal;
  bool evaluation_enabled = false;
  std::filesystem::path evaluation_split_file;
  bool online_evaluation_enabled = false;
  int evaluation_max_output_mib = 256;
  int evaluation_min_free_disk_mib = 1024;
};

void ValidateOnlineGaussianOptions(const OnlineGaussianOptions& options);

// Read-only filesystem checks, performed after logical configuration validation
// and before creating either the keyframe dataset or Gaussian output.
void ValidateOnlineGaussianPaths(const OnlineGaussianOptions& options,
                                 const std::filesystem::path& dataset);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_CORE_ONLINE_GAUSSIAN_OPTIONS_H_
