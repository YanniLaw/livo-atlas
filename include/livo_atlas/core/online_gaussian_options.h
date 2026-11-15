#ifndef LIVO_ATLAS_CORE_ONLINE_GAUSSIAN_OPTIONS_H_
#define LIVO_ATLAS_CORE_ONLINE_GAUSSIAN_OPTIONS_H_

#include <chrono>
#include <filesystem>

namespace livo_atlas {

struct OnlineGaussianOptions {
  bool enabled = false;
  std::filesystem::path socket_path;
  std::filesystem::path output_directory;
  std::chrono::milliseconds connect_timeout{10000};
  std::chrono::milliseconds transfer_timeout{5000};
  std::chrono::milliseconds job_timeout{30000};
  std::chrono::milliseconds shutdown_timeout{3000};
};

void ValidateOnlineGaussianOptions(const OnlineGaussianOptions& options);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_CORE_ONLINE_GAUSSIAN_OPTIONS_H_
