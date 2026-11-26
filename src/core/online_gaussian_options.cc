#include "livo_atlas/core/online_gaussian_options.h"

#include <stdexcept>
#include <system_error>

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

bool ContainsPath(const fs::path& parent, const fs::path& path) {
  auto left = parent.begin();
  auto right = path.begin();
  for (; left != parent.end(); ++left, ++right) {
    if (right == path.end() || *left != *right) return false;
  }
  return true;
}

}  // namespace

void ValidateOnlineGaussianOptions(const OnlineGaussianOptions& options) {
  if (options.evaluation_enabled &&
      (!options.enabled || !options.persist_submaps ||
       options.evaluation_split_file.empty()))
    throw std::invalid_argument(
        "Gaussian evaluation requires persistence and a frozen split file");
  if (options.joint_render_policy != GaussianRenderPolicy::kTemporal &&
      options.joint_render_policy != GaussianRenderPolicy::kSpatial)
    throw std::invalid_argument("Unknown Gaussian joint render policy");
  if (options.joint_render_policy == GaussianRenderPolicy::kSpatial &&
      (!options.enabled || !options.persist_submaps ||
       !options.joint_render_enabled))
    throw std::invalid_argument(
        "Spatial rendering requires Gaussian persistence and joint rendering");
  if (options.persist_submaps && !options.enabled)
    throw std::invalid_argument(
        "Gaussian persistence requires gaussian.enabled");
  if (options.finalize_on_shutdown &&
      (!options.enabled || !options.persist_submaps))
    throw std::invalid_argument("Gaussian finalization requires persistence");
  if (options.joint_render_enabled &&
      (!options.enabled || !options.persist_submaps))
    throw std::invalid_argument(
        "Joint rendering requires Gaussian persistence");
  if (options.joint_render_enabled &&
      (options.joint_render_interval.count() < 100 ||
       options.joint_render_interval.count() > 60000 ||
       options.joint_render_timeout.count() < 1 ||
       options.joint_render_timeout.count() > 60000))
    throw std::invalid_argument("Invalid joint render interval or timeout");
  if (!options.enabled) return;
  if (options.socket_path.empty() || options.output_directory.empty() ||
      options.connect_timeout.count() <= 0 ||
      options.transfer_timeout.count() <= 0 ||
      options.job_timeout.count() <= 0 ||
      options.shutdown_timeout.count() <= 0) {
    throw std::invalid_argument(
        "Online Gaussian requires socket, output directory and positive "
        "timeouts");
  }
  if (options.finalize_timeout.count() < 1 ||
      options.finalize_timeout.count() > 600000)
    throw std::invalid_argument(
        "Gaussian finalization timeout is outside 1..600000 ms");
}

void ValidateOnlineGaussianPaths(const OnlineGaussianOptions& options,
                                 const fs::path& dataset) {
  ValidateOnlineGaussianOptions(options);
  if (!options.enabled) return;
  // The optional worker uses Linux AF_UNIX, whose path holds at most 107 bytes.
  if (fs::absolute(options.socket_path).string().size() > 107)
    throw std::invalid_argument("Gaussian socket path is too long");
  const auto parent =
      fs::canonical(fs::absolute(options.output_directory).parent_path());
  const auto output = parent / options.output_directory.filename();
  const auto source = fs::weakly_canonical(dataset);
  if (options.output_directory.filename().empty() ||
      ContainsPath(source, output) || ContainsPath(output, source))
    throw std::invalid_argument(
        "Gaussian output must be separate from the dataset");
  std::error_code error;
  const auto existing = fs::symlink_status(output, error);
  if ((!error && existing.type() != fs::file_type::not_found) ||
      (error && error != std::errc::no_such_file_or_directory))
    throw std::invalid_argument(
        "Gaussian output directory must not already exist");
}

}  // namespace livo_atlas
