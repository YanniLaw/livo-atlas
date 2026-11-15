#include "livo_atlas/core/online_gaussian_options.h"

#include <stdexcept>

namespace livo_atlas {

void ValidateOnlineGaussianOptions(const OnlineGaussianOptions& options) {
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
}

}  // namespace livo_atlas
