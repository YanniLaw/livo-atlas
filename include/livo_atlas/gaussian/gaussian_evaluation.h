#ifndef LIVO_ATLAS_GAUSSIAN_GAUSSIAN_EVALUATION_H_
#define LIVO_ATLAS_GAUSSIAN_GAUSSIAN_EVALUATION_H_

#include <string>

#include "livo_atlas/gaussian/gaussian_render.h"

namespace livo_atlas {

struct GaussianEvaluationQuery {
  GaussianCamera camera;
  std::uint64_t submap_id = 0;
};

struct GaussianEvaluationRequest {
  GaussianRenderRequest sources;
  GaussianCamera camera;
  std::string split_id;
  double sealed_at_unix_seconds = 0;
  double dispatched_at_unix_seconds = 0;
};

struct GaussianEvaluationStatus {
  GaussianEvaluationQuery query;
  std::string status = "waiting_seal";
  std::string reason;
  std::string error_code;
  std::string report_sha256;
  double sealed_at_unix_seconds = 0;
  std::uint64_t job_id = 0, map_sequence = 0;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_GAUSSIAN_GAUSSIAN_EVALUATION_H_
