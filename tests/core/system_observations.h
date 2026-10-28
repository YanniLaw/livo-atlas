#ifndef LIVO_ATLAS_TESTS_CORE_SYSTEM_OBSERVATIONS_H_
#define LIVO_ATLAS_TESTS_CORE_SYSTEM_OBSERVATIONS_H_

#include <gtest/gtest.h>

#include <cstdint>
#include <utility>
#include <vector>

#include "core/fixtures.h"
#include "livo_atlas/core/livo_system.h"
#include "test_images.h"

namespace livo_atlas::test {
struct Observations {
  std::vector<std::vector<double>> states;
  std::vector<std::vector<float>> clouds;
  std::vector<FrontendResult> results;
  std::vector<std::uint64_t> image_hashes;
};

inline Observations RunSystemWithKeyframes(SystemOptions options,
                                           bool finish_twice = false) {
  LivoSystem system(options);
  Observations observations;
  const auto collect = [&](const FrontendResult& result, const auto&, double) {
    const auto& s = result.estimate;
    std::vector<double> state;
    for (const auto* matrix : {&s.rot_end})
      state.insert(state.end(), matrix->data(),
                   matrix->data() + matrix->size());
    for (const auto* vector :
         {&s.pos_end, &s.vel_end, &s.bias_g, &s.bias_a, &s.gravity})
      state.insert(state.end(), vector->data(),
                   vector->data() + vector->size());
    state.push_back(s.inv_expo_time);
    state.insert(state.end(), s.cov.data(), s.cov.data() + s.cov.size());
    observations.states.push_back(std::move(state));
    std::vector<float> cloud;
    for (const auto& point : *result.world_cloud)
      cloud.insert(cloud.end(), {point.x, point.y, point.z, point.intensity});
    observations.clouds.push_back(std::move(cloud));
    observations.image_hashes.push_back(
        test_data::ImageHash(result.color_image));
    observations.results.push_back(result);
  };
  for (int frame = 0; frame < 6; ++frame) {
    const double time = 100 + frame * .25;
    if (!system.AddLidar(Scan(time))) ADD_FAILURE() << "Missing LiDAR";
    for (double offset : {.0625, .125, .1875})
      system.AddImu({time + offset, Eigen::Vector3d::Zero(), {0, 0, -9.81}});
    if (options.image_enabled)
      system.AddImage({time + .1, test_data::MakeImage(512, 512, 3)});
    system.ProcessAvailable(collect);
  }
  system.Finish();
  if (finish_twice) system.Finish();
  return observations;
}

}  // namespace livo_atlas::test
#endif  // LIVO_ATLAS_TESTS_CORE_SYSTEM_OBSERVATIONS_H_
