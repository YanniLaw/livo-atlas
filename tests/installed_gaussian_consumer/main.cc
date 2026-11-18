#include <chrono>

#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/gaussian/online_gaussian_mapper.h"
#include "livo_atlas/io/gaussian_render_writer.h"

// Keep mapper lifecycle references in the separate consumer even though the
// executable does not start a mapping session.
void ConstructOptionalMapper(livo_atlas::LivoSystem& system) {
  livo_atlas::OnlineGaussianMapper mapper(system, {}, {});
  (void)mapper.GetMapSnapshot();
  (void)mapper.GetStats();
  (void)mapper.GetLatestRender();
  (void)mapper.GetRenderStats();
  (void)system.IsFinished();
  (void)mapper.Finalize(std::chrono::milliseconds(1));
}

void WriteOptionalRender(const std::filesystem::path& directory,
                         const livo_atlas::GaussianRender& render) {
  livo_atlas::WriteGaussianRender(directory, render);
}

int main() {
  // Existing C++17 aggregate clients must retain their member ordering.
  const livo_atlas::OnlineGaussianOptions options{
      true, true, false, std::chrono::milliseconds{60000},
      "/tmp/livo-consumer.sock", "/tmp/livo-consumer-output"};
  return options.finalize_timeout == std::chrono::milliseconds{60000} &&
                 !options.joint_render_enabled &&
                 options.joint_render_interval ==
                     std::chrono::milliseconds{1000}
             ? 0
             : 1;
}
