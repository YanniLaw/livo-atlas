#include <chrono>

#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/gaussian/online_gaussian_mapper.h"

// Keep mapper lifecycle references in the separate consumer even though the
// executable does not start a mapping session.
void ConstructOptionalMapper(livo_atlas::LivoSystem& system) {
  livo_atlas::OnlineGaussianMapper mapper(system, {}, {});
  (void)mapper.GetMapSnapshot();
  (void)mapper.GetStats();
  (void)system.IsFinished();
  (void)mapper.Finalize(std::chrono::milliseconds(1));
}

int main() { return 0; }
