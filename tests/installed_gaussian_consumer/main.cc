#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/gaussian/online_gaussian_mapper.h"

// Keep constructor and destructor references in the separate consumer even
// though the executable does not start a mapping session.
void ConstructOptionalMapper(livo_atlas::LivoSystem& system) {
  livo_atlas::OnlineGaussianMapper mapper(system, {}, {});
}

int main() { return 0; }
