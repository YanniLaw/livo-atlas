#ifndef LIVO_ATLAS_IO_GAUSSIAN_MAP_WRITER_H_
#define LIVO_ATLAS_IO_GAUSSIAN_MAP_WRITER_H_

#include <filesystem>
#include <stdexcept>

#include "livo_atlas/gaussian/gaussian_map.h"

namespace livo_atlas {

class GaussianMapCommitUncertainError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// One serial publisher per newly created run directory. This writes only the
// index; source assets are immutable and are never read, changed or pruned
// here. Compiled into the optional online_gaussian library, not the default io
// library.
void WriteGaussianMapIndex(const std::filesystem::path& output,
                           const GaussianMapSnapshot& snapshot);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_IO_GAUSSIAN_MAP_WRITER_H_
