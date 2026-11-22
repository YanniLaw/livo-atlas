#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>

#include "livo_atlas/common/logging.h"
#include "livo_atlas/io/keyframe_store.h"

int main(int argc, char** argv) {
  livo_atlas::InitializeLogging(argv[0]);
  if (argc != 2) {
    LOG(ERROR) << "Usage: livo_atlas_inspect_keyframes DATASET_DIRECTORY";
    return 2;
  }
  try {
    livo_atlas::KeyframeStore dataset(argv[1]);
    std::uint64_t points = 0, images = 0;
    double first = 0, last = 0;
    for (auto id : dataset.ids()) {
      const auto frame = dataset.Read(id);
      // Exercise the same pose/geometry contract used by reconstruction.
      const auto cloud =
          livo_atlas::TransformKeyframeCloud(frame, frame.odom_from_body);
      points += cloud.size();
      if (frame.image_timestamp) ++images;
      if (id == 0) first = frame.timestamp;
      last = frame.timestamp;
    }
    std::cout.imbue(std::locale::classic());
    std::cout << std::setprecision(std::numeric_limits<double>::max_digits10)
              << "{\"frames\":" << dataset.ids().size()
              << ",\"images\":" << images << ",\"points\":" << points
              << ",\"first_timestamp\":" << first
              << ",\"last_timestamp\":" << last << "}\n";
    return 0;
  } catch (const std::exception& error) {
    LOG(ERROR) << error.what();
    return 1;
  }
}
