#include <charconv>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <locale>
#include <string>
#include <system_error>

#include "livo_atlas/io/colmap_exporter.h"

namespace {
bool ParseUnsigned(const char* argument, std::uint64_t* value) {
  const std::string text(argument);
  const auto parsed =
      std::from_chars(text.data(), text.data() + text.size(), *value);
  return parsed.ec == std::errc() && parsed.ptr == text.data() + text.size() &&
         text == std::to_string(*value);
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 3 || argc > 5) {
    std::cerr << "Usage: livo_atlas_export_colmap DATASET_DIRECTORY "
                 "OUTPUT_DIRECTORY [POSE_VERSION] [MAX_POINTS_PER_IMAGE]\n";
    return 2;
  }
  livo_atlas::ColmapExportOptions options;
  std::uint64_t limit = options.max_points_per_image;
  if ((argc >= 4 && !ParseUnsigned(argv[3], &options.pose_version)) ||
      (argc == 5 && !ParseUnsigned(argv[4], &limit)) || limit == 0 ||
      limit >= UINT32_MAX) {
    std::cerr << "Expected canonical unsigned arguments; the point limit must "
                 "be in [1, UINT32_MAX)\n";
    return 2;
  }
  options.max_points_per_image = static_cast<std::uint32_t>(limit);
  try {
    const auto result = livo_atlas::ExportColmap(argv[1], argv[2], options);
    std::cout.imbue(std::locale::classic());
    std::cout << "{\"images\":" << result.images
              << ",\"points\":" << result.points
              << ",\"pose_version\":" << options.pose_version << "}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
