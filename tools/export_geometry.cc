#include <charconv>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>

#include "livo_atlas/io/geometry_exporter.h"

namespace {

void Usage() {
  std::cerr
      << "Usage: livo_atlas_export_geometry DATASET_DIRECTORY OUTPUT_DIRECTORY "
         "[--pose-version V] [--voxel-size M]\n";
}

std::uint64_t ParseVersion(const std::string& text) {
  std::uint64_t value = 0;
  const auto parsed =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (parsed.ec != std::errc() || parsed.ptr != text.data() + text.size() ||
      std::to_string(value) != text)
    throw std::invalid_argument("Invalid pose version");
  return value;
}

double ParseSize(const std::string& text) {
  std::istringstream input(text);
  input.imbue(std::locale::classic());
  double value = 0;
  if (!(input >> value) || !input.eof() || !std::isfinite(value) ||
      value <= 0) {
    throw std::invalid_argument("Invalid voxel size");
  }
  return value;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    Usage();
    return 0;
  }
  if (argc < 3) {
    Usage();
    return 2;
  }
  std::uint64_t version = 0;
  livo_atlas::GeometryOptions options;
  bool have_version = false, have_size = false;
  try {
    for (int i = 3; i < argc; ++i) {
      const std::string argument(argv[i]);
      if (i + 1 >= argc) throw std::invalid_argument("Missing option value");
      if (argument == "--pose-version" && !have_version) {
        version = ParseVersion(argv[++i]);
        have_version = true;
      } else if (argument == "--voxel-size" && !have_size) {
        options.voxel_size = ParseSize(argv[++i]);
        have_size = true;
      } else {
        throw std::invalid_argument("Unknown or repeated option");
      }
    }
    livo_atlas::ValidateGeometryOptions(options);
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    Usage();
    return 2;
  }
  try {
    livo_atlas::ExportGeometry(argv[1], argv[2], version, options);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
