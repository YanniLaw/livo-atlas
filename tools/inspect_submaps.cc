#include <charconv>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <locale>
#include <string>
#include <system_error>

#include "livo_atlas/io/submap_store.h"

int main(int argc, char** argv) {
  if (argc != 2 && argc != 3) {
    std::cerr << "Usage: livo_atlas_inspect_submaps DATASET_DIRECTORY "
                 "[POSE_VERSION]\n";
    return 2;
  }
  std::uint64_t version = 0;
  if (argc == 3) {
    const std::string text(argv[2]);
    const auto parsed =
        std::from_chars(text.data(), text.data() + text.size(), version);
    if (parsed.ec != std::errc() || parsed.ptr != text.data() + text.size() ||
        text != std::to_string(version)) {
      std::cerr << "Expected a canonical unsigned pose version\n";
      return 2;
    }
  }
  try {
    const std::filesystem::path directory(argv[1]);
    livo_atlas::KeyframeStore keyframes(directory);
    livo_atlas::SubmapStore submaps(directory / "submaps", keyframes);
    // An explicit version must exist even when no groups have been committed.
    if (version) (void)keyframes.ReadPoses(version);
    std::uint64_t frames = 0, points = 0;
    for (std::size_t id = 0; id < submaps.size(); ++id) {
      frames += submaps.Read(id).keyframe_ids.size();
      points += submaps.ReadCloud(id, version).cloud.size();
    }
    std::cout.imbue(std::locale::classic());
    std::cout << "{\"submaps\":" << submaps.size() << ",\"frames\":" << frames
              << ",\"points\":" << points << ",\"pose_version\":" << version
              << "}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
