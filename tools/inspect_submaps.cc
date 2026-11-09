#include <charconv>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <locale>
#include <string>
#include <system_error>

#include "livo_atlas/io/online_submap_store.h"
#include "livo_atlas/io/submap_store.h"

int main(int argc, char** argv) {
  const bool online = argc >= 3 && std::string(argv[2]) == "--online";
  const int version_argument = online ? 3 : 2;
  if (argc < 2 || argc > version_argument + 1) {
    std::cerr << "Usage: livo_atlas_inspect_submaps DATASET_DIRECTORY "
                 "[--online] [POSE_VERSION]\n";
    return 2;
  }
  std::uint64_t version = 0;
  if (argc == version_argument + 1) {
    const std::string text(argv[version_argument]);
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
    std::cout.imbue(std::locale::classic());
    if (online) {
      livo_atlas::OnlineSubmapStore submaps(directory / "online_submaps",
                                            keyframes);
      const auto frozen = submaps.ReadPoseSnapshot(version);
      const auto poses =
          submaps.MaterializePoseSnapshot(version, keyframes.ids().size());
      std::uint64_t active = 0, support = 0;
      for (std::size_t id = 0; id < submaps.size(); ++id) {
        const auto submap = submaps.Read(id);
        active += submap.state == livo_atlas::OnlineSubmapState::kActive;
        support += submap.support_keyframe_ids.size();
      }
      std::cout
          << "{\"inspection\":\"metadata\",\"mode\":\"online\",\"submaps\":"
          << submaps.size() << ",\"active_submaps\":" << active
          << ",\"primary_frames\":" << submaps.indexed_frame_count()
          << ",\"support_references\":" << support << ",\"unindexed_frames\":"
          << keyframes.ids().size() - submaps.indexed_frame_count()
          << ",\"pose_version\":" << version
          << ",\"stored_frame_count\":" << (version ? frozen.frame_count : 0)
          << ",\"frame_count\":" << poses.frame_count
          << ",\"optimized_frame_count\":" << poses.optimized_frame_count
          << "}\n";
      return 0;
    }
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
