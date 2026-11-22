#include <exception>
#include <filesystem>
#include <string>

#include "io/loop_verifier.h"
#include "livo_atlas/common/logging.h"

int main(int argc, char** argv) {
  livo_atlas::InitializeLogging(argv[0]);
  if ((argc != 5 && argc != 7) || std::string(argv[3]) != "--pose-version" ||
      std::string(argv[4]) != "0" ||
      (argc == 7 && std::string(argv[5]) != "--config")) {
    LOG(ERROR)
        << "Usage: livo_atlas_verify_loops DATASET OUTPUT --pose-version 0 "
           "[--config FILE]";
    return 2;
  }
  try {
    const auto options = argc == 7
                             ? livo_atlas::ReadLoopVerifierOptions(argv[6])
                             : livo_atlas::LoopVerifierOptions{};
    livo_atlas::VerifyLoopDataset(argv[1], argv[2], options);
    LOG(INFO) << "Loop verification completed; see "
              << (std::filesystem::path(argv[2]) / "summary.yml");
    return 0;
  } catch (const std::exception& error) {
    LOG(ERROR) << "Loop verification failed: " << error.what();
    return 1;
  }
}
