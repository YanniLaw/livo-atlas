#include <exception>
#include <filesystem>
#include <string>

#include "io/loop_proposer.h"
#include "livo_atlas/common/logging.h"

int main(int argc, char** argv) {
  livo_atlas::InitializeLogging(argv[0]);
  if ((argc != 5 && argc != 7) || std::string(argv[3]) != "--pose-version" ||
      std::string(argv[4]) != "0" ||
      (argc == 7 && std::string(argv[5]) != "--config")) {
    LOG(ERROR)
        << "Usage: livo_atlas_propose_loops DATASET OUTPUT --pose-version 0 "
           "[--config FILE]";
    return 2;
  }
  try {
    const auto options = argc == 7
                             ? livo_atlas::ReadLoopAdmissionOptions(argv[6])
                             : livo_atlas::LoopAdmissionOptions{};
    livo_atlas::ProposeLoopDataset(argv[1], argv[2], options);
    LOG(INFO) << "Loop admission completed; see "
              << (std::filesystem::path(argv[2]) / "summary.yml");
    return 0;
  } catch (const std::exception& error) {
    LOG(ERROR) << "Loop admission failed: " << error.what();
    return 1;
  } catch (...) {
    LOG(ERROR) << "Loop admission failed: unknown exception";
    return 1;
  }
}
