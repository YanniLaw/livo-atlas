#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

#include "io/loop_verifier.h"

int main(int argc, char** argv) {
  if ((argc != 5 && argc != 7) || std::string(argv[3]) != "--pose-version" ||
      std::string(argv[4]) != "0" ||
      (argc == 7 && std::string(argv[5]) != "--config")) {
    std::cerr
        << "Usage: livo_atlas_verify_loops DATASET OUTPUT --pose-version 0 "
           "[--config FILE]\n";
    return 2;
  }
  try {
    const auto options = argc == 7
                             ? livo_atlas::ReadLoopVerifierOptions(argv[6])
                             : livo_atlas::LoopVerifierOptions{};
    livo_atlas::VerifyLoopDataset(argv[1], argv[2], options);
    std::cout << "Loop verification completed; see "
              << (std::filesystem::path(argv[2]) / "summary.yml") << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Loop verification failed: " << error.what() << '\n';
    return 1;
  }
}
