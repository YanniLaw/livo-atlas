#include <exception>
#include <filesystem>
#include <stdexcept>

#include "io/dataset_metadata.h"
#include "livo_atlas/common/logging.h"
#include "livo_atlas/io/loop_closure_store.h"

int main(int argc, char** argv) {
  livo_atlas::InitializeLogging(argv[0]);
  if (argc != 3) {
    LOG(ERROR) << "Usage: livo_atlas_check_loops DATASET OUTPUT";
    return 2;
  }
  namespace fs = std::filesystem;
  try {
    const auto dataset = fs::canonical(argv[1]);
    const auto output = fs::weakly_canonical(argv[2]);
    const auto relative = output.lexically_relative(dataset);
    if (relative.empty() || *relative.begin() != "..")
      throw std::invalid_argument(
          "Inspection output must be outside the dataset");
    if (!output.parent_path().empty())
      fs::create_directories(output.parent_path());
    if (!fs::create_directory(output))
      throw std::invalid_argument("Inspection output already exists");
    const auto report = livo_atlas::InspectLoopDataset(dataset);
    const auto status = report.status;
    const char* name = status == livo_atlas::LoopRecoveryStatus::kConsistent
                           ? "consistent"
                       : status == livo_atlas::LoopRecoveryStatus::kIncomplete
                           ? "incomplete"
                           : "inconsistent";
    livo_atlas::dataset_metadata::WriteYaml(
        output / "report.yml", [&](auto& yaml) {
          yaml << "schema_version" << 1 << "status" << name
               << "committed_version"
               << std::to_string(report.committed_version)
               << "uncommitted_prepared"
               << "[";
          for (auto version : report.uncommitted_prepared)
            yaml << std::to_string(version);
          yaml << "]"
               << "issues"
               << "[";
          for (const auto& issue : report.issues) yaml << issue;
          yaml << "]";
        });
    LOG(INFO) << name << ": committed version " << report.committed_version;
    return status == livo_atlas::LoopRecoveryStatus::kConsistent   ? 0
           : status == livo_atlas::LoopRecoveryStatus::kIncomplete ? 3
                                                                   : 1;
  } catch (const std::exception& error) {
    LOG(ERROR) << "Loop inspection failed: " << error.what();
    return 1;
  } catch (...) {
    LOG(ERROR) << "Loop inspection failed: unknown exception";
    return 1;
  }
}
