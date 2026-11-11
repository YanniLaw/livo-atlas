#ifndef LIVO_ATLAS_SRC_IO_LOOP_VERIFIER_H_
#define LIVO_ATLAS_SRC_IO_LOOP_VERIFIER_H_

#include <filesystem>

#include "livo_atlas/backend/loop_closure.h"

namespace livo_atlas {

struct LoopVerifierOptions {
  LoopSearchOptions search;
  LoopRegistrationOptions registration;
};

LoopVerifierOptions ReadLoopVerifierOptions(const std::filesystem::path& path);
void VerifyLoopDataset(const std::filesystem::path& dataset,
                       const std::filesystem::path& output,
                       const LoopVerifierOptions& options = {});

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SRC_IO_LOOP_VERIFIER_H_
