#ifndef LIVO_ATLAS_SRC_IO_LOOP_PROPOSER_H_
#define LIVO_ATLAS_SRC_IO_LOOP_PROPOSER_H_

#include <filesystem>

#include "livo_atlas/io/loop_closure_store.h"

namespace livo_atlas {

void ProposeLoopDataset(const std::filesystem::path& dataset,
                        const std::filesystem::path& output,
                        const LoopAdmissionOptions& options = {});

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SRC_IO_LOOP_PROPOSER_H_
