#ifndef LIVO_ATLAS_IO_LOOP_CLOSURE_STORE_H_
#define LIVO_ATLAS_IO_LOOP_CLOSURE_STORE_H_

#include <cstdint>
#include <exception>
#include <filesystem>
#include <string>
#include <vector>

#include "livo_atlas/backend/loop_task.h"

namespace livo_atlas {

enum class LoopDisposition {
  kRejected,
  kAccepted,
  kStale,
  kResourceLimit,
  kFailed
};

struct LoopSessionStats {
  std::uint64_t frame_count = 0, pose_version = 0;
  std::uint64_t submitted = 0, completed = 0, rejected = 0, accepted = 0;
  std::uint64_t stale = 0, skipped = 0, resource_limited = 0;
  std::uint64_t peak_work_payloads = 0, peak_pending = 0;
};

enum class LoopRecoveryStatus { kConsistent, kIncomplete, kInconsistent };

struct LoopRecoveryReport {
  LoopRecoveryStatus status = LoopRecoveryStatus::kIncomplete;
  std::uint64_t committed_version = 0;
  std::vector<std::uint64_t> uncommitted_prepared;
  std::vector<std::string> issues;
};

LoopAdmissionOptions ReadLoopAdmissionOptions(
    const std::filesystem::path& path);

// Only the serial system owner writes these records. A prepared record is
// evidence of intent; the matching published system pose establishes commit.
class LoopClosureStore {
 public:
  LoopClosureStore(const KeyframeReadView& frames,
                   const LoopAdmissionOptions& options);
  void WritePrepared(const LoopTaskResult& result,
                     const PoseSnapshot& candidate);
  void WriteQuery(const LoopTaskResult& result, LoopDisposition disposition,
                  std::uint64_t current_version, std::uint64_t frame_count);
  void WriteCoalesced(std::uint64_t first, std::uint64_t last,
                      std::uint64_t count);
  void WriteResourceLimit(std::uint64_t query_id, std::uint64_t frame_count);
  void WriteFailure(std::exception_ptr error) noexcept;
  void Finish(const LoopSessionStats& stats);

 private:
  void WriteSummary(const LoopSessionStats& stats, bool complete);
  std::filesystem::path directory_;
  std::uint64_t next_event_ = 1;
};

// Call only after the dataset writer has stopped. This never repairs inputs.
LoopRecoveryReport InspectLoopDataset(const std::filesystem::path& dataset);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_IO_LOOP_CLOSURE_STORE_H_
