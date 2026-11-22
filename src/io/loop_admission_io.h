#ifndef LIVO_ATLAS_SRC_IO_LOOP_ADMISSION_IO_H_
#define LIVO_ATLAS_SRC_IO_LOOP_ADMISSION_IO_H_

#include <cstdint>
#include <filesystem>
#include <vector>
#include <opencv2/core/persistence.hpp>

#include "livo_atlas/backend/loop_admission.h"

namespace livo_atlas {
class OnlineSubmapStore;

namespace loop_admission_io {
LoopAdmissionOptions ReadOptions(const cv::FileNode& node,
                                 bool with_geometry = false);
void WriteOptions(cv::FileStorage& yaml, LoopAdmissionOptions options);
void WriteMembership(cv::FileStorage& yaml, const char* name,
                     const OnlineSubmap& group);
void WriteQuery(cv::FileStorage& yaml, const LoopSubmapInfo& source,
                const LoopAdmissionInput& input,
                const LoopAdmissionResult& result, double elapsed_seconds);
void WriteProposal(cv::FileStorage& yaml, const LoopAdmissionInput& input,
                   const LoopAdmissionResult& result);
void CheckUniqueKeys(const cv::FileNode& node);
void WriteSnapshot(cv::FileStorage& yaml, const char* name,
                   const PoseSnapshot& snapshot);
PoseSnapshot ReadSnapshot(const cv::FileNode& node);
OnlineSubmap ReadMembership(const cv::FileNode& node);
AcceptedLoop ReadLoop(const cv::FileNode& node);
PoseUpdate ReadUpdate(const cv::FileNode& node);
bool SameMembership(const OnlineSubmap& a, const OnlineSubmap& b);
bool SameSnapshot(const PoseSnapshot& a, const PoseSnapshot& b);
// Checks stored diagnostics and metadata only; never reloads clouds or reruns
// registration/optimization. Early rejection records may omit later stages.
void CheckAdmissionDiagnostics(const cv::FileNode& node,
                               const OnlineSubmapStore& store,
                               const std::vector<double>& timestamps,
                               std::uint64_t query_id,
                               const PoseSnapshot& base);
void CheckVerification(const cv::FileNode& node, const OnlineSubmap& target,
                       const OnlineSubmap& source, const PoseSnapshot& base);
}  // namespace loop_admission_io
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SRC_IO_LOOP_ADMISSION_IO_H_
