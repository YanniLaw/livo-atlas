#ifndef LIVO_ATLAS_SRC_IO_LOOP_ADMISSION_IO_H_
#define LIVO_ATLAS_SRC_IO_LOOP_ADMISSION_IO_H_

#include <filesystem>
#include <opencv2/core/persistence.hpp>

#include "livo_atlas/backend/loop_admission.h"
#include "livo_atlas/mapping/geometry_mapper.h"

namespace livo_atlas {
namespace loop_admission_io {
LoopAdmissionOptions ReadOptions(const cv::FileNode& node,
                                 bool with_geometry = false);
void WriteOptions(cv::FileStorage& yaml, LoopAdmissionOptions options,
                  const GeometryOptions& geometry);
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
}  // namespace loop_admission_io
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SRC_IO_LOOP_ADMISSION_IO_H_
