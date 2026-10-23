#ifndef LIVO_ATLAS_IO_RESULT_WRITER_H_
#define LIVO_ATLAS_IO_RESULT_WRITER_H_

#include <filesystem>
#include <memory>
#include <string>

#include "livo_atlas/output/output_processor.h"

namespace livo_atlas {

struct OutputOptions {
  std::filesystem::path directory;
  std::string sequence_name = "01";
  FrontendMode mode = FrontendMode::kLidarInertialVisual;
  Calibration calibration;
  bool trajectory = false;
  bool pcd = false;
  bool images = false;
  bool colmap = false;
  int pcd_interval = -1;
  int image_interval = 1;
  int pcd_type = 0;
  double voxel_size = 0.5;
};

// Validates all options without touching the filesystem. Throws on error.
void ValidateOutputOptions(const OutputOptions& options);

class ResultWriter {
 public:
  explicit ResultWriter(const OutputOptions& options);
  ~ResultWriter();
  ResultWriter(const ResultWriter&) = delete;
  ResultWriter& operator=(const ResultWriter&) = delete;

  void Write(const FrontendResult& result, const PreparedOutput& prepared);
  // Explicit completion reports I/O failures. Destruction only closes streams.
  void Finish();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_IO_RESULT_WRITER_H_
