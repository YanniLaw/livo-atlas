/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "livo_atlas/frontend/vision/frame.h"

#include <algorithm>
#include <opencv2/core/mat.hpp>
#include <stdexcept>

#include "livo_atlas/frontend/vision/feature.h"
#include "livo_atlas/mapping/visual_point.h"

namespace livo_atlas {

std::atomic<int> Frame::frame_counter_{0};

Frame::Frame(livo_atlas::Camera* cam, const cv::Mat& img)
    : id_(frame_counter_.fetch_add(1, std::memory_order_relaxed)), cam_(cam) {
  Initialize(img);
}

Frame::~Frame() {
  std::for_each(fts_.begin(), fts_.end(), [&](Feature* i) { delete i; });
}

void Frame::Initialize(const cv::Mat& img) {
  if (img.empty()) {
    throw std::runtime_error("Frame: provided image is empty");
  }

  if (img.cols != cam_->width() || img.rows != cam_->height()) {
    throw std::runtime_error(
        "Frame: provided image has not the same size as the camera model");
  }

  if (img.type() != CV_8UC1) {
    throw std::runtime_error("Frame: provided image is not grayscale");
  }

  img_ = img;
}

}  // namespace livo_atlas
