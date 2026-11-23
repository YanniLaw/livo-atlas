#include "livo_atlas/ros_common/sensor_input.h"

#include <gtest/gtest.h>

#include <exception>
#include <filesystem>
#include <stdexcept>
#include <string>

#include "core/fixtures.h"
#include "livo_atlas/io/loop_closure_store.h"

namespace livo_atlas::ros_detail {
namespace {

class SensorInputTest : public testing::Test {
 protected:
  void TearDown() override {
    std::filesystem::remove_all(options_.exporter.directory);
  }

  SystemOptions options_ = test::Options(FrontendMode::kLidarOnly);
};

TEST_F(SensorInputTest, PreservesDriverFixAnchorOrderAndImageFifo) {
  options_.frontend.imu.enabled = options_.image_enabled = true;
  options_.driver_fix = true;
  LivoSystem system(options_);
  SensorInput input(options_.sensor_buffer, false);
  const cv::Mat image(1, 1, CV_8UC3, cv::Scalar(1, 2, 3));
  ASSERT_TRUE(input.Push(test::Scan(100)));
  ASSERT_TRUE(input.Push(Imu{100.75}));
  ASSERT_TRUE(input.Push(TimedImage{101.7, image}));
  ASSERT_TRUE(input.Push(test::Scan(101)));
  ASSERT_TRUE(input.Push(Imu{100.15}));
  ASSERT_TRUE(input.Push(TimedImage{101.8, image}));
  EXPECT_EQ(system.GetSensorBufferCounts().lidar, 0u);
  EXPECT_EQ(input.ProcessNext(system, {}), SyncStatus::kWaiting);
  const auto counts = system.GetSensorBufferCounts();
  EXPECT_EQ(counts.lidar, 2u);
  // The two IMUs map to 99.75 and 101.15 with the corresponding LiDAR
  // anchors. Applying both scans first would reject the second IMU.
  EXPECT_EQ(counts.imu, 2u);
  EXPECT_EQ(counts.image, 2u);
}

TEST_F(SensorInputTest, BoundsEachStepAndDrainsAllReadyInput) {
  LivoSystem system(options_);
  SensorInput input(options_.sensor_buffer, false);
  for (int scan = 0; scan < 40; ++scan) {
    ASSERT_TRUE(input.Push(test::Scan(100 + scan * 0.05)));
  }
  EXPECT_EQ(input.ProcessNext(system, {}), SyncStatus::kReady);
  EXPECT_EQ(system.GetSensorBufferCounts().lidar, 31u);
  input.Drain(system);
  EXPECT_EQ(system.GetSensorBufferCounts().lidar, 0u);
  EXPECT_FALSE(input.Push(test::Scan(103)));
  input.Drain(system);
  system.Finish();
  EXPECT_TRUE(system.IsFinished());
}

TEST_F(SensorInputTest, RetainsUnsynchronizedInputForShutdownDiagnostics) {
  options_.frontend.imu.enabled = true;
  LivoSystem system(options_);
  SensorInput input(options_.sensor_buffer, false);
  ASSERT_TRUE(input.Push(test::Scan(100)));
  input.Drain(system);
  EXPECT_EQ(system.GetSensorBufferCounts().lidar, 1u);
  system.Finish();
  EXPECT_TRUE(system.IsFinished());
}

TEST_F(SensorInputTest, DrainsReadyScansBeforeRefillingAFullSystemQueue) {
  options_.sensor_buffer.max_lidar_scans = 2;
  LivoSystem system(options_);
  SensorInput input(options_.sensor_buffer, false);
  ASSERT_TRUE(input.Push(test::Scan(100)));
  ASSERT_TRUE(input.Push(test::Scan(100.1)));
  ASSERT_EQ(input.ProcessNext(system, {}), SyncStatus::kReady);
  ASSERT_EQ(system.GetSensorBufferCounts().lidar, 1u);
  ASSERT_TRUE(input.Push(test::Scan(100.2)));
  ASSERT_TRUE(input.Push(test::Scan(100.3)));
  EXPECT_EQ(input.ProcessNext(system, {}), SyncStatus::kReady);
  EXPECT_NO_THROW(input.Drain(system));
  EXPECT_EQ(system.GetSensorBufferCounts().lidar, 0u);
}

TEST_F(SensorInputTest, DrainsReadyScansWithATightTimeSpan) {
  options_.sensor_buffer.max_span_s = 0.1;
  LivoSystem system(options_);
  SensorInput input(options_.sensor_buffer, false);
  ASSERT_TRUE(input.Push(test::Scan(100)));
  ASSERT_TRUE(input.Push(test::Scan(100.2)));
  EXPECT_NO_THROW(input.Drain(system));
  EXPECT_EQ(system.GetSensorBufferCounts().lidar, 0u);
}

TEST_F(SensorInputTest, ProcessesReadyScansBeforeExceedingTheirSpan) {
  options_.sensor_buffer.max_span_s = 0.1;
  for (const double timestamp : {100.0, -0.9}) {
    SCOPED_TRACE(timestamp);
    LivoSystem system(options_);
    SensorInput input(options_.sensor_buffer, false);
    ASSERT_TRUE(input.Push(test::Scan(timestamp)));
    ASSERT_TRUE(input.Push(test::Scan(timestamp + 0.2)));
    EXPECT_EQ(input.ProcessNext(system, {}), SyncStatus::kReady);
    EXPECT_EQ(input.ProcessNext(system, {}), SyncStatus::kReady);
    EXPECT_EQ(input.ProcessNext(system, {}), SyncStatus::kWaiting);
  }
}

TEST_F(SensorInputTest,
       FirstOverflowFailsTheSessionAndLeavesAnIncompleteJournal) {
  options_.sensor_buffer.max_lidar_scans = 1;
  options_.sensor_buffer.max_imu_samples = 1;
  options_.sensor_buffer.max_images = 1;
  options_.keyframes.enabled = options_.submaps.enabled = true;
  options_.submap_mode = SubmapMode::kOnline;
  options_.loop_closure.emplace();
  const auto dataset = options_.exporter.directory / "keyframes";
  {
    LivoSystem system(options_);
    SensorInput input(options_.sensor_buffer, false);
    ASSERT_TRUE(input.Push(test::Scan(100)));
    ASSERT_TRUE(input.Push(Imu{100.01}));
    ASSERT_TRUE(input.Push(TimedImage{100.02, {}}));
    EXPECT_FALSE(input.Push(test::Scan(101)));
    EXPECT_FALSE(input.Push(Imu{101.01}));
    input.Fail(std::make_exception_ptr(std::runtime_error("later failure")));
    EXPECT_EQ(system.GetSensorBufferCounts().lidar, 0u);
    try {
      input.Drain(system);
      ADD_FAILURE() << "Drain should report the first ingress failure";
    } catch (const std::overflow_error& error) {
      EXPECT_NE(std::string(error.what())
                    .find("ROS sensor input overflow: lidar count=1 limit=1"),
                std::string::npos);
    }
    EXPECT_THROW(system.Finish(), std::overflow_error);
    EXPECT_FALSE(system.IsFinished());
  }
  EXPECT_EQ(InspectLoopDataset(dataset).status,
            LoopRecoveryStatus::kIncomplete);
}

}  // namespace
}  // namespace livo_atlas::ros_detail
