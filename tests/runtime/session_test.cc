#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <fstream>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>

#include "livo_atlas/runtime/mapping_session.h"
#include "runtime/fixtures.h"
#include "test_images.h"
namespace livo_atlas {
namespace {
TEST(SessionOptionsTest, RejectsInvalidDirectOptionsBeforeTouchingOutput) {
  const std::array<std::function<void(SessionOptions&)>, 7> corrupt = {
      [](auto& options) {
        options.frontend.calibration.imu_from_lidar.translation.x() =
            std::numeric_limits<double>::quiet_NaN();
      },
      [](auto& options) {
        options.frontend.calibration.imu_from_lidar.rotation(0, 0) = 2;
      },
      [](auto& options) {
        options.frontend.calibration.camera_from_lidar.translation.y() =
            std::numeric_limits<double>::infinity();
      },
      [](auto& options) {
        options.frontend.calibration.camera_from_lidar.rotation(0, 0) = -1;
      },
      [](auto& options) {
        options.frontend.visual.outlier_threshold =
            std::numeric_limits<double>::quiet_NaN();
      },
      [](auto& options) {
        options.frontend.visual.plot_time =
            std::numeric_limits<double>::infinity();
      },
      [](auto& options) {
        options.processor.minimum_color_distance =
            std::numeric_limits<double>::quiet_NaN();
      }};
  for (bool existing : {false, true}) {
    for (std::size_t index = 0; index < corrupt.size(); ++index) {
      SCOPED_TRACE(index);
      SCOPED_TRACE(existing);
      auto options = test::Options(FrontendMode::kLidarOnly);
      const auto directory = options.output.directory;
      if (existing) {
        std::filesystem::create_directories(directory);
        for (const char* name : {"mat_pre.txt", "mat_out.txt"})
          std::ofstream(directory / name) << "keep existing data";
      }
      corrupt[index](options);
      EXPECT_THROW(PrepareSessionOptions(options), std::invalid_argument);
      EXPECT_THROW((void)MappingSession(options), std::invalid_argument);
      if (existing) {
        for (const char* name : {"mat_pre.txt", "mat_out.txt"}) {
          std::ifstream input(directory / name);
          std::string content;
          std::getline(input, content);
          EXPECT_EQ(content, "keep existing data");
        }
      } else {
        EXPECT_FALSE(std::filesystem::exists(directory));
      }
    }
  }
}

void CheckNonfiniteTimestamps(bool image, bool hilti) {
  auto options = test::Options(FrontendMode::kLidarInertialVisual);
  options.hilti = hilti;
  MappingSession reference(options);
  options.output.directory /= "injected";
  MappingSession injected(options);
  const auto pixels = test_data::MakeImage(512, 512, 3);
  std::array<std::vector<State>, 2> states;
  std::array<std::vector<double>, 2> times;
  for (int frame = 0; frame < (hilti ? 16 : 6); ++frame) {
    const double timestamp = 100 + frame * .25;
    if (!image) {
      for (double bad : {std::numeric_limits<double>::infinity(),
                         -std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
        ASSERT_FALSE(injected.AddLidar(test::Scan(bad)));
      }
    }
    for (auto* session : {&reference, &injected}) {
      ASSERT_TRUE(session->AddLidar(test::Scan(timestamp)));
      for (double offset : {.0625, .125, .1875}) {
        ASSERT_TRUE(session->AddImu(
            {timestamp + offset, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
      }
    }
    if (image) {
      for (double bad : {std::numeric_limits<double>::infinity(),
                         -std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
        ASSERT_FALSE(injected.AddImage({bad, pixels}));
      }
    }
    const bool accepted = reference.AddImage({timestamp + .125, pixels});
    ASSERT_EQ(injected.AddImage({timestamp + .125, pixels}), accepted);
    std::array<MappingSession*, 2> sessions = {&reference, &injected};
    for (std::size_t index = 0; index < sessions.size(); ++index) {
      sessions[index]->ProcessAvailable(
          [&](const auto& result, const auto&, double) {
            EXPECT_TRUE(result.estimate.pos_end.allFinite());
            EXPECT_TRUE(result.estimate.cov.allFinite());
            states[index].push_back(result.estimate);
            times[index].push_back(result.timestamp);
          });
    }
  }
  ASSERT_GT(states[0].size(), 2u);
  ASSERT_EQ(states[0].size(), states[1].size());
  EXPECT_EQ(times[0], times[1]);
  for (std::size_t index = 0; index < states[0].size(); ++index) {
    EXPECT_TRUE(states[0][index].pos_end == states[1][index].pos_end);
    EXPECT_TRUE(states[0][index].cov == states[1][index].cov);
  }
  reference.Finish();
  injected.Finish();
}

TEST(SessionTest, NonfiniteLidarTimesCannotChangeSubsequentValidEstimates) {
  CheckNonfiniteTimestamps(false, false);
}

TEST(SessionTest, NonfiniteImageTimesCannotBlockEstimatesOrAdvanceHiltiPhase) {
  CheckNonfiniteTimestamps(true, false);
  CheckNonfiniteTimestamps(true, true);
}

TEST(RuntimeStepTest, LidarOnlyStepPreservesAnchorAndProducesFiniteWorldCloud) {
  MappingSession session(test::Options(FrontendMode::kLidarOnly));
  int updates = 0;
  for (int frame = 0; frame < 2; ++frame) {
    const double timestamp = 100 + frame * .25;
    ASSERT_TRUE(session.AddLidar(test::Scan(timestamp)));
    EXPECT_EQ(session.ProcessNext([&](const auto& result, const auto&, double) {
      ++updates;
      EXPECT_DOUBLE_EQ(result.timestamp, timestamp + .125);
      EXPECT_DOUBLE_EQ(result.timestamp - result.relative_time, -1);
      ASSERT_TRUE(result.world_cloud);
      EXPECT_FALSE(result.world_cloud->empty());
      EXPECT_TRUE(result.estimate.pos_end.allFinite());
    }),
              SynchronizationResult::kReady);
  }
  EXPECT_EQ(updates, 2);
  EXPECT_EQ(session.ProcessNext({}), SynchronizationResult::kWaiting);
  session.Finish();
}

TEST(RuntimeStepTest, CoverageUsesTimeBeforeSecondNanosecondQuantization) {
  auto options = test::Options(FrontendMode::kLidarInertial);
  options.driver_fix = true;
  MappingSession session(options);
  ASSERT_TRUE(session.AddLidar(test::Scan(.975)));
  ASSERT_TRUE(session.AddImu({100.1, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_EQ(session.ProcessNext({}), SynchronizationResult::kWaiting);
  ASSERT_TRUE(session.AddImu({100.25, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_FALSE(session.AddImu({100., Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_EQ(session.ProcessNext({}), SynchronizationResult::kReady);
  session.Finish();
}

TEST(RuntimeStepTest, TransportRangeAllowsUnsignedSecondsAndRejectsCarry) {
  auto options = test::Options(FrontendMode::kLidarInertial);
  options.imu_time_range = {0, 4294967296.0};
  MappingSession unsigned_session(options);
  ASSERT_TRUE(unsigned_session.AddLidar(test::Scan(2147483664.0)));
  EXPECT_TRUE(unsigned_session.AddImu(
      {2147483664.0625, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_THROW(
      unsigned_session.AddImu({-1., Eigen::Vector3d::Zero(), {0, 0, -9.81}}),
      std::invalid_argument);
  options.output.directory /= "bounded";
  options.imu_time_range = {0, 101};
  MappingSession bounded(options);
  ASSERT_TRUE(bounded.AddLidar(test::Scan(100)));
  EXPECT_THROW(
      bounded.AddImu({100.9999999996, Eigen::Vector3d::Zero(), {0, 0, -9.81}}),
      std::invalid_argument);
  EXPECT_TRUE(
      bounded.AddImu({100.0625, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  unsigned_session.Finish();
  bounded.Finish();
}

TEST(RuntimeStepTest, EmptyCloudCannotErasePendingScanBeforeRollback) {
  MappingSession session(test::Options(FrontendMode::kLidarOnly));
  ASSERT_TRUE(session.AddLidar(test::Scan(100)));
  EXPECT_FALSE(session.AddLidar({99, {}}));
  std::vector<double> timestamps;
  const auto collect = [&](const auto& result, const auto&, double) {
    timestamps.push_back(result.timestamp);
  };
  EXPECT_EQ(session.ProcessNext(collect), SynchronizationResult::kReady);
  ASSERT_EQ(timestamps.size(), 1u);
  EXPECT_DOUBLE_EQ(timestamps.back(), 100.125);
  ASSERT_TRUE(session.AddLidar(test::Scan(200)));
  ASSERT_TRUE(session.AddLidar(test::Scan(199)));
  session.ProcessAvailable(collect);
  ASSERT_EQ(timestamps.size(), 2u);
  EXPECT_DOUBLE_EQ(timestamps.back(), 199.125);
  session.Finish();
}

TEST(RuntimeStepTest, InitialImuWindowDoesNotPublishAnEstimate) {
  MappingSession session(test::Options(FrontendMode::kLidarInertial));
  ASSERT_TRUE(session.AddLidar(test::Scan(100)));
  for (double t : {100.0625, 100.125}) {
    ASSERT_TRUE(session.AddImu({t, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  }
  int updates = 0;
  EXPECT_EQ(
      session.ProcessNext([&](const auto&, const auto&, double) { ++updates; }),
      SynchronizationResult::kReady);
  EXPECT_EQ(updates, 0);
  session.Finish();
}

TEST(RuntimeStepTest, RepeatedVisualUpdatesConsumeCloudsWithoutRosPublishers) {
  auto options = test::Options(FrontendMode::kLidarInertialVisual);
  options.processor.scan_interval = 4;
  MappingSession session(options);
  int visual = 0, lidar = 0;
  std::vector<std::size_t> cloud_sizes;
  for (int frame = 0; frame < 6; ++frame) {
    const double t = 100 + frame * .25;
    ASSERT_TRUE(session.AddLidar(test::Scan(t)));
    for (double offset : {.0625, .125, .1875}) {
      session.AddImu({t + offset, Eigen::Vector3d::Zero(), {0, 0, -9.81}});
    }
    ASSERT_TRUE(
        session.AddImage({t + .125, test_data::MakeImage(512, 512, 3)}));
    session.ProcessAvailable(
        [&](const auto& result, const auto& prepared, double) {
          EXPECT_TRUE(result.estimate.cov.allFinite());
          if (result.kind == UpdateKind::kVisual) {
            ++visual;
            cloud_sizes.push_back(prepared.colored_cloud->size());
          } else {
            ++lidar;
          }
        });
  }
  EXPECT_GT(lidar, 2);
  EXPECT_GT(visual, 2);
  EXPECT_NE(std::find(cloud_sizes.begin(), cloud_sizes.end(), 0),
            cloud_sizes.end());
  EXPECT_TRUE(std::any_of(cloud_sizes.begin(), cloud_sizes.end(),
                          [](auto size) { return size > 0; }));
  session.Finish();
  EXPECT_GT(
      std::filesystem::file_size(options.output.directory / "mat_out.txt"), 0u);
}

TEST(SessionTest, RejectsEarlyAndReverseDataWithoutBlockingValidScan) {
  MappingSession session(test::Options(FrontendMode::kLidarInertial));
  EXPECT_FALSE(session.AddImu({100, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_TRUE(session.AddLidar(test::Scan(100)));
  EXPECT_TRUE(session.AddImu({100.1, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_FALSE(session.AddImu({99, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_FALSE(session.AddLidar({99, {}}));
  session.Finish();
}
TEST(SessionTest, IndependentPropagationCursors) {
  auto options = test::Options(FrontendMode::kLidarInertial);
  options.propagate_imu = true;
  MappingSession a(options);
  options.output.directory /= "second";
  MappingSession b(options);
  for (int frame = 0; frame < 3; ++frame) {
    double t = 100 + frame * .25;
    for (auto* session : {&a, &b}) {
      ASSERT_TRUE(session->AddLidar(test::Scan(t)));
      session->AddImu({t + .0625, Eigen::Vector3d::Zero(), {0, 0, -9.81}});
      session->AddImu({t + .125, Eigen::Vector3d::Zero(), {0, 0, -9.81}});
      session->ProcessAvailable([](const auto&, const auto&, double) {});
      session->AddImu({t + .1875, Eigen::Vector3d::Zero(), {1, 0, -9.81}});
    }
    auto first = a.PropagateLatest(), second = b.PropagateLatest();
    if (frame == 0) {
      EXPECT_FALSE(first);
      EXPECT_FALSE(second);
      continue;
    }
    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    EXPECT_TRUE(first->state.pos_end == second->state.pos_end);
    EXPECT_FALSE(a.PropagateLatest());
    EXPECT_FALSE(b.PropagateLatest());
  }
  a.Finish();
  b.Finish();
}
TEST(SessionTest, ImageRejectionAndHiltiCountersBelongToEachInstance) {
  auto options = test::Options(FrontendMode::kLidarInertialVisual);
  MappingSession normal(options);
  const auto image = test_data::MakeImage(512, 512, 3);
  EXPECT_FALSE(normal.AddImage({100.1, image}));
  ASSERT_TRUE(normal.AddLidar(test::Scan(100)));
  EXPECT_TRUE(normal.AddImage({100.1, image}));
  EXPECT_FALSE(normal.AddImage({100.1, image}));
  EXPECT_FALSE(normal.AddImage({100.11, image}));
  EXPECT_FALSE(normal.AddImage({100.08, image}));
  EXPECT_TRUE(normal.AddImage({100.2, image}));
  normal.Finish();
  options.hilti = true;
  options.output.directory /= "hilti-first";
  MappingSession first(options);
  options.output.directory /= "hilti-second";
  MappingSession second(options);
  ASSERT_TRUE(first.AddLidar(test::Scan(100)));
  ASSERT_TRUE(second.AddLidar(test::Scan(100)));
  for (int i = 1; i <= 4; ++i) {
    EXPECT_EQ(first.AddImage({100 + i * .05, image}), i == 4);
    EXPECT_EQ(second.AddImage({100 + i * .05, image}), i == 4);
  }
  first.Finish();
  second.Finish();
}
void ExpectSameStates(const std::vector<FrontendResult>& reference,
                      const std::vector<FrontendResult>& actual) {
  ASSERT_EQ(reference.size(), actual.size());
  ASSERT_GT(reference.size(), 2u);
  for (std::size_t i = 0; i < reference.size(); ++i) {
    EXPECT_DOUBLE_EQ(reference[i].timestamp, actual[i].timestamp);
    const auto& a = reference[i].estimate;
    const auto& b = actual[i].estimate;
    EXPECT_TRUE(a.rot_end == b.rot_end);
    EXPECT_TRUE(a.pos_end == b.pos_end);
    EXPECT_TRUE(a.vel_end == b.vel_end);
    EXPECT_TRUE(a.bias_a == b.bias_a);
    EXPECT_TRUE(a.bias_g == b.bias_g);
    EXPECT_TRUE(a.gravity == b.gravity);
    EXPECT_TRUE(a.cov == b.cov);
    EXPECT_DOUBLE_EQ(a.inv_expo_time, b.inv_expo_time);
    EXPECT_TRUE(b.pos_end.allFinite() && b.cov.allFinite());
  }
}
TEST(SessionTest, RejectingNonfiniteImuLeavesValidSequenceUnchanged) {
  auto options = test::Options(FrontendMode::kLidarInertial);
  MappingSession reference(options);
  options.output.directory /= "injected";
  MappingSession actual(options);
  std::vector<FrontendResult> expected, observed;
  for (int frame = 0; frame < 6; ++frame) {
    const double t = 100 + frame * .25;
    reference.AddLidar(test::Scan(t));
    actual.AddLidar(test::Scan(t));
    for (int axis = 0; axis < 6; ++axis) {
      ImuMeasurement invalid{t + 1000, Eigen::Vector3d::Zero(), {0, 0, -9.81}};
      if (axis < 3)
        invalid.angular_velocity[axis] =
            std::numeric_limits<double>::quiet_NaN();
      else
        invalid.linear_acceleration[axis - 3] =
            std::numeric_limits<double>::infinity();
      ASSERT_FALSE(actual.AddImu(invalid));
    }
    for (double offset : {.0625, .125, .1875}) {
      const ImuMeasurement imu{
          t + offset, Eigen::Vector3d::Zero(), {frame * .01, 0, -9.81}};
      ASSERT_TRUE(reference.AddImu(imu));
      ASSERT_TRUE(actual.AddImu(imu));
    }
    reference.ProcessAvailable(
        [&](const auto& r, const auto&, double) { expected.push_back(r); });
    actual.ProcessAvailable(
        [&](const auto& r, const auto&, double) { observed.push_back(r); });
  }
  ExpectSameStates(expected, observed);
  reference.Finish();
  actual.Finish();
}
TEST(SessionTest, AppliesImuOffsetAndDriverCorrectionAtActualIngress) {
  struct Case {
    double begin;
    double offset;
    double raw_delta;
    bool driver;
  };
  // Corrected reference stamps are dyadic fractions. The final case crosses
  // a whole-second nanosecond carry: 1000.999999999 - (-0.6ns) -> 1001.
  for (const auto& c :
       std::vector<Case>{{1000, .03, .03, false},
                         {1700000000, .03, -512 + .03, true},
                         {1000.9375, -.0000000006, -.000000001, false}}) {
    SCOPED_TRACE(c.begin);
    auto options = test::Options(FrontendMode::kLidarInertial);
    MappingSession reference(options);
    options.output.directory /= "corrected";
    options.imu_time_offset = c.offset;
    options.driver_fix = c.driver;
    MappingSession actual(options);
    std::vector<FrontendResult> expected, observed;
    for (int frame = 0; frame < 6; ++frame) {
      const double t = c.begin + frame * .25;
      reference.AddLidar(test::Scan(t));
      actual.AddLidar(test::Scan(t));
      for (double offset : {.0625, .125, .1875}) {
        ImuMeasurement imu{
            t + offset, Eigen::Vector3d::Zero(), {frame * .01, 0, -9.81}};
        ASSERT_TRUE(reference.AddImu(imu));
        imu.timestamp = (t + offset + c.raw_delta);
        ASSERT_TRUE(actual.AddImu(imu));
      }
      ImuMeasurement reverse{
          (t + .04 + c.raw_delta), Eigen::Vector3d::Zero(), {0, 0, -9.81}};
      EXPECT_FALSE(actual.AddImu(reverse));
      reference.ProcessAvailable(
          [&](const auto& r, const auto&, double) { expected.push_back(r); });
      actual.ProcessAvailable(
          [&](const auto& r, const auto&, double) { observed.push_back(r); });
    }
    ExpectSameStates(expected, observed);
    reference.Finish();
    actual.Finish();
  }
}

}  // namespace
}  // namespace livo_atlas
