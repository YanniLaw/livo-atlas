#include "livo_atlas/core/livo_system.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "core/fixtures.h"
#include "test_images.h"
namespace livo_atlas {
namespace {
TEST(SystemOptionsTest, RejectsInvalidDirectOptionsBeforeTouchingOutput) {
  const std::array<std::function<void(SystemOptions&)>, 7> corrupt = {
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
        options.colorizer.minimum_color_distance =
            std::numeric_limits<double>::quiet_NaN();
      }};
  for (bool existing : {false, true}) {
    for (std::size_t index = 0; index < corrupt.size(); ++index) {
      SCOPED_TRACE(index);
      SCOPED_TRACE(existing);
      auto options = test::Options(FrontendMode::kLidarOnly);
      const auto directory = options.exporter.directory;
      if (existing) {
        std::filesystem::create_directories(directory);
        for (const char* name : {"mat_pre.txt", "mat_out.txt"})
          std::ofstream(directory / name) << "keep existing data";
      }
      corrupt[index](options);
      EXPECT_THROW(PrepareSystemOptions(options), std::invalid_argument);
      EXPECT_THROW((void)LivoSystem(options), std::invalid_argument);
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
  LivoSystem reference(options);
  options.exporter.directory /= "injected";
  LivoSystem injected(options);
  const auto pixels = test_data::MakeImage(512, 512, 3);
  std::array<std::vector<FilterState>, 2> states;
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
    std::array<LivoSystem*, 2> sessions = {&reference, &injected};
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

TEST(SystemTest, NonfiniteLidarTimesCannotChangeSubsequentValidEstimates) {
  CheckNonfiniteTimestamps(false, false);
}

TEST(SystemTest, NonfiniteImageTimesCannotBlockEstimatesOrAdvanceHiltiPhase) {
  CheckNonfiniteTimestamps(true, false);
  CheckNonfiniteTimestamps(true, true);
}

TEST(SystemStepTest, LidarOnlyStepPreservesAnchorAndProducesFiniteWorldCloud) {
  LivoSystem session(test::Options(FrontendMode::kLidarOnly));
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
              SyncStatus::kReady);
  }
  EXPECT_EQ(updates, 2);
  EXPECT_EQ(session.ProcessNext({}), SyncStatus::kWaiting);
  session.Finish();
}

TEST(SystemStepTest, CoverageUsesTimeBeforeSecondNanosecondQuantization) {
  auto options = test::Options(FrontendMode::kLidarInertial);
  options.driver_fix = true;
  LivoSystem session(options);
  ASSERT_TRUE(session.AddLidar(test::Scan(.975)));
  ASSERT_TRUE(session.AddImu({100.1, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_EQ(session.ProcessNext({}), SyncStatus::kWaiting);
  ASSERT_TRUE(session.AddImu({100.25, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_FALSE(session.AddImu({100., Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_EQ(session.ProcessNext({}), SyncStatus::kReady);
  session.Finish();
}

TEST(SystemStepTest, TransportRangeAllowsUnsignedSecondsAndRejectsCarry) {
  auto options = test::Options(FrontendMode::kLidarInertial);
  options.imu_time_range = {0, 4294967296.0};
  LivoSystem unsigned_session(options);
  ASSERT_TRUE(unsigned_session.AddLidar(test::Scan(2147483664.0)));
  EXPECT_TRUE(unsigned_session.AddImu(
      {2147483664.0625, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_THROW(
      unsigned_session.AddImu({-1., Eigen::Vector3d::Zero(), {0, 0, -9.81}}),
      std::invalid_argument);
  options.exporter.directory /= "bounded";
  options.imu_time_range = {0, 101};
  LivoSystem bounded(options);
  ASSERT_TRUE(bounded.AddLidar(test::Scan(100)));
  EXPECT_THROW(
      bounded.AddImu({100.9999999996, Eigen::Vector3d::Zero(), {0, 0, -9.81}}),
      std::invalid_argument);
  EXPECT_TRUE(
      bounded.AddImu({100.0625, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  unsigned_session.Finish();
  bounded.Finish();
}

TEST(SystemStepTest, EmptyCloudCannotErasePendingScanBeforeRollback) {
  LivoSystem session(test::Options(FrontendMode::kLidarOnly));
  ASSERT_TRUE(session.AddLidar(test::Scan(100)));
  EXPECT_FALSE(session.AddLidar({99, {}}));
  std::vector<double> timestamps;
  const auto collect = [&](const auto& result, const auto&, double) {
    timestamps.push_back(result.timestamp);
  };
  EXPECT_EQ(session.ProcessNext(collect), SyncStatus::kReady);
  ASSERT_EQ(timestamps.size(), 1u);
  EXPECT_DOUBLE_EQ(timestamps.back(), 100.125);
  ASSERT_TRUE(session.AddLidar(test::Scan(200)));
  ASSERT_TRUE(session.AddLidar(test::Scan(199)));
  session.ProcessAvailable(collect);
  ASSERT_EQ(timestamps.size(), 2u);
  EXPECT_DOUBLE_EQ(timestamps.back(), 199.125);
  session.Finish();
}

TEST(SystemStepTest, InitialImuWindowDoesNotPublishAnEstimate) {
  LivoSystem session(test::Options(FrontendMode::kLidarInertial));
  ASSERT_TRUE(session.AddLidar(test::Scan(100)));
  for (double t : {100.0625, 100.125}) {
    ASSERT_TRUE(session.AddImu({t, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  }
  int updates = 0;
  EXPECT_EQ(
      session.ProcessNext([&](const auto&, const auto&, double) { ++updates; }),
      SyncStatus::kReady);
  EXPECT_EQ(updates, 0);
  session.Finish();
}

TEST(SystemStepTest, RepeatedVisualUpdatesConsumeCloudsWithoutRosPublishers) {
  auto options = test::Options(FrontendMode::kLidarInertialVisual);
  options.colorizer.scan_interval = 4;
  LivoSystem session(options);
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
            cloud_sizes.push_back(prepared->size());
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
      std::filesystem::file_size(options.exporter.directory / "mat_out.txt"),
      0u);
}

TEST(SystemTest, RejectsEarlyAndReverseDataWithoutBlockingValidScan) {
  auto options = test::Options(FrontendMode::kLidarInertial);
  options.buffer_before_first_lidar = false;
  LivoSystem session(options);
  EXPECT_FALSE(session.AddImu({100, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_TRUE(session.AddLidar(test::Scan(100)));
  EXPECT_TRUE(session.AddImu({100.1, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_FALSE(session.AddImu({99, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_FALSE(session.AddLidar({99, {}}));
  session.Finish();
}

TEST(SystemTest, StartupOrderDoesNotChangeInitialLivoResults) {
  struct Observation {
    double timestamp;
    UpdateKind kind;
    Eigen::Vector3d position;
    Eigen::Matrix3d rotation;
  };
  struct RunResult {
    std::vector<Observation> observations;
    std::array<std::string, 3> files;
  };
  const auto run = [](bool measurements_first) {
    auto options = test::Options(FrontendMode::kLidarInertialVisual);
    LivoSystem system(options);
    RunResult run_result;
    const auto collect = [&](const auto& result, const auto&, double) {
      run_result.observations.push_back({result.timestamp, result.kind,
                                         result.estimate.pos_end,
                                         result.estimate.rot_end});
    };
    const auto input_measurements = [&](int frame) {
      const double time = 100 + frame * .25;
      for (double offset : {.0625, .125, .1875}) {
        EXPECT_TRUE(
            system.AddImu({time + offset,
                           Eigen::Vector3d::Zero(),
                           {0.01 * frame + offset, 0.02 * offset, -9.81}}));
      }
      EXPECT_TRUE(
          system.AddImage({time + .125, test_data::MakeImage(512, 512, 3)}));
    };
    if (measurements_first) input_measurements(0);
    EXPECT_TRUE(system.AddLidar(test::Scan(100)));
    if (!measurements_first) input_measurements(0);
    system.ProcessAvailable(collect);
    for (int frame = 1; frame < 6; ++frame) {
      EXPECT_TRUE(system.AddLidar(test::Scan(100 + frame * .25)));
      input_measurements(frame);
      system.ProcessAvailable(collect);
    }
    system.Finish();
    const std::array<const char*, 3> names = {"mat_pre.txt", "mat_out.txt",
                                              "imu.txt"};
    for (std::size_t i = 0; i < names.size(); ++i) {
      std::ifstream stream(options.exporter.directory / names[i]);
      EXPECT_TRUE(stream.good());
      run_result.files[i] = std::string(std::istreambuf_iterator<char>(stream),
                                        std::istreambuf_iterator<char>());
    }
    return run_result;
  };

  const auto lidar_first = run(false);
  const auto measurements_first = run(true);
  ASSERT_GT(lidar_first.observations.size(), 2u);
  ASSERT_EQ(lidar_first.observations.size(),
            measurements_first.observations.size());
  for (std::size_t i = 0; i < lidar_first.observations.size(); ++i) {
    const auto& expected = lidar_first.observations[i];
    const auto& actual = measurements_first.observations[i];
    EXPECT_DOUBLE_EQ(actual.timestamp, expected.timestamp);
    EXPECT_EQ(actual.kind, expected.kind);
    EXPECT_TRUE(actual.position.isApprox(expected.position, 0));
    EXPECT_TRUE(actual.rotation.isApprox(expected.rotation, 0));
  }
  EXPECT_EQ(lidar_first.files, measurements_first.files);
}

TEST(SystemTest, StartupBufferRetainsRecentSamplesWithinLimits) {
  auto options = test::Options(FrontendMode::kLidarInertialVisual);
  LivoSystem system(options);
  auto reference_options = options;
  reference_options.exporter.directory /= "retained-suffix";
  LivoSystem reference(reference_options);
  const auto image = test_data::MakeImage(512, 512, 3);
  for (int i = 0; i < 2001; ++i) {
    ASSERT_TRUE(system.AddImu(
        {100 + i * .001, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
    if (i != 0)
      ASSERT_TRUE(reference.AddImu(
          {100 + i * .001, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  }
  for (int i = 0; i < 9; ++i) {
    ASSERT_TRUE(system.AddImage({100 + i * .03, image}));
    if (i != 0) ASSERT_TRUE(reference.AddImage({100 + i * .03, image}));
  }
  auto stats = system.GetStartupBufferStats();
  EXPECT_EQ(stats.pending_imu, 2000u);
  EXPECT_EQ(stats.pending_images, 8u);
  EXPECT_EQ(stats.evicted_imu, 1u);
  EXPECT_EQ(stats.evicted_images, 1u);
  EXPECT_FALSE(system.AddLidar({99, {}}));
  EXPECT_EQ(system.GetStartupBufferStats().pending_imu, 2000u);
  EXPECT_TRUE(system.AddLidar(test::Scan(100)));
  stats = system.GetStartupBufferStats();
  EXPECT_EQ(stats.pending_imu, 0u);
  EXPECT_EQ(stats.pending_images, 0u);
  EXPECT_EQ(stats.evicted_imu, 1u);
  EXPECT_EQ(stats.evicted_images, 1u);
  EXPECT_EQ(stats.rejected_imu, 0u);
  ASSERT_TRUE(reference.AddLidar(test::Scan(100)));
  ASSERT_TRUE(system.AddLidar(test::Scan(100.25)));
  ASSERT_TRUE(reference.AddLidar(test::Scan(100.25)));
  system.ProcessAvailable({});
  reference.ProcessAvailable({});
  system.Finish();
  reference.Finish();
  for (const char* name : {"mat_pre.txt", "mat_out.txt", "imu.txt"}) {
    std::ifstream actual(options.exporter.directory / name);
    std::ifstream expected(reference_options.exporter.directory / name);
    ASSERT_TRUE(actual.good());
    ASSERT_TRUE(expected.good());
    EXPECT_EQ(std::string(std::istreambuf_iterator<char>(actual), {}),
              std::string(std::istreambuf_iterator<char>(expected), {}));
  }
}

TEST(SystemTest, StartupBufferRejectsInvalidInputWithoutChangingState) {
  auto options = test::Options(FrontendMode::kLidarInertialVisual);
  options.imu_time_range = {0, 101};
  LivoSystem system(options);
  const double invalid = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(system.AddImu({100, {invalid, 0, 0}, {0, 0, -9.81}}));
  EXPECT_THROW(system.AddImu({invalid, Eigen::Vector3d::Zero(), {0, 0, -9.81}}),
               std::invalid_argument);
  EXPECT_THROW(system.AddImu({-1, Eigen::Vector3d::Zero(), {0, 0, -9.81}}),
               std::invalid_argument);
  EXPECT_FALSE(system.AddImage({invalid, test_data::MakeImage(512, 512, 3)}));
  const auto stats = system.GetStartupBufferStats();
  EXPECT_EQ(stats.pending_imu, 0u);
  EXPECT_EQ(stats.pending_images, 0u);
  EXPECT_EQ(stats.evicted_imu, 0u);
  EXPECT_EQ(stats.evicted_images, 0u);
  system.Finish();
}

TEST(SystemTest, NegativeLidarTimeDoesNotReleaseStartupData) {
  auto options = test::Options(FrontendMode::kLidarInertial);
  LivoSystem system(options);
  ASSERT_TRUE(system.AddImu({100, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  ASSERT_TRUE(system.AddLidar(test::Scan(-.5)));
  EXPECT_EQ(system.GetStartupBufferStats().pending_imu, 1u);
  ASSERT_TRUE(system.AddLidar(test::Scan(100)));
  EXPECT_EQ(system.GetStartupBufferStats().pending_imu, 0u);
  system.Finish();
}

TEST(SystemTest, StartupBufferCanPreserveLegacyInputBehavior) {
  auto options = test::Options(FrontendMode::kLidarInertialVisual);
  options.buffer_before_first_lidar = false;
  LivoSystem system(options);
  const auto image = test_data::MakeImage(512, 512, 3);
  EXPECT_FALSE(system.AddImu({100.1, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_FALSE(system.AddImage({100.1, image}));
  ASSERT_TRUE(system.AddLidar(test::Scan(100)));
  EXPECT_TRUE(system.AddImu({100.1, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_TRUE(system.AddImage({100.1, image}));
  const auto stats = system.GetStartupBufferStats();
  EXPECT_EQ(stats.buffered_imu, 0u);
  EXPECT_EQ(stats.buffered_images, 0u);
  system.Finish();
}

TEST(SystemTest, StartupImagesPreserveSamplingAndTimeChecks) {
  auto options = test::Options(FrontendMode::kLidarInertialVisual);
  LivoSystem normal(options);
  const auto image = test_data::MakeImage(512, 512, 3);
  EXPECT_TRUE(normal.AddImage({100.1, image}));
  EXPECT_FALSE(normal.AddImage({100.1, image}));
  EXPECT_FALSE(normal.AddImage({100.11, image}));
  EXPECT_FALSE(normal.AddImage({100.08, image}));
  EXPECT_TRUE(normal.AddImage({100.2, image}));
  EXPECT_EQ(normal.GetStartupBufferStats().buffered_images, 2u);
  ASSERT_TRUE(normal.AddLidar(test::Scan(100)));
  EXPECT_TRUE(normal.AddImage({100.3, image}));
  normal.Finish();

  options.hilti = true;
  options.exporter.directory /= "startup-hilti";
  LivoSystem hilti(options);
  for (int i = 1; i <= 4; ++i)
    EXPECT_EQ(hilti.AddImage({100 + i * .05, image}), i == 4);
  EXPECT_EQ(hilti.GetStartupBufferStats().buffered_images, 1u);
  ASSERT_TRUE(hilti.AddLidar(test::Scan(100)));
  for (int i = 5; i <= 8; ++i)
    EXPECT_EQ(hilti.AddImage({100 + i * .05, image}), i == 8);
  hilti.Finish();
}

TEST(SystemTest, SealingWithoutLidarReleasesStartupMeasurements) {
  auto options = test::Options(FrontendMode::kLidarInertialVisual);
  LivoSystem system(options);
  ASSERT_TRUE(system.AddImu({100, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  ASSERT_TRUE(system.AddImage({100, test_data::MakeImage(512, 512, 3)}));
  ASSERT_EQ(system.GetStartupBufferStats().pending_imu, 1u);
  ASSERT_EQ(system.GetStartupBufferStats().pending_images, 1u);
  system.SealInput();
  EXPECT_EQ(system.GetStartupBufferStats().pending_imu, 0u);
  EXPECT_EQ(system.GetStartupBufferStats().pending_images, 0u);
  EXPECT_THROW(system.AddImu({}), std::logic_error);
  system.Finish();
}

TEST(SystemTest, StartupImuCorrectionUsesFirstLidarAnchor) {
  auto options = test::Options(FrontendMode::kLidarInertial);
  options.driver_fix = true;
  options.imu_time_offset = .25;
  LivoSystem lidar_first(options);
  options.exporter.directory /= "imu-first";
  LivoSystem imu_first(options);
  const std::array<double, 3> timestamps = {100.1, 100.25, 100.5};
  ASSERT_TRUE(lidar_first.AddLidar(test::Scan(.975)));
  for (double time : timestamps) {
    ASSERT_TRUE(
        lidar_first.AddImu({time, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
    ASSERT_TRUE(
        imu_first.AddImu({time, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  }
  ASSERT_TRUE(imu_first.AddLidar(test::Scan(.975)));
  EXPECT_EQ(lidar_first.ProcessNext({}), SyncStatus::kReady);
  EXPECT_EQ(imu_first.ProcessNext({}), SyncStatus::kReady);
  EXPECT_EQ(imu_first.GetStartupBufferStats().rejected_imu, 0u);
  ASSERT_TRUE(lidar_first.AddLidar(test::Scan(1.225)));
  ASSERT_TRUE(imu_first.AddLidar(test::Scan(1.225)));
  ASSERT_TRUE(
      lidar_first.AddImu({100.75, Eigen::Vector3d::Zero(), {0.1, 0, -9.81}}));
  ASSERT_TRUE(
      imu_first.AddImu({100.75, Eigen::Vector3d::Zero(), {0.1, 0, -9.81}}));
  EXPECT_EQ(lidar_first.ProcessNext({}), SyncStatus::kReady);
  EXPECT_EQ(imu_first.ProcessNext({}), SyncStatus::kReady);
  lidar_first.Finish();
  imu_first.Finish();
  for (const char* name : {"mat_pre.txt", "mat_out.txt", "imu.txt"}) {
    std::ifstream expected(options.exporter.directory.parent_path() / name);
    std::ifstream actual(options.exporter.directory / name);
    ASSERT_TRUE(expected.good());
    ASSERT_TRUE(actual.good());
    EXPECT_EQ(std::string(std::istreambuf_iterator<char>(actual), {}),
              std::string(std::istreambuf_iterator<char>(expected), {}));
  }
}

TEST(SystemTest, StartupImuRejectsOnlyBadCorrectedOrReversedSamples) {
  auto options = test::Options(FrontendMode::kLidarInertial);
  options.driver_fix = true;
  options.imu_time_range = {0, 101};
  LivoSystem out_of_range(options);
  ASSERT_TRUE(
      out_of_range.AddImu({100.4, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_TRUE(out_of_range.AddLidar(test::Scan(100.975)));
  EXPECT_EQ(out_of_range.GetStartupBufferStats().rejected_imu, 1u);
  EXPECT_EQ(out_of_range.GetStartupBufferStats().pending_imu, 0u);
  out_of_range.Finish();

  options.imu_time_range = {0, 1000};
  options.exporter.directory /= "reversed";
  LivoSystem reversed(options);
  ASSERT_TRUE(
      reversed.AddImu({100.25, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  ASSERT_TRUE(reversed.AddImu({100.1, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_TRUE(reversed.AddLidar(test::Scan(.975)));
  EXPECT_EQ(reversed.GetStartupBufferStats().rejected_imu, 1u);
  reversed.Finish();
}

TEST(SystemTest, IndependentPropagationCursors) {
  auto options = test::Options(FrontendMode::kLidarInertial);
  options.propagate_imu = true;
  LivoSystem a(options);
  options.exporter.directory /= "second";
  LivoSystem b(options);
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

TEST(SystemTest, ImageRejectionAndHiltiCountersBelongToEachInstance) {
  auto options = test::Options(FrontendMode::kLidarInertialVisual);
  options.buffer_before_first_lidar = false;
  LivoSystem normal(options);
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
  options.exporter.directory /= "hilti-first";
  LivoSystem first(options);
  options.exporter.directory /= "hilti-second";
  LivoSystem second(options);
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

TEST(SystemTest, RejectingNonfiniteImuLeavesValidSequenceUnchanged) {
  auto options = test::Options(FrontendMode::kLidarInertial);
  LivoSystem reference(options);
  options.exporter.directory /= "injected";
  LivoSystem actual(options);
  std::vector<FrontendResult> expected, observed;
  for (int frame = 0; frame < 6; ++frame) {
    const double t = 100 + frame * .25;
    reference.AddLidar(test::Scan(t));
    actual.AddLidar(test::Scan(t));
    for (int axis = 0; axis < 6; ++axis) {
      Imu invalid{t + 1000, Eigen::Vector3d::Zero(), {0, 0, -9.81}};
      if (axis < 3)
        invalid.angular_velocity[axis] =
            std::numeric_limits<double>::quiet_NaN();
      else
        invalid.linear_acceleration[axis - 3] =
            std::numeric_limits<double>::infinity();
      ASSERT_FALSE(actual.AddImu(invalid));
    }
    for (double offset : {.0625, .125, .1875}) {
      const Imu imu{
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

TEST(SystemTest, AppliesImuOffsetAndDriverCorrectionAtActualIngress) {
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
    LivoSystem reference(options);
    options.exporter.directory /= "corrected";
    options.imu_time_offset = c.offset;
    options.driver_fix = c.driver;
    LivoSystem actual(options);
    std::vector<FrontendResult> expected, observed;
    for (int frame = 0; frame < 6; ++frame) {
      const double t = c.begin + frame * .25;
      reference.AddLidar(test::Scan(t));
      actual.AddLidar(test::Scan(t));
      for (double offset : {.0625, .125, .1875}) {
        Imu imu{t + offset, Eigen::Vector3d::Zero(), {frame * .01, 0, -9.81}};
        ASSERT_TRUE(reference.AddImu(imu));
        imu.timestamp = (t + offset + c.raw_delta);
        ASSERT_TRUE(actual.AddImu(imu));
      }
      Imu reverse{
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
