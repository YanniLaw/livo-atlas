#include "livo_atlas/gaussian/gaussian_render_output.h"

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gaussian/render_output_test_helpers.h"

namespace livo_atlas {
namespace {
using namespace std::chrono_literals;

bool Wait(const std::function<bool()>& predicate) {
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) return true;
    std::this_thread::sleep_for(1ms);
  }
  return predicate();
}

GaussianRenderOutputOptions Enabled() {
  GaussianRenderOutputOptions result;
  result.enabled = true;
  result.minimum_interval = 100ms;
  return result;
}

TEST(GaussianRenderOutputTest, DisabledDoesNoWorkAndCreatesNoSource) {
  int calls = 0;
  GaussianRenderOutput output({}, [&](const auto&) {
    ++calls;
    return true;
  });
  output.Observe(test::ReadyRenderSnapshot());
  output.Finish();
  EXPECT_EQ(calls, 0);
  EXPECT_EQ(output.GetStats().submitted, 0u);
  EXPECT_TRUE(output.GetStats().finished);
}

TEST(GaussianRenderOutputTest, BlockedConsumerKeepsOnlyLatestPendingAndJoins) {
  std::mutex mutex;
  std::condition_variable condition;
  bool entered = false, released = false;
  std::vector<GaussianRenderSnapshot> received;
  GaussianRenderOutput output(Enabled(), [&](const auto& value) {
    std::unique_lock<std::mutex> lock(mutex);
    if (!entered) {
      entered = true;
      condition.notify_all();
      condition.wait(lock, [&] { return released; });
    }
    received.push_back(value);
    return true;
  });
  auto snapshot = test::ReadyRenderSnapshot();
  std::weak_ptr<const GaussianRender> first = snapshot.render;
  output.Observe(snapshot);
  bool started;
  {
    std::unique_lock<std::mutex> lock(mutex);
    started = condition.wait_for(lock, 3s, [&] { return entered; });
  }
  for (int i = 0; i < 100; ++i) {
    auto render = std::make_shared<GaussianRender>(*snapshot.render);
    ++render->request.job_id;
    snapshot.render = render;
    output.Observe(snapshot);
  }
  const auto blocked = output.GetStats();
  {
    std::lock_guard<std::mutex> lock(mutex);
    released = true;
    condition.notify_all();
  }
  EXPECT_TRUE(started);
  EXPECT_EQ(blocked.pending, 1u);
  EXPECT_EQ(blocked.in_flight, 1u);
  EXPECT_EQ(blocked.peak_pending, 1u);
  EXPECT_GE(blocked.coalesced, 99u);
  EXPECT_TRUE(Wait([&] { return output.GetStats().published == 2; }));
  output.Finish();
  ASSERT_EQ(received.size(), 3u);
  EXPECT_EQ(received[0].revision, 1u);
  EXPECT_EQ(received[1].revision, 101u);
  EXPECT_EQ(received[1].render->request.job_id, 9007199254741093ULL);
  EXPECT_EQ(received[0].render->capture_time, received[1].render->capture_time);
  EXPECT_EQ(received.back().status, GaussianRenderStatus::kStopped);
  EXPECT_FALSE(received.back().render);
  EXPECT_LT(received[1].revision, received.back().revision);
  EXPECT_EQ(output.GetStats().pending, 0u);
  EXPECT_EQ(output.GetStats().in_flight, 0u);
  received.clear();
  EXPECT_TRUE(first.expired());
}

TEST(GaussianRenderOutputTest, InvalidationBypassesRateAndDeduplicates) {
  auto options = Enabled();
  options.minimum_interval = 60s;
  GaussianRenderOutput output(options, [](const auto&) { return true; });
  auto snapshot = test::ReadyRenderSnapshot();
  output.Observe(snapshot);
  ASSERT_TRUE(Wait([&] { return output.GetStats().published == 1; }));
  for (int i = 0; i < 20; ++i) output.Observe(snapshot);
  EXPECT_EQ(output.GetStats().submitted, 1u);
  snapshot.status = GaussianRenderStatus::kInvalidated;
  snapshot.reason = "source_changed";
  snapshot.render.reset();
  output.Observe(snapshot);
  EXPECT_TRUE(Wait([&] { return output.GetStats().published == 2; }));
  output.Finish();
}

TEST(GaussianRenderOutputTest, FailureClearsPixelsAndTriesErrorOnlyOnce) {
  for (bool malformed : {false, true}) {
    std::vector<GaussianRenderSnapshot> received;
    GaussianRenderOutput output(Enabled(), [&](const auto& value) {
      received.push_back(value);
      if (value.status == GaussianRenderStatus::kReady)
        throw std::runtime_error("Controlled publication failure");
      return true;
    });
    auto snapshot = test::ReadyRenderSnapshot();
    if (malformed) {
      auto bad = std::make_shared<GaussianRender>(*snapshot.render);
      bad->K(0, 0) = -1;
      snapshot.render = bad;
    }
    output.Observe(snapshot);
    EXPECT_TRUE(Wait([&] { return output.GetStats().published == 1; }));
    output.Finish();
    ASSERT_FALSE(received.empty());
    EXPECT_EQ(received.back().status, GaussianRenderStatus::kError);
    EXPECT_EQ(received.back().reason,
              malformed ? "invalid_snapshot" : "publication_failed");
    EXPECT_FALSE(received.back().render);
    EXPECT_EQ(output.GetStats().failed, 1u);
    EXPECT_FALSE(output.GetStats().error.empty());
    EXPECT_EQ(received.size(), malformed ? 1u : 2u);
  }
}

TEST(GaussianRenderOutputTest, UnavailableContextIsNotCountedAsPublished) {
  GaussianRenderOutput output(Enabled(), [](const auto&) { return false; });
  output.Observe(test::ReadyRenderSnapshot());
  EXPECT_TRUE(Wait([&] { return output.GetStats().unavailable == 1; }));
  output.Finish();
  EXPECT_EQ(output.GetStats().published, 0u);
  EXPECT_EQ(output.GetStats().unavailable, 2u);
}

TEST(GaussianRenderOutputTest, FinishWithdrawsPendingWhileConsumerIsBlocked) {
  std::mutex mutex;
  std::condition_variable condition;
  bool entered = false, released = false;
  std::vector<GaussianRenderSnapshot> received;
  GaussianRenderOutput output(Enabled(), [&](const auto& value) {
    std::unique_lock<std::mutex> lock(mutex);
    if (!entered) {
      entered = true;
      condition.notify_all();
      condition.wait(lock, [&] { return released; });
    }
    received.push_back(value);
    return true;
  });
  auto first = test::ReadyRenderSnapshot();
  output.Observe(first);
  bool started;
  {
    std::unique_lock<std::mutex> lock(mutex);
    started = condition.wait_for(lock, 3s, [&] { return entered; });
  }
  auto second = first;
  auto replacement = std::make_shared<GaussianRender>(*first.render);
  ++replacement->request.job_id;
  second.render = replacement;
  output.Observe(second);
  std::thread finishing([&] { output.Finish(); });
  const bool stopping =
      Wait([&] { return output.GetStats().last_revision == 3; });
  {
    std::lock_guard<std::mutex> lock(mutex);
    released = true;
    condition.notify_all();
  }
  finishing.join();
  EXPECT_TRUE(stopping);
  EXPECT_EQ(received.size(), 2u);
  EXPECT_TRUE(started);
  ASSERT_FALSE(received.empty());
  EXPECT_EQ(received.back().status, GaussianRenderStatus::kStopped);
  const auto count = received.size();
  output.Observe(first);
  EXPECT_EQ(received.size(), count);
  EXPECT_TRUE(output.GetStats().finished);
}

TEST(GaussianRenderOutputTest, BadHeaderAfterReadyRevokesItsPixels) {
  std::vector<GaussianRenderSnapshot> received;
  GaussianRenderOutput output(Enabled(), [&](const auto& value) {
    received.push_back(value);
    return true;
  });
  auto source = test::ReadyRenderSnapshot();
  output.Observe(source);
  ASSERT_TRUE(Wait([&] { return output.GetStats().published == 1; }));
  auto invalid = std::make_shared<GaussianRender>(*source.render);
  invalid->width = 1000001;
  source.render = invalid;
  output.Observe(source);
  EXPECT_TRUE(Wait([&] { return output.GetStats().published == 2; }));
  output.Finish();
  ASSERT_EQ(received.size(), 2u);
  EXPECT_EQ(received.back().status, GaussianRenderStatus::kError);
  EXPECT_EQ(received.back().reason, "invalid_snapshot");
  EXPECT_FALSE(received.back().render);
}

TEST(GaussianRenderOutputTest, FirstBadPixelsKeepOnlyAUsableSourceHeader) {
  for (bool usable_identity : {true, false}) {
    std::vector<GaussianRenderSnapshot> received;
    GaussianRenderOutput output(Enabled(), [&](const auto& value) {
      received.push_back(value);
      return true;
    });
    auto source = test::ReadyRenderSnapshot();
    auto malformed = std::make_shared<GaussianRender>(*source.render);
    malformed->rgb.pop_back();
    source.render = malformed;
    if (!usable_identity) source.session_id = "foreign";
    output.Observe(source);
    EXPECT_TRUE(Wait([&] { return output.GetStats().failed == 1; }));
    output.Finish();
    EXPECT_EQ(received.size(), usable_identity ? 1u : 0u);
    if (usable_identity) {
      EXPECT_EQ(received.back().status, GaussianRenderStatus::kError);
      EXPECT_FALSE(received.back().render);
      EXPECT_EQ(received.back().session_id, std::string(32, 'a'));
    }
  }
}

TEST(GaussianRenderOutputTest, JoinedStatusPreservesMapperFailureCategory) {
  std::vector<GaussianRenderSnapshot> received;
  GaussianRenderOutput output(Enabled(), [&](const auto& value) {
    received.push_back(value);
    return true;
  });
  auto source = test::ReadyRenderSnapshot();
  source.render.reset();
  source.status = GaussianRenderStatus::kError;
  source.reason = "mapper_failed";
  output.Observe(source);
  output.Finish();
  ASSERT_FALSE(received.empty());
  EXPECT_EQ(received.back().status, GaussianRenderStatus::kStopped);
  EXPECT_EQ(received.back().reason, "mapper_failed");
  EXPECT_FALSE(received.back().render);
}

TEST(GaussianRenderOutputTest, ProducerRestartSeparatesRevisionDomains) {
  std::string first, second;
  auto snapshot = test::ReadyRenderSnapshot();
  {
    GaussianRenderOutput output(Enabled(), [&](const auto& value) {
      first = value.producer_id;
      return true;
    });
    output.Observe(snapshot);
    output.Finish();
  }
  {
    GaussianRenderOutput output(Enabled(), [&](const auto& value) {
      second = value.producer_id;
      return true;
    });
    output.Observe(snapshot);
    output.Finish();
  }
  EXPECT_EQ(first.size(), 32u);
  EXPECT_EQ(second.size(), 32u);
  EXPECT_NE(first, second);
}

}  // namespace
}  // namespace livo_atlas
