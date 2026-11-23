#include "common/thread_pool.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>

#include "common/task.h"

namespace livo_atlas {
namespace {
using namespace std::chrono_literals;

TEST(ThreadPoolTest, ReadyTasksRunTogetherBeforeDependentTask) {
  std::mutex mutex;
  std::condition_variable condition;
  int running = 0;
  std::promise<void> release;
  const auto released = release.get_future().share();
  std::atomic<int> completed{0};
  std::promise<int> dependent_result;
  auto result = dependent_result.get_future();
  ThreadPool pool(2);

  const auto work = [&] {
    {
      std::lock_guard<std::mutex> lock(mutex);
      ++running;
    }
    condition.notify_one();
    released.wait();
    ++completed;
  };
  auto first = std::make_unique<Task>();
  first->SetWorkItem(work);
  auto second = std::make_unique<Task>();
  second->SetWorkItem(work);
  auto dependent = std::make_unique<Task>();
  dependent->AddDependency(pool.Schedule(std::move(first)));
  dependent->AddDependency(pool.Schedule(std::move(second)));
  dependent->SetWorkItem([&] { dependent_result.set_value(completed.load()); });
  pool.Schedule(std::move(dependent));

  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(condition.wait_for(lock, 5s, [&] { return running == 2; }));
  }
  EXPECT_EQ(result.wait_for(0s), std::future_status::timeout);
  release.set_value();
  ASSERT_EQ(result.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(result.get(), 2);
}

}  // namespace
}  // namespace livo_atlas
