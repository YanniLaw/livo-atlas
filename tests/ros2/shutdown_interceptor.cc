#include <dlfcn.h>
#include <fcntl.h>
#include <rcl/context.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <thread>

// A test-only gate at the real middleware shutdown boundary. The parent sends
// a second signal here, after application work but before context cleanup.
extern "C" rcl_ret_t rcl_shutdown(rcl_context_t* context) {
  const char* entered = std::getenv("LIVO_SHUTDOWN_ENTERED_FILE");
  const char* release = std::getenv("LIVO_SHUTDOWN_RELEASE_FILE");
  if (entered && release) {
    const int fd = open(entered, O_WRONLY | O_CREAT | O_EXCL, 0600);
    if (fd >= 0) {
      close(fd);
      const auto deadline = std::chrono::steady_clock::now() +
                            std::chrono::seconds(3);
      while (access(release, F_OK) != 0 &&
             std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }
  using Shutdown = rcl_ret_t (*)(rcl_context_t*);
  const auto shutdown =
      reinterpret_cast<Shutdown>(dlsym(RTLD_NEXT, "rcl_shutdown"));
  return shutdown ? shutdown(context) : RCL_RET_ERROR;
}
