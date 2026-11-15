#include "gaussian/unix_socket.h"

#include <poll.h>
#include <sys/socket.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <climits>
#include <stdexcept>
#include <system_error>

namespace livo_atlas::gaussian_detail {
namespace {

void Wait(int fd, short events, Deadline deadline) {
  while (true) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) throw std::runtime_error("Socket transfer timed out");
    const auto remaining =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    const auto milliseconds = std::max<std::int64_t>(1, remaining.count());
    pollfd descriptor{fd, events, 0};
    const int result =
        poll(&descriptor, 1,
             static_cast<int>(std::min<std::int64_t>(
                 milliseconds, static_cast<std::int64_t>(INT_MAX))));
    if (result > 0) {
      if (descriptor.revents & POLLNVAL) {
        throw std::runtime_error("Invalid socket");
      }
      return;
    }
    if (result < 0 && errno != EINTR) {
      throw std::system_error(errno, std::generic_category(), "poll");
    }
  }
}

}  // namespace

void ReadExact(int fd, std::uint8_t* data, std::size_t size,
               Deadline deadline) {
  std::size_t offset = 0;
  while (offset < size) {
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error("Socket transfer timed out");
    }
    const auto received = recv(fd, data + offset, size - offset, MSG_DONTWAIT);
    if (received > 0) {
      offset += static_cast<std::size_t>(received);
    } else if (received == 0) {
      throw std::runtime_error("Socket closed during message");
    } else if (errno == EINTR) {
      continue;
    } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
      Wait(fd, POLLIN, deadline);
    } else {
      throw std::system_error(errno, std::generic_category(), "recv");
    }
  }
}

void WriteExact(int fd, const std::uint8_t* data, std::size_t size,
                Deadline deadline) {
  std::size_t offset = 0;
  while (offset < size) {
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error("Socket transfer timed out");
    }
    const auto sent =
        send(fd, data + offset, size - offset, MSG_DONTWAIT | MSG_NOSIGNAL);
    if (sent > 0) {
      offset += static_cast<std::size_t>(sent);
    } else if (sent == 0) {
      throw std::runtime_error("Socket closed during message");
    } else if (errno == EINTR) {
      continue;
    } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
      Wait(fd, POLLOUT, deadline);
    } else {
      throw std::system_error(errno, std::generic_category(), "send");
    }
  }
}

}  // namespace livo_atlas::gaussian_detail
