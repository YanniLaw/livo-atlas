#ifndef LIVO_ATLAS_SRC_GAUSSIAN_UNIX_SOCKET_H_
#define LIVO_ATLAS_SRC_GAUSSIAN_UNIX_SOCKET_H_

#include <chrono>
#include <cstddef>
#include <cstdint>

namespace livo_atlas::gaussian_detail {

using Deadline = std::chrono::steady_clock::time_point;

void ReadExact(int fd, std::uint8_t* data, std::size_t size, Deadline deadline);

void WriteExact(int fd, const std::uint8_t* data, std::size_t size,
                Deadline deadline);

}  // namespace livo_atlas::gaussian_detail

#endif  // LIVO_ATLAS_SRC_GAUSSIAN_UNIX_SOCKET_H_
