#ifndef LIVO_ATLAS_SRC_GAUSSIAN_WIRE_PROTOCOL_H_
#define LIVO_ATLAS_SRC_GAUSSIAN_WIRE_PROTOCOL_H_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace livo_atlas {

struct GaussianMessage {
  std::string header_json;
  std::vector<std::uint8_t> body;
};

std::vector<std::uint8_t> EncodeGaussianMessage(const GaussianMessage& message);

GaussianMessage ReadGaussianMessage(int fd, std::chrono::milliseconds timeout);

void WriteGaussianMessage(int fd, const GaussianMessage& message,
                          std::chrono::milliseconds timeout);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SRC_GAUSSIAN_WIRE_PROTOCOL_H_
