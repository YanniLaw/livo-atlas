#include "gaussian/wire_protocol.h"

#include <chrono>
#include <exception>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  try {
    const int fd = std::stoi(argv[1]);
    const auto message =
        livo_atlas::ReadGaussianMessage(fd, std::chrono::seconds(2));
    livo_atlas::WriteGaussianMessage(fd, message, std::chrono::seconds(2));
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
