#include "gaussian/wire_protocol.h"

#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace livo_atlas {
namespace {

GaussianMessage View() {
  GaussianMessage message;
  message.header_json =
      R"({"schema":1,"type":"view",)"
      R"("session_id":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",)"
      R"("job_id":"2147483649","generation":"1",)"
      R"("submap_id":"0","pose_version":"0",)"
      R"("membership_revision":"1",)"
      R"("frame_count":"2147483650",)"
      R"("optimized_frame_count":"0",)"
      R"("keyframe_id":"2147483649",)"
      R"("calibration_id":"0","parent_frame":"odom",)"
      R"("state_timestamp":100.0,"cloud_timestamp":99.99,)"
      R"("image_timestamp":100.01,"inverse_exposure_time":1.0,)"
      R"("width":8,"height":6,"fx":4.0,"fy":4.0,)"
      R"("cx":3.5,"cy":2.5,)"
      R"("camera_from_submap":[0,-1,0,1,1,0,0,2,0,0,1,3,0,0,0,1],)"
      R"("parent_from_submap":[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]})";
  constexpr std::size_t kPixels = 48;
  message.body.resize(kPixels * 9);
  for (std::size_t i = 0; i < kPixels; ++i) {
    message.body[3 * i] = 10;
    message.body[3 * i + 1] = 20;
    message.body[3 * i + 2] = 30;
    message.body[3 * kPixels + i] = 1;
  }
  const auto index = 3 * 8 + 4;
  message.body[4 * kPixels + index] = 1;
  const auto depth = 5 * kPixels + 4 * index;
  message.body[depth + 2] = 0x80;
  message.body[depth + 3] = 0x40;
  return message;
}

void Replace(std::string* text, const std::string& from,
             const std::string& to) {
  const auto position = text->find(from);
  ASSERT_NE(position, std::string::npos);
  text->replace(position, from.size(), to);
}

class SocketPair {
 public:
  SocketPair() { EXPECT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fd_), 0); }

  ~SocketPair() {
    if (fd_[0] >= 0) close(fd_[0]);
    if (fd_[1] >= 0) close(fd_[1]);
  }

  int reader() const { return fd_[0]; }

  int writer() const { return fd_[1]; }

  void CloseReader() {
    close(fd_[0]);
    fd_[0] = -1;
  }

 private:
  int fd_[2] = {-1, -1};
};

TEST(GaussianWireProtocolTest, PreservesViewBytesAndLargeIdsOverSocket) {
  const auto expected = View();
  const auto encoded = EncodeGaussianMessage(expected);
  ASSERT_GE(encoded.size(), 12u);
  EXPECT_EQ(encoded[4], 0xb0);  // 432 bytes, little-endian u64.
  EXPECT_EQ(encoded[5], 0x01);
  SocketPair sockets;
  WriteGaussianMessage(sockets.writer(), expected, std::chrono::seconds(1));
  const auto actual =
      ReadGaussianMessage(sockets.reader(), std::chrono::seconds(1));
  EXPECT_EQ(actual.header_json, expected.header_json);
  EXPECT_EQ(actual.body, expected.body);
}

TEST(GaussianWireProtocolTest, RejectsMalformedOptionalTelemetry) {
  GaussianMessage message;
  message.header_json =
      R"({"schema":1,"type":"candidate",)"
      R"("session_id":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",)"
      R"("job_id":"1","generation":"1","submap_id":"0",)"
      R"("pose_version":"0","membership_revision":"1",)"
      R"("frame_count":"1","optimized_frame_count":"0",)"
      R"("width":1,"height":1,"frame_ids":["0"],"steps":20,)"
      R"("rgb_l1":0.2,"depth_l1_m":0.1,"supervised_depth_pixels":1,)"
      R"("gaussian_count":1,"nonzero_gradient":2})";
  message.body = {1, 2, 3};
  EXPECT_THROW(EncodeGaussianMessage(message), std::exception);
  Replace(&message.header_json, "\"nonzero_gradient\":2",
          "\"gpu_peak_reserved_bytes\":\"-1\"");
  EXPECT_THROW(EncodeGaussianMessage(message), std::exception);
}

TEST(GaussianWireProtocolTest, RejectsInvalidMasksDepthDimensionsAndSession) {
  auto message = View();
  message.body[3 * 48] = 255;
  EXPECT_THROW(EncodeGaussianMessage(message), std::exception);
  message = View();
  message.body[3 * 48 + 28] = 0;
  EXPECT_THROW(EncodeGaussianMessage(message), std::exception);
  message = View();
  const auto depth = 5 * 48 + 4 * 28;
  message.body[depth + 2] = 0xc0;
  message.body[depth + 3] = 0x7f;
  EXPECT_THROW(EncodeGaussianMessage(message), std::exception);
  message = View();
  Replace(&message.header_json, "\"width\":8", "\"width\":2147483647");
  EXPECT_THROW(EncodeGaussianMessage(message), std::exception);
  message = View();
  Replace(&message.header_json, "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", "bad");
  EXPECT_THROW(EncodeGaussianMessage(message), std::exception);
}

TEST(GaussianWireProtocolTest, RejectsTruncatedAndOversizedFrames) {
  for (const auto& bytes : std::vector<std::vector<std::uint8_t>>{
           {1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
           {1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0},
           {10, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, '{', '}'},
           {1, 2}}) {
    SocketPair sockets;
    ASSERT_EQ(send(sockets.writer(), bytes.data(), bytes.size(), MSG_NOSIGNAL),
              static_cast<ssize_t>(bytes.size()));
    ASSERT_EQ(shutdown(sockets.writer(), SHUT_WR), 0);
    EXPECT_THROW(
        ReadGaussianMessage(sockets.reader(), std::chrono::milliseconds(500)),
        std::exception);
  }
}

TEST(GaussianWireProtocolTest, RejectsJsonWithEmbeddedNullAndTrailingBytes) {
  GaussianMessage message;
  message.header_json =
      R"({"schema":1,"type":"hello","session_id":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"})";
  message.header_json.push_back('\0');
  message.header_json += "invalid";
  EXPECT_THROW(EncodeGaussianMessage(message), std::exception);
}

TEST(GaussianWireProtocolTest, RejectsInvalidUtf8AndTrailingJson) {
  GaussianMessage message;
  message.header_json =
      R"({"schema":1,"type":"hello","session_id":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"})";
  message.header_json += R"( {"extra":1})";
  EXPECT_THROW(EncodeGaussianMessage(message), std::exception);
  message.header_json =
      R"({"schema":1,"type":"hello","session_id":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","note":")";
  message.header_json.push_back(static_cast<char>(0xc3));
  message.header_json.push_back('(');
  message.header_json += R"("})";
  EXPECT_THROW(EncodeGaussianMessage(message), std::exception);
}

TEST(GaussianWireProtocolTest, RejectsDuplicateJsonKeys) {
  GaussianMessage message;
  message.header_json =
      R"({"schema":1,"type":"hello","type":"stop","session_id":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"})";
  EXPECT_THROW(EncodeGaussianMessage(message), std::exception);
}

TEST(GaussianWireProtocolTest, TimeoutAndClosedPeerFailWithoutSigpipe) {
  SocketPair sockets;
  EXPECT_THROW(
      ReadGaussianMessage(sockets.reader(), std::chrono::milliseconds(30)),
      std::exception);
  sockets.CloseReader();
  EXPECT_THROW(WriteGaussianMessage(sockets.writer(), View(),
                                    std::chrono::milliseconds(500)),
               std::exception);
}

TEST(GaussianWireProtocolTest, ReadsFragmentedFramesAndTimesOutOnFullSocket) {
  SocketPair sockets;
  const auto expected = View();
  const auto bytes = EncodeGaussianMessage(expected);
  std::thread writer([&] {
    for (const auto byte : bytes) {
      if (send(sockets.writer(), &byte, 1, MSG_NOSIGNAL) != 1) return;
    }
  });
  GaussianMessage actual;
  try {
    actual = ReadGaussianMessage(sockets.reader(), std::chrono::seconds(2));
  } catch (...) {
    shutdown(sockets.reader(), SHUT_RDWR);
    writer.join();
    throw;
  }
  writer.join();
  EXPECT_EQ(actual.header_json, expected.header_json);
  EXPECT_EQ(actual.body, expected.body);

  SocketPair full;
  const int buffer_bytes = 4096;
  ASSERT_EQ(setsockopt(full.writer(), SOL_SOCKET, SO_SNDBUF, &buffer_bytes,
                       sizeof(buffer_bytes)),
            0);
  std::vector<std::uint8_t> fill(4096, 0);
  while (send(full.writer(), fill.data(), fill.size(),
              MSG_DONTWAIT | MSG_NOSIGNAL) > 0) {
  }
  EXPECT_THROW(WriteGaussianMessage(full.writer(), expected,
                                    std::chrono::milliseconds(30)),
               std::exception);
}

}  // namespace
}  // namespace livo_atlas
