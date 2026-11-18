#ifndef LIVO_ATLAS_TESTS_GAUSSIAN_SUBMAP_PEER_H_
#define LIVO_ATLAS_TESTS_GAUSSIAN_SUBMAP_PEER_H_

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <mutex>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gaussian/wire_protocol.h"
#include "gaussian/render_test_helpers.h"

namespace livo_atlas {
namespace test {
namespace fs = std::filesystem;
using namespace std::chrono_literals;

inline std::string Text(const cv::FileNode& node) {
  return static_cast<std::string>(node);
}

// This peer exercises actual framing and files, while controlling only timings
// and returned model telemetry. Real arrays and CUDA are tested by Python.
class SubmapPeer {
 public:
  explicit SubmapPeer(const fs::path& path) : path_(path) {
    listener_ = socket(AF_UNIX, SOCK_STREAM, 0);
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    const auto name = path.string();
    std::copy(name.begin(), name.end(), address.sun_path);
    if (listener_ < 0 ||
        bind(listener_, reinterpret_cast<sockaddr*>(&address),
             sizeof(address)) ||
        listen(listener_, 1))
      throw std::runtime_error("Cannot start submap peer");
    thread_ = std::thread(&SubmapPeer::Run, this);
  }

  ~SubmapPeer() {
    stopping = true;
    if (client_ >= 0) shutdown(client_, SHUT_RDWR);
    shutdown(listener_, SHUT_RDWR);
    if (thread_.joinable()) thread_.join();
    if (client_ >= 0) close(client_);
    close(listener_);
    fs::remove(path_);
  }

  std::vector<std::string> Events() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return events_;
  }

  std::atomic<bool> hold_candidate{false}, hold_commit{false},
      hold_checkpoint{false}, hold_map_ack{false}, hold_empty{false},
      corrupt_checkpoint{false}, wrong_stop{false}, ineffective_training{false};
  std::atomic<bool> hold_render{false}, oom_render{false},
      no_render_capability{false};
  std::atomic<int> renders{0};
  std::atomic<std::uint64_t> render_job{0};
  std::atomic<int> views{0}, checkpoints{0}, acknowledgements{0}, prunes{0},
      discards{0}, reuses{0}, rebuilds{0}, commits{0}, decisions{0};
  std::atomic<bool> stopping{false};
  std::string Error() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return error_;
  }

 private:
  void Hold(const std::atomic<bool>& flag) {
    while (flag && !stopping) std::this_thread::sleep_for(1ms);
  }

  void Run() {
    client_ = accept(listener_, nullptr, nullptr);
    try {
      std::string session;
      fs::path states;
      std::vector<std::string> confirmed, proposed;
      std::string source_generation, content_job;
      for (;;) {
        const auto message = ReadGaussianMessage(client_, 30s);
        cv::FileStorage input(message.header_json,
                              cv::FileStorage::READ | cv::FileStorage::MEMORY);
        const auto kind = Text(input["type"]);
        cv::FileStorage out(".json", cv::FileStorage::WRITE |
                                         cv::FileStorage::MEMORY |
                                         cv::FileStorage::FORMAT_JSON);
        if (kind == "hello") session = Text(input["session_id"]);
        out << "schema" << 2 << "session_id" << session;
        for (const auto* field :
             {"job_id", "generation", "pose_version", "submap_id", "operation"})
          if (!input[field].empty())
            out << field
                << (kind == "stop" && wrong_stop &&
                            std::string(field) == "generation"
                        ? "999999"
                        : Text(input[field]));
        {
          std::lock_guard<std::mutex> lock(mutex_);
          events_.push_back(
              kind + ":" +
              (input["submap_id"].empty() ? "" : Text(input["submap_id"])));
        }
        if (kind == "hello") {
          if (!input["capabilities"].empty() && !no_render_capability)
            out << "capabilities" << "[" << "joint_render_v1" << "]";
          states = Text(input["state_directory"]);
          fs::create_directory(states);
          out << "type"
              << "hello_ok"
              << "mode"
              << "persistent_submaps"
              << "state_schema" << 1;
          for (const auto* key :
               {"python", "torch", "gsplat", "cuda", "device"})
            out << key << "controlled";
          out << "gpu_free_bytes"
              << "5000000000"
              << "gpu_total_bytes"
              << "8000000000";
        } else if (kind == "joint_render") {
          render_job = std::stoull(Text(input["job_id"]));
          ++renders;
          Hold(hold_render);
          WriteGaussianMessage(client_, RenderReply(message, oom_render), 3s);
          continue;
        } else if (kind == "activate" || kind == "view") {
          const auto action = kind == "view" ? "view" : Text(input["action"]);
          if (action == "empty") {
            Hold(hold_empty);
            confirmed.clear();
            out << "type"
                << "activated";
          } else {
            proposed = confirmed;
            if (action == "view") {
              ++views;
              proposed.push_back(Text(input["keyframe_id"]));
              if (proposed.size() > 10) proposed.erase(proposed.begin());
            } else {
              if (!input["asset_id"].empty()) {
                cv::FileStorage saved(
                    (states / Text(input["asset_id"]) / "peer.json").string(),
                    cv::FileStorage::READ);
                proposed.clear();
                for (const auto& id : saved["frames"])
                  proposed.push_back(Text(id));
              }
              if (action == "reuse")
                ++reuses;
              else
                ++rebuilds;
              out << "operation" << action;
            }
            if (action == "view")
              out << "operation"
                  << "view";
            if (kind == "activate") {
              pending_source_ = Text(input["source_generation"]);
              pending_content_ = Text(input["content_job_id"]);
            }
            Hold(hold_candidate);
            out << "type"
                << "candidate"
                << "frame_ids"
                << "[";
            for (const auto& id : proposed) out << id;
            out << "]";
            for (const auto* key : {"membership_revision", "frame_count",
                                    "optimized_frame_count"})
              out << key << Text(input[key]);
            const int width =
                kind == "view" ? static_cast<int>(input["width"]) : 8;
            const int height =
                kind == "view" ? static_cast<int>(input["height"]) : 6;
            out << "width" << width << "height" << height << "steps"
                << (action == "reuse"
                        ? 0
                        : action == "view"
                              ? 20
                              : 20 * static_cast<int>(proposed.size()))
                << "gaussian_count" << static_cast<int>(proposed.size())
                << "supervised_depth_pixels"
                << (action == "reuse" || ineffective_training ? 0 : 20)
                << "rgb_l1" << .2 << "depth_l1_m" << .1 << "nonzero_gradient"
                << (action == "reuse" || ineffective_training ? 0 : 1)
                << "parameter_updated"
                << (action == "reuse" || ineffective_training ? 0 : 1)
                << "capture_time" << 100.;
            WriteGaussianMessage(
                client_,
                {out.releaseAndGetString(),
                 std::vector<std::uint8_t>(width * height * 3, 40)},
                3s);
            continue;
          }
        } else if (kind == "decision") {
          ++decisions;
          Hold(hold_commit);
          if (Text(input["action"]) == "accept") {
            confirmed = proposed;
            ++commits;
            if (Text(input["operation"]) != "reuse") {
              source_generation = Text(input["generation"]);
              content_job = Text(input["job_id"]);
            } else if (!pending_source_.empty()) {
              source_generation = pending_source_;
              content_job = pending_content_;
            }
            out << "type"
                << "commit_ok"
                << "source_generation" << source_generation << "content_job_id"
                << content_job << "frame_ids"
                << "[";
            for (const auto& id : confirmed) out << id;
            out << "]"
                << "gaussian_count" << static_cast<int>(confirmed.size());
          } else {
            ++discards;
            out << "type"
                << "discard_ok";
          }
        } else if (kind == "checkpoint") {
          ++checkpoints;
          Hold(hold_checkpoint);
          const auto asset = Text(input["asset_id"]);
          fs::create_directory(states / asset);
          cv::FileStorage saved((states / asset / "peer.json").string(),
                                cv::FileStorage::WRITE);
          saved << "frames"
                << "[";
          for (const auto& id : confirmed) saved << id;
          saved << "]";
          saved.release();
          out << "type"
              << "checkpoint_ok"
              << "asset_id" << asset << "source_generation"
              << Text(input["source_generation"]) << "content_job_id"
              << (corrupt_checkpoint ? "999999" : Text(input["content_job_id"]))
              << "manifest_sha256" << std::string(64, 'b') << "manifest_bytes"
              << "2048"
              << "gaussian_bytes"
              << "1024"
              << "observation_bytes"
              << "4096"
              << "frame_ids"
              << "[";
          for (const auto& id : confirmed) out << id;
          out << "]"
              << "gaussian_count" << static_cast<int>(confirmed.size());
        } else if (kind == "map_committed") {
          if (input["asset_ids"].size()) Hold(hold_map_ack);
          ++acknowledgements;
          out << "type"
              << "map_committed_ok"
              << "map_sequence" << Text(input["map_sequence"]) << "asset_ids"
              << "[";
          for (const auto& id : input["asset_ids"]) out << Text(id);
          out << "]";
        } else if (kind == "prune") {
          ++prunes;
          fs::remove_all(states / Text(input["asset_id"]));
          out << "type"
              << "prune_ok"
              << "map_sequence" << Text(input["map_sequence"]) << "asset_id"
              << Text(input["asset_id"]);
        } else if (kind == "stop") {
          out << "type"
              << "stopped"
              << "map_sequence" << Text(input["map_sequence"]);
          if (!input["incomplete"].empty())
            out << "incomplete" << static_cast<int>(input["incomplete"]);
          WriteGaussianMessage(client_, {out.releaseAndGetString(), {}}, 3s);
          return;
        } else {
          throw std::runtime_error("Unexpected request " + kind);
        }
        if (kind == "activate" && Text(input["action"]) != "empty") {
          pending_source_ = Text(input["source_generation"]);
          pending_content_ = Text(input["content_job_id"]);
        }
        WriteGaussianMessage(client_, {out.releaseAndGetString(), {}}, 3s);
      }
    } catch (const std::exception& exception) {
      if (!stopping) {
        std::lock_guard<std::mutex> lock(mutex_);
        error_ = exception.what();
      }
    }
  }

  fs::path path_;
  int listener_ = -1;
  std::atomic<int> client_{-1};
  mutable std::mutex mutex_;
  std::vector<std::string> events_;
  std::string pending_source_, pending_content_, error_;
  std::thread thread_;
};

}  // namespace test
}  // namespace livo_atlas
#endif  // LIVO_ATLAS_TESTS_GAUSSIAN_SUBMAP_PEER_H_
