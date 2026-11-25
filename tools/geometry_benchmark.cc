#include "geometry_benchmark.h"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

#include "livo_atlas/common/logging.h"
#include "livo_atlas/io/keyframe_store.h"
#include "livo_atlas/io/online_submap_store.h"
#include "livo_atlas/mapping/geometry_mapper.h"

namespace livo_atlas {
namespace {

using Clock = std::chrono::steady_clock;
constexpr std::uint64_t kMiB = 1024 * 1024;
constexpr std::uint64_t kHashSeed = 14695981039346656037ull;

struct Options {
  std::string dataset;
  std::uint64_t version = 0;
  std::uint64_t rounds = 5;
  std::uint64_t preload_mib = 512;
  GeometryOptions geometry;
};

struct Workload {
  Options options;
  std::shared_ptr<const PoseSnapshot> poses;
  std::vector<OnlineSubmap> groups;
  std::vector<PointCloud::ConstPtr> clouds;
  std::uint64_t preload_bytes = 0;
};

struct Sample {
  double reference_add = 0;
  double reference_snapshot = 0;
  double mapper_total = 0;
  double mapper_build = 0;
  double mapper_snapshot = 0;
  double mapper_read = 0;
  std::uint64_t reference_hash = kHashSeed;
  std::uint64_t mapper_hash = kHashSeed;
  std::uint64_t voxels = 0;
  std::uint64_t normals = 0;
};

void Usage() {
  LOG(INFO) << "Usage: livo_atlas_benchmark_geometry --dataset DIRECTORY "
               "[--pose-version N] [--rounds N] [--max-preload-mb N] "
               "[--voxel-size M] [--min-points N] [--max-plane-rms M] "
               "[--max-plane-ratio R] [--max-voxels N] [--max-points N] "
               "[--max-resident-submaps N]";
}

std::uint64_t Unsigned(const std::string& text) {
  std::uint64_t value = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(),
                                      value);
  if (result.ec != std::errc() || result.ptr != text.data() + text.size() ||
      std::to_string(value) != text)
    throw std::invalid_argument("Expected canonical unsigned integer: " + text);
  return value;
}

int Integer(const std::string& text) {
  const auto value = Unsigned(text);
  if (value > static_cast<std::uint64_t>(std::numeric_limits<int>::max()))
    throw std::invalid_argument("Integer exceeds supported range");
  return static_cast<int>(value);
}

double Number(const std::string& text) {
  std::istringstream input(text);
  input.imbue(std::locale::classic());
  double value = 0;
  if (!(input >> value) || !input.eof() || !std::isfinite(value))
    throw std::invalid_argument("Expected finite number: " + text);
  return value;
}

Options Parse(const std::vector<std::string>& arguments) {
  Options options;
  options.geometry.enabled = true;
  std::set<std::string> seen;
  for (std::size_t i = 0; i < arguments.size(); ++i) {
    const auto& name = arguments[i];
    if (i + 1 == arguments.size() || !seen.insert(name).second)
      throw std::invalid_argument("Missing value or repeated option: " + name);
    const auto& value = arguments[++i];
    if (name == "--dataset") {
      options.dataset = value;
    } else if (name == "--pose-version") {
      options.version = Unsigned(value);
    } else if (name == "--rounds") {
      options.rounds = Unsigned(value);
    } else if (name == "--max-preload-mb") {
      options.preload_mib = Unsigned(value);
    } else if (name == "--voxel-size") {
      options.geometry.voxel_size = Number(value);
    } else if (name == "--min-points") {
      options.geometry.min_points_per_voxel = Integer(value);
    } else if (name == "--max-plane-rms") {
      options.geometry.max_plane_rms = Number(value);
    } else if (name == "--max-plane-ratio") {
      options.geometry.max_plane_ratio = Number(value);
    } else if (name == "--max-voxels") {
      options.geometry.max_voxels_per_submap = Integer(value);
    } else if (name == "--max-points") {
      options.geometry.max_points_per_keyframe = Integer(value);
    } else if (name == "--max-resident-submaps") {
      options.geometry.max_resident_submaps = Integer(value);
    } else {
      throw std::invalid_argument("Unknown option: " + name);
    }
  }
  if (options.dataset.empty() || !options.rounds || options.rounds > 1000 ||
      !options.preload_mib || options.preload_mib > 65536)
    throw std::invalid_argument("Dataset, rounds or preload limit is invalid");
  ValidateGeometryOptions(options.geometry);
  return options;
}

Workload Load(const Options& options) {
  Workload workload;
  workload.options = options;
  KeyframeStore frames(options.dataset);
  OnlineSubmapStore store(std::filesystem::path(options.dataset) /
                             "online_submaps", frames);
  if (frames.ids().empty() || !store.size() ||
      store.indexed_frame_count() != frames.ids().size())
    throw std::invalid_argument("Expected a nonempty, fully indexed dataset");
  workload.poses = std::make_shared<const PoseSnapshot>(
      store.MaterializePoseSnapshot(options.version, frames.ids().size()));
  for (std::size_t id = 0; id < store.size(); ++id) {
    auto group = store.Read(id);
    if (group.state != OnlineSubmapState::kSealed)
      throw std::invalid_argument("Seal all submaps before benchmarking");
    workload.groups.push_back(std::move(group));
  }
  const auto limit = options.preload_mib * kMiB;
  for (auto id : frames.ids()) {
    const auto remaining = (limit - workload.preload_bytes) / sizeof(Point);
    if (!remaining) throw std::invalid_argument("Point preload limit exceeded");
    auto cloud = frames.ReadBodyCloud(
        id, std::min<std::uint64_t>(
                options.geometry.max_points_per_keyframe, remaining));
    const auto bytes = cloud->points.capacity() * sizeof(Point);
    if (bytes > limit - workload.preload_bytes)
      throw std::invalid_argument("Point preload limit exceeded");
    workload.preload_bytes += bytes;
    workload.clouds.push_back(std::move(cloud));
  }
  return workload;
}

RigidTransform LocalPose(const Workload& workload, const OnlineSubmap& group,
                         std::uint64_t id) {
  const auto& anchor = workload.poses->poses.at(
      group.primary.anchor_keyframe_id).parent_from_body();
  const auto& pose = workload.poses->poses.at(id).parent_from_body();
  const Eigen::Matrix3d rotation = anchor.rotation.transpose();
  const Eigen::Vector3d translation = -rotation * anchor.translation;
  return {rotation * pose.rotation, rotation * pose.translation + translation};
}

std::uint64_t Bits(double value) {
  static_assert(sizeof(double) == sizeof(std::uint64_t) &&
                    std::numeric_limits<double>::is_iec559,
                "Benchmark fingerprints require IEEE754 binary64");
  std::uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

// Canonical words encode every field, without padding or host byte order.
template <typename Emit>
void Visit(const GeometryData& data, Emit emit) {
  const auto& o = data.options;
  emit(o.enabled);
  emit(Bits(o.voxel_size));
  emit(o.min_points_per_voxel);
  emit(Bits(o.max_plane_rms));
  emit(Bits(o.max_plane_ratio));
  emit(o.max_voxels_per_submap);
  emit(o.max_points_per_keyframe);
  emit(o.max_resident_submaps);
  emit(data.input_point_count);
  emit(data.sources.size());
  for (const auto& source : data.sources) {
    emit(source.keyframe_id);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        emit(Bits(source.submap_from_body.rotation(r, c)));
    for (int r = 0; r < 3; ++r)
      emit(Bits(source.submap_from_body.translation[r]));
  }
  emit(data.voxels.size());
  for (const auto& voxel : data.voxels) {
    emit(static_cast<std::uint64_t>(voxel.key.x));
    emit(static_cast<std::uint64_t>(voxel.key.y));
    emit(static_cast<std::uint64_t>(voxel.key.z));
    emit(voxel.point_count);
    for (int r = 0; r < 3; ++r) emit(Bits(voxel.centroid[r]));
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        emit(Bits(voxel.spatial_covariance(r, c)));
    emit(Bits(voxel.mean_intensity));
    emit(voxel.normal.has_value());
    if (voxel.normal)
      for (int r = 0; r < 3; ++r) emit(Bits((*voxel.normal)[r]));
    emit(voxel.keyframe_ids.size());
    for (auto id : voxel.keyframe_ids) emit(id);
  }
}

void Hash(std::uint64_t word, std::uint64_t& hash) {
  for (int byte = 0; byte < 8; ++byte) {
    hash ^= word & 0xff;
    hash *= 1099511628211ull;
    word >>= 8;
  }
}

void Compare(const GeometryData& reference, const GeometryData& actual,
              Sample& sample) {
  std::vector<std::uint64_t> words;
  Visit(reference, [&](std::uint64_t word) {
    words.push_back(word);
    Hash(word, sample.reference_hash);
  });
  std::size_t index = 0;
  Visit(actual, [&](std::uint64_t word) {
    if (index == words.size() || word != words[index++])
      throw std::runtime_error("Geometry field/bit comparison failed");
    Hash(word, sample.mapper_hash);
  });
  if (index != words.size())
    throw std::runtime_error("Geometry field count changed");
}

double Seconds(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

Sample RunRound(const Workload& workload, bool reference_first) {
  GeometryMapper mapper(workload.options.geometry, [&](std::uint64_t id) {
    return workload.clouds.at(id);
  });
  mapper.Update(workload.groups, workload.poses);
  Sample sample;
  for (const auto& group : workload.groups) {
    std::shared_ptr<const GeometryData> reference, actual;
    const auto run_reference = [&] {
      const auto start = Clock::now();
      GeometrySubmapBuilder builder(workload.options.geometry);
      for (auto id : group.primary.keyframe_ids)
        builder.AddKeyframe(id, *workload.clouds.at(id),
                             LocalPose(workload, group, id));
      sample.reference_add += Seconds(start);
      const auto snapshot_start = Clock::now();
      reference = builder.Snapshot();
      sample.reference_snapshot += Seconds(snapshot_start);
    };
    const auto run_mapper = [&] {
      const auto start = Clock::now();
      actual = mapper.GetSubmap(group.primary.id).data;
      sample.mapper_total += Seconds(start);
      const auto stats = mapper.GetStats();
      sample.mapper_build += stats.last_full_build_seconds;
      sample.mapper_snapshot += stats.last_snapshot_seconds;
      sample.mapper_read += stats.last_cloud_read_seconds;
    };
    if (reference_first) {
      run_reference();
      run_mapper();
    } else {
      run_mapper();
      run_reference();
    }
    Compare(*reference, *actual, sample);
    sample.voxels += actual->voxels.size();
    for (const auto& voxel : actual->voxels) sample.normals += !!voxel.normal;
  }
  return sample;
}

std::string Hex(std::uint64_t value) {
  std::ostringstream out;
  out << std::hex << std::setfill('0') << std::setw(16) << value;
  return out.str();
}

void Print(const Workload& workload, const std::vector<Sample>& samples) {
  const auto& options = workload.options;
  const auto& geometry = options.geometry;
  std::cout.imbue(std::locale::classic());
  std::cout << std::setprecision(std::numeric_limits<double>::max_digits10)
            << "{\"schema_version\":1,\"geometry_matches\":true,"
               "\"pose_version\":"
            << workload.options.version << ",\"frame_count\":"
            << workload.clouds.size() << ",\"submap_count\":"
            << workload.groups.size() << ",\"preload_bytes\":"
            << workload.preload_bytes << ",\"voxel_count\":"
            << samples.front().voxels << ",\"normal_voxels\":"
            << samples.front().normals << ",\"reference_fingerprint\":\""
            << Hex(samples.front().reference_hash)
            << "\",\"mapper_fingerprint\":\""
            << Hex(samples.front().mapper_hash)
            << "\",\"max_preload_mb\":" << options.preload_mib
            << ",\"geometry\":{\"voxel_size_m\":" << geometry.voxel_size
            << ",\"min_points_per_voxel\":" << geometry.min_points_per_voxel
            << ",\"max_plane_rms\":" << geometry.max_plane_rms
            << ",\"max_plane_ratio\":" << geometry.max_plane_ratio
            << ",\"max_voxels_per_submap\":" << geometry.max_voxels_per_submap
            << ",\"max_points_per_keyframe\":"
            << geometry.max_points_per_keyframe
            << ",\"max_resident_submaps\":" << geometry.max_resident_submaps
            << "},\"samples\":[";
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const auto& s = samples[i];
    if (i) std::cout << ',';
    std::cout << "{\"round\":" << i << ",\"reference_first\":" << (i % 2 == 0)
              << ",\"reference_add_seconds\":" << s.reference_add
              << ",\"reference_snapshot_seconds\":" << s.reference_snapshot
              << ",\"mapper_total_seconds\":" << s.mapper_total
              << ",\"mapper_build_seconds\":" << s.mapper_build
              << ",\"mapper_snapshot_seconds\":" << s.mapper_snapshot
              << ",\"mapper_read_seconds\":" << s.mapper_read << '}';
  }
  std::cout << "]}\n";
}

}  // namespace

int RunGeometryBenchmark(const std::vector<std::string>& arguments) {
  if (arguments == std::vector<std::string>{"--help"}) {
    Usage();
    return 0;
  }
  Options options;
  try {
    options = Parse(arguments);
  } catch (const std::exception& error) {
    LOG(ERROR) << error.what();
    Usage();
    return 2;
  }
  try {
    const auto workload = Load(options);
    const auto warmup = RunRound(workload, true);
    std::vector<Sample> samples;
    for (std::uint64_t i = 0; i < options.rounds; ++i) {
      auto sample = RunRound(workload, i % 2 == 0);
      if (sample.reference_hash != warmup.reference_hash ||
          sample.mapper_hash != warmup.mapper_hash)
        throw std::runtime_error("Geometry changed between rounds");
      samples.push_back(sample);
    }
    Print(workload, samples);
    return 0;
  } catch (const std::exception& error) {
    LOG(ERROR) << "Geometry benchmark failed: " << error.what();
    return 1;
  }
}

}  // namespace livo_atlas
