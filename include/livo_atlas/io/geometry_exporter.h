#ifndef LIVO_ATLAS_IO_GEOMETRY_EXPORTER_H_
#define LIVO_ATLAS_IO_GEOMETRY_EXPORTER_H_

#include <cstdint>
#include <filesystem>

#include "livo_atlas/mapping/geometry_submap.h"

namespace livo_atlas {

// Exports a stopped online dataset. The source and existing outputs are never
// modified; an incomplete attempt remains in a sibling pending directory.
void ExportGeometry(const std::filesystem::path& dataset,
                    const std::filesystem::path& output,
                    std::uint64_t pose_version, const GeometryOptions& options);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_IO_GEOMETRY_EXPORTER_H_
