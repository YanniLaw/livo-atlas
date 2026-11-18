#ifndef LIVO_ATLAS_IO_GAUSSIAN_RENDER_WRITER_H_
#define LIVO_ATLAS_IO_GAUSSIAN_RENDER_WRITER_H_

#include <filesystem>

#include "livo_atlas/gaussian/gaussian_render.h"

namespace livo_atlas {

// Publish one new historical pair; the final JSON rename marks completion.
// Retention belongs to the serialized I/O owner, which knows its own files.
// A directory must have one publisher. Serialize every write and retention
// operation for it, including direct SDK calls from other threads/processes.
void WriteGaussianRender(const std::filesystem::path& directory,
                         const GaussianRender& render);

}  // namespace livo_atlas
#endif  // LIVO_ATLAS_IO_GAUSSIAN_RENDER_WRITER_H_
