#pragma once

#include "path_analysis/recording.h"

#include <filesystem>
#include <istream>

namespace path_analysis {

// Required keys: path, robot, cleaning_gadget. Each point has two finite numbers.
// Path may be empty; robot needs at least three vertices; gadget needs exactly two
// distinct endpoints. Extra keys are ignored. Preserve point order and duplicates.
// Invalid JSON or geometry throws std::runtime_error with a useful message.
Recording read_recording(std::istream& input);

// Open the supplied file and call read_recording; I/O errors throw std::runtime_error.
Recording load_recording(const std::filesystem::path& filename);

} // namespace path_analysis
