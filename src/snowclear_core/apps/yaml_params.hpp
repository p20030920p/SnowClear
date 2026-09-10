#ifndef SNOWCLEAR_APPS_YAML_PARAMS_HPP
#define SNOWCLEAR_APPS_YAML_PARAMS_HPP

// =====================================================================
// YAML parameter file -> MapParamSource
// ---------------------------------------------------------------------
// Lives in the app layer, not in the library: the algorithm core stays free
// of a YAML dependency so an embedded or ROS-only build does not have to
// pull yaml-cpp in.
//
// Nested maps are flattened with '/' and every key is normalised to its bare
// spelling, so
//
//     detector: { score_threshold: 0.75 }
//     use_idsor_intensity_threshold: true
//
// resolves the same way as the flat shipped file
// (config/snowclear_params.yaml).
// =====================================================================

#include "snowclear/param_source.hpp"

#include <string>

namespace snowclear {
namespace app {

// Merge every scalar found in `path` into `out`.
// Returns false (and logs a warning) when the file cannot be read or parsed;
// the caller decides whether that is fatal.
bool load_yaml_into(const std::string& path, MapParamSource& out);

}  // namespace app
}  // namespace snowclear

#endif  // SNOWCLEAR_APPS_YAML_PARAMS_HPP
