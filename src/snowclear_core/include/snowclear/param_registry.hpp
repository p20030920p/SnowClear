#ifndef SNOWCLEAR_PARAM_REGISTRY_HPP
#define SNOWCLEAR_PARAM_REGISTRY_HPP

// =====================================================================
// Parameter registry
// ---------------------------------------------------------------------
// Every parameter the core reads, together with the C++ type it is stored
// as. The body of param_registry() is generated from the loaders by
// tools/gen_param_map.py, so this list cannot drift from what the algorithm
// actually consumes — adding a parameter without registering it fails the
// generator's --check.
//
// The ROS 2 node walks this list to declare its parameters, which is what
// makes `ros2 param list` / `ros2 param get` show the complete configuration
// rather than a hand-maintained subset.
// =====================================================================

#include <string>
#include <vector>

namespace snowclear {

enum class ParamType { kBool, kInt, kDouble, kString };

struct ParamSpec {
    std::string name;
    ParamType type;
};

const std::vector<ParamSpec>& param_registry();

}  // namespace snowclear

#endif  // SNOWCLEAR_PARAM_REGISTRY_HPP
