#pragma once

#include <string>
#include <vector>

namespace depthai_bridge {
// Parse quoted arguments without shell expansion, substitution, or globbing.
std::vector<std::string> splitArguments(const std::string& arguments);
// Execute argv directly and collect stdout. Nonzero exit/timeout is an error.
std::string runProcess(const std::vector<std::string>& arguments);
}  // namespace depthai_bridge
