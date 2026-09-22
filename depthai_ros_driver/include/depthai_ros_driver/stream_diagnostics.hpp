#pragma once

#include <chrono>
#include <deque>
#include <functional>
#include <mutex>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
class StreamDiagnostics {
   public:
    StreamDiagnostics(const std::shared_ptr<rclcpp::Node>& node, const std::string& name, std::function<bool()> demanded);
    ~StreamDiagnostics();
    void record(std::chrono::steady_clock::time_point captured, int64_t sequence);

   private:
    friend class StreamDiagnosticsTestAccess;
    void report(diagnostic_updater::DiagnosticStatusWrapper& status);
    std::string name;
    std::function<bool()> demanded;
    std::unique_ptr<diagnostic_updater::Updater> updater;
    std::mutex mutex;
    std::deque<std::pair<std::chrono::steady_clock::time_point, double>> samples;
    std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
    int64_t previousSequence = -1;
    uint64_t observed = 0, sequenceGaps = 0;
    double warningAgeMs, errorAgeMs, timeoutSeconds;
};
}  // namespace depthai_ros_driver
