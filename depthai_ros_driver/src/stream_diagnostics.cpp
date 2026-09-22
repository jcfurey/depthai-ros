#include "depthai_ros_driver/stream_diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace depthai_ros_driver {
StreamDiagnostics::StreamDiagnostics(const std::shared_ptr<rclcpp::Node>& node, const std::string& name, std::function<bool()> demanded)
    : name(name), demanded(std::move(demanded)) {
    auto parameter = [&](const std::string& key, double value) {
        if(!node->has_parameter(key)) node->declare_parameter<double>(key, value);
        return node->get_parameter(key).as_double();
    };
    warningAgeMs = parameter("diagnostics.max_frame_age_ms", 200.0);
    errorAgeMs = parameter("diagnostics.error_frame_age_ms", 1000.0);
    timeoutSeconds = parameter("diagnostics.stream_timeout_seconds", 2.0);
    if(!std::isfinite(warningAgeMs) || !std::isfinite(errorAgeMs) || !std::isfinite(timeoutSeconds) || warningAgeMs <= 0 || errorAgeMs < warningAgeMs
       || timeoutSeconds <= 0)
        throw std::invalid_argument("Invalid stream diagnostic thresholds");
    updater = std::make_unique<diagnostic_updater::Updater>(node);
    updater->setHardwareID(node->get_fully_qualified_name());
    updater->add(name, [this](diagnostic_updater::DiagnosticStatusWrapper& status) { report(status); });
}
StreamDiagnostics::~StreamDiagnostics() {
    updater->removeByName(name);
}
void StreamDiagnostics::record(std::chrono::steady_clock::time_point captured, int64_t sequence) {
    const auto now = std::chrono::steady_clock::now();
    const auto age = std::chrono::duration<double, std::milli>(now - captured).count();
    std::lock_guard<std::mutex> lock(mutex);
    ++observed;
    if(previousSequence >= 0 && sequence > previousSequence + 1) sequenceGaps += sequence - previousSequence - 1;
    previousSequence = sequence;
    samples.emplace_back(now, std::max(0.0, age));
    if(samples.size() > 128) samples.pop_front();
}
void StreamDiagnostics::report(diagnostic_updater::DiagnosticStatusWrapper& status) {
    std::lock_guard<std::mutex> lock(mutex);
    using Status = diagnostic_msgs::msg::DiagnosticStatus;
    if(!demanded()) {
        samples.clear();
        previousSequence = -1;
        started = std::chrono::steady_clock::now();
        status.summary(Status::OK, "Idle: inactive pipeline or no subscribers");
        return;
    }
    const auto silence = std::chrono::duration<double>(std::chrono::steady_clock::now() - (samples.empty() ? started : samples.back().first)).count();
    std::vector<double> ages;
    for(const auto& sample : samples) ages.push_back(sample.second);
    std::sort(ages.begin(), ages.end());
    const double median = ages.empty() ? 0 : ages[(ages.size() - 1) / 2];
    const double p95 = ages.empty() ? 0 : ages[static_cast<size_t>(0.95 * (ages.size() - 1))];
    double fps = 0;
    if(samples.size() > 1) {
        const auto elapsed = std::chrono::duration<double>(samples.back().first - samples.front().first).count();
        if(elapsed > 0) fps = (samples.size() - 1) / elapsed;
    }
    if(silence > timeoutSeconds)
        status.summary(Status::ERROR, "No recent frames");
    else if(p95 > errorAgeMs)
        status.summary(Status::ERROR, "Excessive frame age");
    else if(p95 > warningAgeMs)
        status.summary(Status::WARN, "Frames are delayed");
    else
        status.summary(Status::OK, samples.empty() ? "Waiting for first frame" : "Streaming");
    status.add("observed_fps", fps);
    status.add("capture_to_conversion_p50_ms", median);
    status.add("capture_to_conversion_p95_ms", p95);
    status.add("seconds_since_last_frame", silence);
    status.add("frames_observed", observed);
    status.add("observed_sequence_gaps", sequenceGaps);
}
}  // namespace depthai_ros_driver
