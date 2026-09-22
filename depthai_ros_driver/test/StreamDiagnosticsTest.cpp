#include <gtest/gtest.h>

#include "depthai_ros_driver/stream_diagnostics.hpp"

namespace depthai_ros_driver {
class StreamDiagnosticsTestAccess {
   public:
    static diagnostic_msgs::msg::DiagnosticStatus report(StreamDiagnostics& diagnostics) {
        diagnostic_updater::DiagnosticStatusWrapper status;
        diagnostics.report(status);
        return status;
    }
    static void expire(StreamDiagnostics& diagnostics) {
        diagnostics.started = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    }
};
class StreamDiagnosticsTest : public testing::Test {
   protected:
    void SetUp() override {
        context = std::make_shared<rclcpp::Context>();
        context->init(0, nullptr);
        node = std::make_shared<rclcpp::Node>(
            "stream_health_test",
            rclcpp::NodeOptions().context(context).parameter_overrides(
                {rclcpp::Parameter("diagnostics.max_frame_age_ms", 10.0), rclcpp::Parameter("diagnostics.error_frame_age_ms", 100.0)}));
        diagnostics = std::make_unique<StreamDiagnostics>(node, "image", [this]() { return demanded; });
    }
    void TearDown() override {
        diagnostics.reset();
        node.reset();
        context->shutdown("test complete");
    }
    std::shared_ptr<rclcpp::Context> context;
    std::shared_ptr<rclcpp::Node> node;
    std::unique_ptr<StreamDiagnostics> diagnostics;
    bool demanded = true;
};
TEST_F(StreamDiagnosticsTest, ReportsDelayedFramesAndObservedSequenceGaps) {
    const auto captured = std::chrono::steady_clock::now() - std::chrono::milliseconds(50);
    diagnostics->record(captured, 1);
    diagnostics->record(captured, 3);
    const auto status = StreamDiagnosticsTestAccess::report(*diagnostics);
    EXPECT_EQ(status.level, status.WARN);
    auto value = [&](const std::string& key) {
        for(const auto& entry : status.values)
            if(entry.key == key) return std::stod(entry.value);
        ADD_FAILURE() << "Missing diagnostic " << key;
        return -1.0;
    };
    EXPECT_GE(value("capture_to_conversion_p50_ms"), 50);
    EXPECT_EQ(value("observed_sequence_gaps"), 1);
    EXPECT_EQ(value("frames_observed"), 2);
}
TEST_F(StreamDiagnosticsTest, DistinguishesIdleFromMissingFrames) {
    StreamDiagnosticsTestAccess::expire(*diagnostics);
    auto status = StreamDiagnosticsTestAccess::report(*diagnostics);
    EXPECT_EQ(status.level, status.ERROR);
    demanded = false;
    status = StreamDiagnosticsTestAccess::report(*diagnostics);
    EXPECT_EQ(status.level, status.OK);
    demanded = true;
    status = StreamDiagnosticsTestAccess::report(*diagnostics);
    EXPECT_EQ(status.level, status.OK);  // A fresh subscriber gets a first-frame grace period.
}
TEST_F(StreamDiagnosticsTest, BoundedWindowRecoversAfterOldFrames) {
    diagnostics->record(std::chrono::steady_clock::now() - std::chrono::seconds(1), 0);
    EXPECT_EQ(StreamDiagnosticsTestAccess::report(*diagnostics).level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    for(int i = 1; i <= 128; ++i) diagnostics->record(std::chrono::steady_clock::now(), i);
    EXPECT_EQ(StreamDiagnosticsTestAccess::report(*diagnostics).level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}
}  // namespace depthai_ros_driver
