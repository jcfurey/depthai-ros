#pragma once

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "rclcpp/rclcpp.hpp"

namespace depthai_examples {
template <typename T>
T parameter(const std::shared_ptr<rclcpp::Node>& node, const std::string& name, const T& value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "Example startup setting; restart the example to change it.";
    return node->declare_parameter<T>(name, value, descriptor);
}

inline std::shared_ptr<dai::Device> connect(const std::shared_ptr<rclcpp::Node>& node, const std::string& legacyId = "", const std::string& legacyIp = "") {
    const auto ip = parameter<std::string>(node, "device_ip", legacyIp);
    const auto id = parameter<std::string>(node, "device_id", legacyId);
    const auto usb = parameter<std::string>(node, "usb_port_id", "");
    if(int(!ip.empty()) + int(!id.empty()) + int(!usb.empty()) > 1) throw std::invalid_argument("Select only one of device_ip, device_id or usb_port_id");
    if(ip.empty() && id.empty() && usb.empty()) return std::make_shared<dai::Device>();
    return std::make_shared<dai::Device>(dai::DeviceInfo(!ip.empty() ? ip : !id.empty() ? id : usb));
}
inline std::string framePrefix(const std::shared_ptr<rclcpp::Node>& node) {
    return depthai_bridge::resolveFramePrefix(parameter<std::string>(node, "tf_prefix", node->get_name()), node->get_name());
}
inline void spinPipeline(const std::shared_ptr<rclcpp::Node>& node, dai::Pipeline& pipeline) {
    rclcpp::ExecutorOptions options;
    options.context = node->get_node_base_interface()->get_context();
    rclcpp::executors::SingleThreadedExecutor executor(options);
    executor.add_node(node);
    try {
        while(rclcpp::ok(options.context) && pipeline.isRunning()) executor.spin_once(std::chrono::milliseconds(50));
    } catch(const std::exception& error) {
        RCLCPP_ERROR(node->get_logger(), "Example stopped: %s", error.what());
    }
    // Stop SDK producers while publishers and captured converters still exist.
    try {
        pipeline.stop();
        pipeline.wait();
    } catch(const std::exception& error) {
        RCLCPP_WARN(node->get_logger(), "Pipeline shutdown: %s", error.what());
    }
}
}  // namespace depthai_examples
