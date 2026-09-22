#include "depthai_filters/utils.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#if __has_include("cv_bridge/cv_bridge.hpp")
    #include "cv_bridge/cv_bridge.hpp"
#else
    #include "cv_bridge/cv_bridge.h"
#endif
#include "rclcpp/rclcpp.hpp"

namespace depthai_filters {
namespace utils {
rclcpp::QoS inputQoS(rclcpp::Node& node) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "Input QoS chosen when the component is loaded.";
    const auto reliability = node.declare_parameter<std::string>("input_qos.reliability", "best_effort", descriptor);
    const auto depth = node.declare_parameter<int>("input_qos.depth", 5, descriptor);
    if(depth < 1 || depth > 1000) throw std::invalid_argument("input_qos.depth must be between 1 and 1000");
    rclcpp::QoS qos(depth);
    if(reliability == "best_effort")
        qos.best_effort();
    else if(reliability == "reliable")
        qos.reliable();
    else
        throw std::invalid_argument("input_qos.reliability must be best_effort or reliable");
    return qos;
}

float depthAt(int x, int y, const sensor_msgs::msg::Image& image) {
    const float invalid = std::numeric_limits<float>::quiet_NaN();
    const size_t bytes = image.encoding == "16UC1" ? 2 : image.encoding == "32FC1" ? 4 : 0;
    if(!bytes || x < 0 || y < 0 || static_cast<uint32_t>(x) >= image.width || static_cast<uint32_t>(y) >= image.height
       || image.step < uint64_t(image.width) * bytes || uint64_t(image.step) * image.height > image.data.size())
        return invalid;
    const auto* p = image.data.data() + size_t(y) * image.step + size_t(x) * bytes;
    uint32_t value = 0;
    for(size_t i = 0; i < bytes; ++i) value |= uint32_t(p[i]) << (8 * (image.is_bigendian ? bytes - i - 1 : i));
    float metres;
    if(bytes == 2)
        metres = value * 0.001f;
    else
        std::memcpy(&metres, &value, sizeof(metres));
    return std::isfinite(metres) && metres > 0 ? metres : invalid;
}

cv::Mat msgToMat(const rclcpp::Logger& logger, const sensor_msgs::msg::Image::ConstSharedPtr& img, const std::string& encoding) {
    cv::Mat mat;
    if(!img || !img->width || !img->height) return mat;
    try {
        mat = cv_bridge::toCvCopy(img, encoding)->image;
    } catch(const std::exception& e) {
        static rclcpp::Clock diagnosticClock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(logger, diagnosticClock, 5000, "Dropping invalid image: %s", e.what());
    }
    return mat;
}
void addTextToFrame(cv::Mat& frame, const std::string& text, int x, int y) {
    auto white = cv::Scalar(255, 255, 255);
    auto black = cv::Scalar(0, 0, 0);

    cv::putText(frame, text, cv::Point(x, y), cv::FONT_HERSHEY_TRIPLEX, 0.5, white, 3);
    cv::putText(frame, text, cv::Point(x, y), cv::FONT_HERSHEY_TRIPLEX, 0.5, black);
}
}  // namespace utils
}  // namespace depthai_filters
