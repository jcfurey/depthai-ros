#pragma once

#include "opencv2/core/mat.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace rclcpp {
class Logger;
}

namespace depthai_filters {
namespace utils {
rclcpp::QoS inputQoS(rclcpp::Node& node);
float depthAt(int x, int y, const sensor_msgs::msg::Image& image);
cv::Mat msgToMat(const rclcpp::Logger& logger, const sensor_msgs::msg::Image::ConstSharedPtr& img, const std::string& encoding);
void addTextToFrame(cv::Mat& frame, const std::string& text, int x, int y);
}  // namespace utils
}  // namespace depthai_filters