#include "depthai_filters/thermal_temp.hpp"

#if __has_include("cv_bridge/cv_bridge.hpp")
    #include "cv_bridge/cv_bridge.hpp"
#else
    #include "cv_bridge/cv_bridge.h"
#endif
#include "depthai_filters/utils.hpp"

namespace depthai_filters {

ThermalTemp::ThermalTemp(const rclcpp::NodeOptions& options) : rclcpp::Node("thermal_temp", options) {
    onInit();
}
void ThermalTemp::onInit() {
    const auto qos = utils::inputQoS(*this);
    declare_parameter<int>("sample_x", 0);
    declare_parameter<int>("sample_y", 0);
    sub = this->create_subscription<sensor_msgs::msg::Image>("thermal/raw_data/image_raw", qos, std::bind(&ThermalTemp::subCB, this, std::placeholders::_1));
    colorPub = this->create_publisher<sensor_msgs::msg::Image>("color", 10);
}

void ThermalTemp::mouseCallback(int /* event */, int x, int y, int /* flags */, void* /* userdata */) {
    mouseX = x;
    mouseY = y;
}

void ThermalTemp::subCB(const sensor_msgs::msg::Image::ConstSharedPtr& img) {
    cv::Mat frameFp32 = utils::msgToMat(this->get_logger(), img, sensor_msgs::image_encodings::TYPE_32FC1);
    if(frameFp32.empty()) return;
    mouseX = get_parameter("sample_x").as_int();
    mouseY = get_parameter("sample_y").as_int();
    cv::Mat normalized;
    cv::normalize(frameFp32, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::Mat colormapped;
    cv::applyColorMap(normalized, colormapped, cv::COLORMAP_MAGMA);
    if(mouseX < 0 || mouseY < 0 || mouseX >= colormapped.cols || mouseY >= colormapped.rows) {
        mouseX = std::max(0, std::min(static_cast<int>(mouseX), colormapped.cols - 1));
        mouseY = std::max(0, std::min(static_cast<int>(mouseY), colormapped.rows - 1));
    }
    double min, max;
    cv::minMaxLoc(frameFp32, &min, &max);
    auto textColor = cv::Scalar(255, 255, 255);
    // Draw crosshair
    cv::line(colormapped, cv::Point(mouseX - 10, mouseY), cv::Point(mouseX + 10, mouseY), textColor, 1);
    cv::line(colormapped, cv::Point(mouseX, mouseY - 10), cv::Point(mouseX, mouseY + 10), textColor, 1);
    // Draw deg C
    char text[32];
    snprintf(text, sizeof(text), "%.1f deg C", frameFp32.at<float>(mouseY, mouseX));
    bool putTextLeft = mouseX > colormapped.cols / 2;
    cv::putText(colormapped, text, cv::Point(putTextLeft ? mouseX - 100 : mouseX + 10, mouseY - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, textColor, 1);
    sensor_msgs::msg::Image outMsg;
    cv_bridge::CvImage(img->header, sensor_msgs::image_encodings::BGR8, colormapped).toImageMsg(outMsg);

    colorPub->publish(outMsg);
}

}  // namespace depthai_filters
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::ThermalTemp);
