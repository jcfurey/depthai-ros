#include "depthai_filters/detection2d_overlay.hpp"

#include <cmath>

#if __has_include("cv_bridge/cv_bridge.hpp")
    #include "cv_bridge/cv_bridge.hpp"
#else
    #include "cv_bridge/cv_bridge.h"
#endif
#include "depthai_filters/utils.hpp"

namespace depthai_filters {

Detection2DOverlay::Detection2DOverlay(const rclcpp::NodeOptions& options) : rclcpp::Node("detection_overlay", options) {
    onInit();
}
void Detection2DOverlay::onInit() {
    const auto qos = utils::inputQoS(*this);
    previewSub.subscribe(this, "nn/passthrough/image_raw", qos);
    detSub.subscribe(this, "nn/detections", qos);
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), previewSub, detSub);
    sync->registerCallback(std::bind(&Detection2DOverlay::overlayCB, this, std::placeholders::_1, std::placeholders::_2));
    overlayPub = this->create_publisher<sensor_msgs::msg::Image>("overlay", 10);
}

void Detection2DOverlay::overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& preview,
                                   const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections) {
    cv::Mat previewMat = utils::msgToMat(this->get_logger(), preview, sensor_msgs::image_encodings::BGR8);

    if(previewMat.empty() || !detections) return;
    auto blue = cv::Scalar(255, 0, 0);

    for(auto& detection : detections->detections) {
        if(detection.results.empty() || !std::isfinite(detection.bbox.center.position.x) || !std::isfinite(detection.bbox.center.position.y)
           || !std::isfinite(detection.bbox.size_x) || !std::isfinite(detection.bbox.size_y) || detection.bbox.size_x <= 0 || detection.bbox.size_y <= 0)
            continue;
        auto x1 = detection.bbox.center.position.x - detection.bbox.size_x / 2.0;
        auto x2 = detection.bbox.center.position.x + detection.bbox.size_x / 2.0;
        auto y1 = detection.bbox.center.position.y - detection.bbox.size_y / 2.0;
        auto y2 = detection.bbox.center.position.y + detection.bbox.size_y / 2.0;
        x1 = std::clamp(x1, 0.0, double(previewMat.cols));
        x2 = std::clamp(x2, 0.0, double(previewMat.cols));
        y1 = std::clamp(y1, 0.0, double(previewMat.rows));
        y2 = std::clamp(y2, 0.0, double(previewMat.rows));
        if(x2 <= x1 || y2 <= y1) continue;
        auto labelStr = detection.results[0].hypothesis.class_id;
        auto confidence = detection.results[0].hypothesis.score;
        utils::addTextToFrame(previewMat, labelStr, x1 + 10, y1 + 20);
        std::stringstream confStr;
        confStr << std::fixed << std::setprecision(2) << confidence * 100;
        utils::addTextToFrame(previewMat, confStr.str(), x1 + 10, y1 + 40);
        cv::rectangle(previewMat, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), blue);
    }
    sensor_msgs::msg::Image outMsg;
    cv_bridge::CvImage(preview->header, sensor_msgs::image_encodings::BGR8, previewMat).toImageMsg(outMsg);

    overlayPub->publish(outMsg);
}

}  // namespace depthai_filters
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::Detection2DOverlay);
