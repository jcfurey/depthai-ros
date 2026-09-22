#include "depthai_filters/spatial_bb.hpp"

#include <cmath>

#include "depthai_filters/utils.hpp"

namespace depthai_filters {
SpatialBB::SpatialBB(const rclcpp::NodeOptions& options) : rclcpp::Node("spatial_bb_node", options) {
    onInit();
}
void SpatialBB::onInit() {
    // Metric detections carry their own frame and stamp; no image/calibration synchronization is needed.
    detSub.subscribe(this, "nn/spatial_detections", utils::inputQoS(*this));
    detSub.registerCallback([this](const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg) { overlayCB(nullptr, nullptr, msg); });
    markerPub = create_publisher<visualization_msgs::msg::MarkerArray>("spatial_bb", 1);
}
void SpatialBB::overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr&,
                          const sensor_msgs::msg::CameraInfo::ConstSharedPtr&,
                          const vision_msgs::msg::Detection3DArray::ConstSharedPtr& detections) {
    if(!detections) return;
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    for(const auto& detection : detections->detections) {
        const auto& box = detection.bbox;
        const auto& p = box.center.position;
        if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(box.size.x) || !std::isfinite(box.size.y)
           || !std::isfinite(box.size.z) || box.size.x < 0 || box.size.y < 0 || box.size.z < 0)
            continue;
        visualization_msgs::msg::Marker marker;
        marker.header = detection.header.frame_id.empty() ? detections->header : detection.header;
        marker.ns = "detections";
        marker.id = id++;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = box.center;
        const auto& q = marker.pose.orientation;
        const auto norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if(!std::isfinite(norm)) continue;
        if(norm == 0)
            marker.pose.orientation.w = 1;
        else {
            marker.pose.orientation.x /= norm;
            marker.pose.orientation.y /= norm;
            marker.pose.orientation.z /= norm;
            marker.pose.orientation.w /= norm;
        }
        if(box.size.x > 0 && box.size.y > 0 && box.size.z > 0) {
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.scale = box.size;
        } else {
            // Display a position marker, not an invented estimate of object dimensions.
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.05;
        }
        marker.color.g = 1.0;
        marker.color.a = 0.7;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        markers.markers.push_back(marker);
        if(!detection.results.empty()) {
            marker.ns = "detections_label";
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.pose.position.y -= 0.1;
            marker.scale.z = 0.08;
            marker.color.r = marker.color.b = marker.color.a = 1.0;
            marker.text = detection.results.front().hypothesis.class_id;
            markers.markers.push_back(marker);
        }
    }
    markerPub->publish(markers);
}
}  // namespace depthai_filters
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::SpatialBB);
