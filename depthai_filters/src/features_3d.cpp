#include "depthai_filters/features_3d.hpp"

#include <cmath>
#include <limits>

#if __has_include("cv_bridge/cv_bridge.hpp")
    #include "cv_bridge/cv_bridge.hpp"
#else
    #include "cv_bridge/cv_bridge.h"
#endif
#include "depthai_filters/utils.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace depthai_filters {

Features3D::Features3D(const rclcpp::NodeOptions& options) : rclcpp::Node("features3d", options) {
    onInit();
}
void Features3D::onInit() {
    const auto qos = utils::inputQoS(*this);
    depthSub.subscribe(this, "stereo/image_raw", qos);
    infoSub.subscribe(this, "stereo/camera_info", qos);
    featureSub.subscribe(this, "feature_tracker/tracked_features", qos);
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), depthSub, infoSub, featureSub);
    sync->registerCallback(std::bind(&Features3D::overlayCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    pclPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("features", 10);
    overlayPub = this->create_publisher<sensor_msgs::msg::Image>("overlay", 10);
    desqueeze = this->declare_parameter<bool>("desqueeze", false);
}
float Features3D::getDepthAt(int x, int y, const sensor_msgs::msg::Image::ConstSharedPtr& depth_image) {
    return depth_image ? utils::depthAt(x, y, *depth_image) : std::numeric_limits<float>::quiet_NaN();
}
void Features3D::overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& depth,
                           const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info,
                           const depthai_ros_msgs::msg::TrackedFeatures::ConstSharedPtr& features) {
    if(!depth || !info || !features || !depth->width || !depth->height || !std::isfinite(info->k[0]) || info->k[0] <= 0 || !std::isfinite(info->k[4])
       || info->k[4] <= 0 || !std::isfinite(info->k[2]) || !std::isfinite(info->k[5]) || (depth->encoding != "16UC1" && depth->encoding != "32FC1")
       || info->width != depth->width || info->height != depth->height || (!info->header.frame_id.empty() && info->header.frame_id != depth->header.frame_id)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping depth/features with invalid encoding, calibration or frame alignment");
        return;
    }
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = depth->header;
    cloud.is_dense = false;
    cloud.height = 1;
    cloud.width = features->features.size();  // assuming features is your vector of 2D points
    sensor_msgs::PointCloud2Modifier pcd_modifier(cloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
    double fx = info->k[0];
    double fy = info->k[4];
    double cx = info->k[2];
    double cy = info->k[5];
    for(const auto& feature : features->features) {
        const bool inBounds = std::isfinite(feature.position.x) && std::isfinite(feature.position.y) && feature.position.x >= 0 && feature.position.y >= 0
                              && feature.position.x < depth->width && feature.position.y < depth->height;
        const float depthVal =
            inBounds ? getDepthAt(static_cast<int>(feature.position.x), static_cast<int>(feature.position.y), depth) : std::numeric_limits<float>::quiet_NaN();
        *out_x = (feature.position.x - cx) * depthVal / fx;
        *out_y = (feature.position.y - cy) * depthVal / fy;
        *out_z = depthVal;
        ++out_x;
        ++out_y;
        ++out_z;
    }
    pclPub->publish(cloud);
}

}  // namespace depthai_filters
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::Features3D);
