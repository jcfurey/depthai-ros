#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <thread>

#include "depthai_filters/detection2d_overlay.hpp"
#include "depthai_filters/features_3d.hpp"
#include "depthai_filters/spatial_bb.hpp"
#include "depthai_filters/thermal_temp.hpp"
#include "depthai_filters/utils.hpp"
#include "depthai_filters/wls_filter.hpp"

TEST(DepthImage, EncodingsByteOrdersPaddingAndInvalidInput) {
    sensor_msgs::msg::Image image;
    image.width = 2;
    image.height = 1;
    image.step = 6;
    image.encoding = "16UC1";
    image.data = {232, 3, 208, 7, 0, 0};
    EXPECT_FLOAT_EQ(depthai_filters::utils::depthAt(0, 0, image), 1);
    EXPECT_FLOAT_EQ(depthai_filters::utils::depthAt(1, 0, image), 2);
    image.is_bigendian = true;
    image.data = {3, 232, 7, 208, 0, 0};
    EXPECT_FLOAT_EQ(depthai_filters::utils::depthAt(0, 0, image), 1);
    EXPECT_FLOAT_EQ(depthai_filters::utils::depthAt(1, 0, image), 2);
    EXPECT_TRUE(std::isnan(depthai_filters::utils::depthAt(-1, 0, image)));
    EXPECT_TRUE(std::isnan(depthai_filters::utils::depthAt(2, 0, image)));
    EXPECT_TRUE(std::isnan(depthai_filters::utils::depthAt(0, 1, image)));
    image.encoding = "32FC1";
    image.width = 1;
    image.step = 4;
    image.data = {0x3f, 0x80, 0, 0};
    EXPECT_FLOAT_EQ(depthai_filters::utils::depthAt(0, 0, image), 1);
    image.is_bigendian = false;
    image.data = {0, 0, 0x80, 0x3f};
    EXPECT_FLOAT_EQ(depthai_filters::utils::depthAt(0, 0, image), 1);
    image.data = {0, 0, 0x80, 0x7f};
    EXPECT_TRUE(std::isnan(depthai_filters::utils::depthAt(0, 0, image)));
    image.data.resize(1);
    EXPECT_TRUE(std::isnan(depthai_filters::utils::depthAt(0, 0, image)));
    image.encoding = "mono8";
    EXPECT_TRUE(std::isnan(depthai_filters::utils::depthAt(0, 0, image)));
}

TEST(FilterComponents, SensorInputQoSAndBadFramesAreSafe) {
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    auto options = rclcpp::NodeOptions().context(context);
    {
        depthai_filters::Features3D features(options);
        EXPECT_EQ(features.depthSub.getSubscriber()->get_actual_qos().reliability(), rclcpp::ReliabilityPolicy::BestEffort);
        auto image = std::make_shared<sensor_msgs::msg::Image>();
        image->width = image->height = 1;
        image->encoding = "16UC1";
        EXPECT_NO_THROW(
            features.overlayCB(image, std::make_shared<sensor_msgs::msg::CameraInfo>(), std::make_shared<depthai_ros_msgs::msg::TrackedFeatures>()));
        depthai_filters::Detection2DOverlay overlay(options);
        auto detections = std::make_shared<vision_msgs::msg::Detection2DArray>();
        detections->detections.resize(1);  // Valid detection with no hypotheses.
        image->encoding = "bgr8";
        image->step = 3;
        image->data = {0, 0, 0};
        EXPECT_NO_THROW(overlay.overlayCB(image, detections));
        image->data.clear();
        EXPECT_NO_THROW(overlay.overlayCB(image, detections));
        depthai_filters::ThermalTemp thermal(options);
        image->encoding = "32FC1";
        image->step = 4;
        image->data = {0, 0, 0x80, 0x3f};
        EXPECT_NO_THROW(thermal.subCB(image));  // No GUI/display required.
    }
    context->shutdown("test complete");
}

TEST(FilterComponents, BestEffortDepthProducesMetricCloudWithCaptureStamp) {
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    {
        auto options = rclcpp::NodeOptions().context(context);
        auto filter = std::make_shared<depthai_filters::Features3D>(options);
        auto source = std::make_shared<rclcpp::Node>("depth_source", options);
        auto depth = source->create_publisher<sensor_msgs::msg::Image>("stereo/image_raw", rclcpp::SensorDataQoS());
        auto info = source->create_publisher<sensor_msgs::msg::CameraInfo>("stereo/camera_info", rclcpp::SensorDataQoS());
        auto features = source->create_publisher<depthai_ros_msgs::msg::TrackedFeatures>("feature_tracker/tracked_features", rclcpp::SensorDataQoS());
        sensor_msgs::msg::PointCloud2::SharedPtr received;
        auto subscription = source->create_subscription<sensor_msgs::msg::PointCloud2>(
            "features", rclcpp::SensorDataQoS(), [&](sensor_msgs::msg::PointCloud2::SharedPtr message) { received = message; });
        sensor_msgs::msg::Image image;
        image.header.stamp.sec = 42;
        image.header.frame_id = "optical";
        image.width = image.height = 1;
        image.step = 4;
        image.encoding = "32FC1";
        image.data = {0, 0, 0x80, 0x3f};
        sensor_msgs::msg::CameraInfo calibration;
        calibration.header = image.header;
        calibration.width = calibration.height = 1;
        calibration.k[0] = calibration.k[4] = calibration.k[8] = 1;
        depthai_ros_msgs::msg::TrackedFeatures points;
        points.header = image.header;
        points.features.resize(2);
        points.features[1].position.x = -1;  // Invalid point must become NaN, without indexing the image.
        rclcpp::ExecutorOptions executorOptions;
        executorOptions.context = context;
        rclcpp::executors::SingleThreadedExecutor executor(executorOptions);
        executor.add_node(source);
        executor.add_node(filter);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while(!received && std::chrono::steady_clock::now() < deadline) {
            depth->publish(image);
            info->publish(calibration);
            features->publish(points);
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ASSERT_NE(received, nullptr);
        EXPECT_EQ(received->header, image.header);
        ASSERT_EQ(received->width, 2u);
        const auto zField = std::find_if(received->fields.begin(), received->fields.end(), [](const auto& field) { return field.name == "z"; });
        ASSERT_NE(zField, received->fields.end());
        float z;
        std::memcpy(&z, received->data.data() + zField->offset, sizeof(z));
        EXPECT_FLOAT_EQ(z, 1);
        std::memcpy(&z, received->data.data() + received->point_step + zField->offset, sizeof(z));
        EXPECT_TRUE(std::isnan(z));
    }
    context->shutdown("test complete");
}

TEST(FilterComponents, MetricMarkersKeepInputHeaderWithoutHypotheses) {
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    {
        auto options = rclcpp::NodeOptions().context(context);
        auto filter = std::make_shared<depthai_filters::SpatialBB>(options);
        auto sink = std::make_shared<rclcpp::Node>("marker_sink", options);
        visualization_msgs::msg::MarkerArray::SharedPtr received;
        auto subscription = sink->create_subscription<visualization_msgs::msg::MarkerArray>(
            "spatial_bb", rclcpp::QoS(1), [&](visualization_msgs::msg::MarkerArray::SharedPtr message) { received = message; });
        auto input = std::make_shared<vision_msgs::msg::Detection3DArray>();
        input->header.frame_id = "optical";
        input->header.stamp.sec = 42;
        input->detections.resize(1);
        input->detections[0].bbox.center.position.x = 0.1;
        input->detections[0].bbox.center.position.z = 2;
        rclcpp::ExecutorOptions executorOptions;
        executorOptions.context = context;
        rclcpp::executors::SingleThreadedExecutor executor(executorOptions);
        executor.add_node(sink);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while(!received && std::chrono::steady_clock::now() < deadline) {
            filter->overlayCB(nullptr, nullptr, input);
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ASSERT_NE(received, nullptr);
        ASSERT_EQ(received->markers.size(), 1u);
        EXPECT_EQ(received->markers[0].header, input->header);
        EXPECT_DOUBLE_EQ(received->markers[0].pose.position.x, 0.1);
        EXPECT_DOUBLE_EQ(received->markers[0].pose.position.z, 2);
        EXPECT_DOUBLE_EQ(received->markers[0].pose.orientation.w, 1);
        EXPECT_EQ(received->markers[0].type, visualization_msgs::msg::Marker::SPHERE);
    }
    context->shutdown("test complete");
}

TEST(FilterComponents, WlsProducesMetresAndDoesNotApplyRejectedParameters) {
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    {
        auto options = rclcpp::NodeOptions().context(context);
        auto filter = std::make_shared<depthai_filters::WLSFilter>(options);
        const auto oldLambda = filter->filter->getLambda();
        EXPECT_FALSE(filter->set_parameters_atomically({rclcpp::Parameter("lambda", 123.0), rclcpp::Parameter("sigma_color", -1.0)}).successful);
        EXPECT_DOUBLE_EQ(filter->filter->getLambda(), oldLambda);
        auto sink = std::make_shared<rclcpp::Node>("wls_sink", options);
        sensor_msgs::msg::Image::SharedPtr received;
        auto subscription = sink->create_subscription<sensor_msgs::msg::Image>(
            "wls_filtered", rclcpp::SensorDataQoS(), [&](sensor_msgs::msg::Image::SharedPtr message) { received = message; });
        auto disparity = std::make_shared<sensor_msgs::msg::Image>();
        disparity->header.frame_id = "right_optical";
        disparity->header.stamp.sec = 42;
        disparity->width = disparity->height = disparity->step = 16;
        disparity->encoding = "mono8";
        disparity->data.resize(256, 10);  // Ten pixels disparity.
        auto left = std::make_shared<sensor_msgs::msg::Image>(*disparity);
        auto info = std::make_shared<sensor_msgs::msg::CameraInfo>();
        info->p[3] = -20;  // f * baseline = 20 pixel metres; expected depth = 2 metres.
        rclcpp::ExecutorOptions executorOptions;
        executorOptions.context = context;
        rclcpp::executors::SingleThreadedExecutor executor(executorOptions);
        executor.add_node(sink);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while(!received && std::chrono::steady_clock::now() < deadline) {
            filter->wlsCB(disparity, info, left);
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ASSERT_NE(received, nullptr);
        EXPECT_EQ(received->encoding, "32FC1");
        EXPECT_EQ(received->header, disparity->header);
        EXPECT_NEAR(depthai_filters::utils::depthAt(8, 8, *received), 2.0, 0.02);
    }
    context->shutdown("test complete");
}
