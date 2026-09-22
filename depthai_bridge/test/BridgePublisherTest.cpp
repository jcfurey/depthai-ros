#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <thread>

#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;
using Publisher = depthai_bridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData>;

class BridgePublisherTest : public ::testing::Test {
   protected:
    void SetUp() override {
        context = std::make_shared<rclcpp::Context>();
        context->init(0, nullptr);
        node = std::make_shared<rclcpp::Node>("bridge_contract_test", rclcpp::NodeOptions().context(context));
        queue = std::make_shared<dai::MessageQueue>(2, false);
    }
    void TearDown() override {
        node.reset();
        context->shutdown("test complete");
    }
    std::shared_ptr<rclcpp::Context> context;
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<dai::MessageQueue> queue;
};

TEST_F(BridgePublisherTest, RemovesCallbackAndDisablesCopiedCallback) {
    std::atomic<int> calls{0};
    std::function<void(std::string, std::shared_ptr<dai::ADatatype>)> copied;
    {
        Publisher pub(queue, node, "imu", [&](auto, auto&) { ++calls; }, rclcpp::QoS(1), false);
        pub.addPublisherCallback();
        EXPECT_THROW(pub.addPublisherCallback(), std::logic_error);
        EXPECT_THROW(pub.startPublisherThread(), std::logic_error);
        copied = queue->callbacks.begin()->second;
        copied("imu", std::make_shared<dai::IMUData>());
        EXPECT_EQ(calls, 1);
    }
    EXPECT_TRUE(queue->callbacks.empty());
    copied("imu", std::make_shared<dai::IMUData>());
    EXPECT_EQ(calls, 1);
}

TEST_F(BridgePublisherTest, StopsWithoutShuttingDownOwningContext) {
    const auto start = std::chrono::steady_clock::now();
    {
        Publisher pub(queue, node, "imu", [](auto, auto&) {});
        pub.startPublisherThread();
        EXPECT_THROW(pub.startPublisherThread(), std::logic_error);
        pub.stop();
        pub.startPublisherThread();
    }
    EXPECT_LT(std::chrono::steady_clock::now() - start, 1s);
    EXPECT_TRUE(context->is_valid());
    // Stopping a publisher must not close its caller's queue.
    EXPECT_NO_THROW(queue->send(std::make_shared<dai::IMUData>()));
}

TEST_F(BridgePublisherTest, DestructionDrainsInflightConversion) {
    std::promise<void> entered, release;
    auto ready = release.get_future().share();
    auto pub = std::make_unique<Publisher>(
        queue,
        node,
        "imu",
        [&](auto, auto&) {
            entered.set_value();
            ready.wait();
        },
        rclcpp::QoS(1),
        false);
    pub->addPublisherCallback();
    auto callback = queue->callbacks.begin()->second;
    auto producer = std::async(std::launch::async, [&]() { callback("imu", std::make_shared<dai::IMUData>()); });
    entered.get_future().wait();
    auto destroy = std::async(std::launch::async, [&]() { pub.reset(); });
    EXPECT_EQ(destroy.wait_for(20ms), std::future_status::timeout);
    release.set_value();
    EXPECT_EQ(destroy.wait_for(1s), std::future_status::ready);
    destroy.get();
    producer.get();
}

TEST_F(BridgePublisherTest, CameraInfoOnlyWithRemappedImageAndDefaultCalibrationName) {
    using ImagePublisher = depthai_bridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>;
    auto camera = std::make_shared<rclcpp::Node>("camera", rclcpp::NodeOptions().context(context).arguments({"--ros-args", "-r", "image:=remapped/image_raw"}));
    ImagePublisher publisher(
        queue,
        camera,
        "image",
        [](auto, auto& output) {
            sensor_msgs::msg::Image image;
            image.header.frame_id = "optical";
            image.header.stamp.sec = 42;
            image.width = image.height = image.step = 1;
            image.encoding = "mono8";
            image.data = {7};
            output.push_back(image);
        },
        size_t(1));
    sensor_msgs::msg::CameraInfo::SharedPtr received;
    auto subscriber = node->create_subscription<sensor_msgs::msg::CameraInfo>(
        "remapped/camera_info", rclcpp::QoS(1), [&](sensor_msgs::msg::CameraInfo::SharedPtr msg) { received = msg; });
    rclcpp::ExecutorOptions executorOptions;
    executorOptions.context = context;
    rclcpp::executors::SingleThreadedExecutor executor(executorOptions);
    executor.add_node(node);
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while(!received && std::chrono::steady_clock::now() < deadline) {
        publisher.publishHelper(std::make_shared<dai::ImgFrame>());
        executor.spin_some();
        std::this_thread::sleep_for(10ms);
    }
    ASSERT_NE(received, nullptr);
    EXPECT_EQ(received->header.frame_id, "optical");
    EXPECT_EQ(received->header.stamp.sec, 42);
}

TEST_F(BridgePublisherTest, CompressedOnlySubscriberWakesLazyRemappedCamera) {
    using ImagePublisher = depthai_bridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>;
    auto camera = std::make_shared<rclcpp::Node>(
        "compressed_camera", "/robot", rclcpp::NodeOptions().context(context).arguments({"--ros-args", "-r", "image:=front/image_raw"}));
    ImagePublisher publisher(
        queue,
        camera,
        "image",
        [](auto, auto& output) {
            sensor_msgs::msg::Image image;
            image.header.frame_id = "front_optical";
            image.width = image.height = image.step = 16;
            image.encoding = "mono8";
            image.data.resize(256, 100);
            output.push_back(std::move(image));
        },
        size_t(1));
    sensor_msgs::msg::CompressedImage::SharedPtr received;
    auto subscriber = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/robot/front/image_raw/compressed", rclcpp::SensorDataQoS(), [&](sensor_msgs::msg::CompressedImage::SharedPtr msg) { received = msg; });
    rclcpp::ExecutorOptions executorOptions;
    executorOptions.context = context;
    rclcpp::executors::SingleThreadedExecutor executor(executorOptions);
    executor.add_node(node);
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while(!received && std::chrono::steady_clock::now() < deadline) {
        publisher.publishHelper(std::make_shared<dai::ImgFrame>());
        executor.spin_some();
        std::this_thread::sleep_for(10ms);
    }
    ASSERT_NE(received, nullptr);
    EXPECT_EQ(received->header.frame_id, "front_optical");
    EXPECT_FALSE(received->data.empty());
}

TEST_F(BridgePublisherTest, PublisherQoSOverridesReachTheEndpoint) {
    auto source = std::make_shared<rclcpp::Node>(
        "qos_source",
        rclcpp::NodeOptions().context(context).parameter_overrides({rclcpp::Parameter("qos_overrides./override_imu.publisher.reliability", "best_effort"),
                                                                    rclcpp::Parameter("qos_overrides./override_imu.publisher.depth", 3)}));
    Publisher publisher(queue, source, "override_imu", [](auto, auto&) {});
    auto endpoints = source->get_publishers_info_by_topic("/override_imu");
    ASSERT_EQ(endpoints.size(), 1u);
    EXPECT_EQ(endpoints.front().qos_profile().reliability(), rclcpp::ReliabilityPolicy::BestEffort);
    EXPECT_EQ(endpoints.front().qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
    EXPECT_EQ(endpoints.front().qos_profile().depth(), 3u);
}

TEST_F(BridgePublisherTest, OdometryAndTransformKeepTheSameCaptureStamp) {
    using OdomPublisher = depthai_bridge::BridgePublisher<nav_msgs::msg::Odometry, dai::IMUData>;
    OdomPublisher publisher(
        queue,
        node,
        "odometry",
        [](auto, auto& output) {
            nav_msgs::msg::Odometry odometry;
            odometry.header.frame_id = "world";
            odometry.child_frame_id = "camera";
            odometry.header.stamp.sec = 42;
            odometry.pose.pose.orientation.w = 1;
            output.push_back(odometry);
        },
        rclcpp::QoS(1),
        false);
    publisher.enableTransformPub();
    nav_msgs::msg::Odometry::SharedPtr odometry;
    tf2_msgs::msg::TFMessage::SharedPtr transforms;
    auto odomSub =
        node->create_subscription<nav_msgs::msg::Odometry>("odometry", rclcpp::QoS(1), [&](nav_msgs::msg::Odometry::SharedPtr msg) { odometry = msg; });
    auto tfSub =
        node->create_subscription<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(10), [&](tf2_msgs::msg::TFMessage::SharedPtr msg) { transforms = msg; });
    rclcpp::ExecutorOptions options;
    options.context = context;
    rclcpp::executors::SingleThreadedExecutor executor(options);
    executor.add_node(node);
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while((!odometry || !transforms) && std::chrono::steady_clock::now() < deadline) {
        publisher.publishHelper(std::make_shared<dai::IMUData>());
        executor.spin_some();
        std::this_thread::sleep_for(10ms);
    }
    ASSERT_NE(odometry, nullptr);
    ASSERT_NE(transforms, nullptr);
    ASSERT_EQ(transforms->transforms.size(), 1u);
    EXPECT_EQ(transforms->transforms[0].header, odometry->header);
    EXPECT_EQ(odometry->header.stamp.sec, 42);
}
