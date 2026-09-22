#include <cstdio>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "depthai_examples/common.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/imu.hpp"

int main(int argc, char** argv) {
    std::string tfPrefix = "oak";
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared(tfPrefix);
    tfPrefix = depthai_examples::framePrefix(node);

    auto device = depthai_examples::connect(node);
    dai::Pipeline pipeline(device);

    // Define sources and outputs
    auto imu = pipeline.create<dai::node::IMU>();
    // Enable ACCELEROMETER_UNCALIBRATED at 480 hz rate
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_UNCALIBRATED, 480);
    // Enable GYROSCOPE_UNCALIBRATED at 400 hz rate
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_UNCALIBRATED, 400);
    // Set batch report threshold and max batch reports
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);

    // Create output queue
    auto imuQ = imu->out.createOutputQueue(8, false);
    pipeline.start();

    // Create a bridge publisher for RGB images
    depthai_bridge::ImuSyncMethod imuMode = depthai_bridge::ImuSyncMethod::COPY;
    auto imuConv = std::make_shared<depthai_bridge::ImuConverter>(depthai_bridge::getFrameName(tfPrefix, "imu_frame"), imuMode);
    imuConv->setClock(node->get_clock());

    auto calibrationHandler = device->readCalibration();
    auto tfPub =
        std::make_unique<depthai_bridge::TFPublisher>(node, calibrationHandler, device->getConnectedCameraFeatures(), tfPrefix, device->getDeviceName());

    auto imuPub = std::make_unique<depthai_bridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData>>(
        imuQ,
        node,
        "imu/data",
        [imuConv](std::shared_ptr<dai::IMUData> msg, std::deque<sensor_msgs::msg::Imu>& rosMsgs) { imuConv->toRosMsg(msg, rosMsgs); },
        30);

    imuPub->addPublisherCallback();

    depthai_examples::spinPipeline(node, pipeline);

    return 0;
}
