#include <cmath>
#include <cstdio>
#include <functional>
#include <tuple>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/host/RGBD.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/PointCloudConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_examples/common.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

struct OutputQueues {
    std::shared_ptr<dai::MessageQueue> stereoOut;
    std::shared_ptr<dai::MessageQueue> rgbOut;
    std::shared_ptr<dai::MessageQueue> leftOut;
    std::shared_ptr<dai::MessageQueue> rightOut;
    std::shared_ptr<dai::MessageQueue> imuOut;
    std::shared_ptr<dai::MessageQueue> previewOut;
    std::shared_ptr<dai::MessageQueue> detectionOut;
    std::shared_ptr<dai::MessageQueue> pclOut;
    std::shared_ptr<dai::InputQueue> controlLeft;
    std::shared_ptr<dai::InputQueue> controlRight;
    std::shared_ptr<dai::InputQueue> controlRgb;
};
struct PipelineOpts {
    dai::Platform platform;
    std::string nnName;
    bool lrcheck;
    bool extended;
    bool subpixel;
    int stereoFPS;
    int rgbWidth;
    int rgbHeight;
    int monoWidth;
    int monoHeight;
};
OutputQueues createPipeline(dai::Pipeline& pipeline, PipelineOpts opts) {
    auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, {}, opts.stereoFPS);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, {}, opts.stereoFPS);
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto rgbd = pipeline.create<dai::node::RGBD>()->build();
    auto imu = pipeline.create<dai::node::IMU>();
    auto spatialDetectionNetwork = pipeline.create<dai::node::SpatialDetectionNetwork>();
    std::shared_ptr<dai::node::ImageAlign> align;

    // StereoDepth
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->setLeftRightCheck(opts.lrcheck);
    stereo->setExtendedDisparity(opts.extended);
    stereo->setSubpixel(opts.subpixel);

    // Imu
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_UNCALIBRATED, 480);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_UNCALIBRATED, 400);
    imu->setBatchReportThreshold(5);
    imu->setMaxBatchReports(10);

    auto rgbOut =
        camRgb->requestOutput(std::make_pair(opts.rgbWidth, opts.rgbHeight), dai::ImgFrame::Type::RGB888i, dai::ImgResizeMode::CROP, opts.stereoFPS, true);

    auto monoOutLeft = monoLeft->requestOutput(std::make_pair(opts.monoWidth, opts.monoHeight));
    monoOutLeft->link(stereo->left);
    auto monoOutRight = monoRight->requestOutput(std::make_pair(opts.monoWidth, opts.monoHeight));
    monoOutRight->link(stereo->right);
    auto stereoOut = stereo->depth.createOutputQueue(8, false);

    if(opts.platform == dai::Platform::RVC4) {
        rgbOut->link(rgbd->inColor);
        align = pipeline.create<dai::node::ImageAlign>();
        stereo->depth.link(align->input);
        rgbOut->link(align->inputAlignTo);
        align->outputAligned.link(rgbd->inDepth);
    } else {
        rgbOut->link(rgbd->inColor);
        rgbOut->link(stereo->inputAlignTo);
        stereo->depth.link(rgbd->inDepth);
    }

    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5f);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);
    dai::NNModelDescription modelDesc;
    modelDesc.model = opts.nnName;
    spatialDetectionNetwork->build(camRgb, stereo, modelDesc, 30);  // 30 FPS

    OutputQueues queues;
    queues.detectionOut = spatialDetectionNetwork->out.createOutputQueue(8, false);
    queues.previewOut = spatialDetectionNetwork->passthrough.createOutputQueue(8, false);
    queues.pclOut = rgbd->pcl.createOutputQueue(8, false);
    queues.stereoOut = stereoOut;
    queues.rgbOut = rgbOut->createOutputQueue(8, false);
    queues.imuOut = imu->out.createOutputQueue(8, false);
    queues.controlRgb = camRgb->inputControl.createInputQueue(8, false);
    queues.controlLeft = monoLeft->inputControl.createInputQueue(8, false);
    queues.controlRight = monoRight->inputControl.createInputQueue(8, false);

    return queues;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::string tfPrefix = "oak";
    auto node = rclcpp::Node::make_shared(tfPrefix);
    tfPrefix = depthai_examples::framePrefix(node);

    std::string mxId = depthai_examples::parameter<std::string>(node, "mxId", "");
    std::string ip = depthai_examples::parameter<std::string>(node, "ip", "");
    std::string nnName = depthai_examples::parameter<std::string>(node, "nnName", "yolov6-nano");
    int imuModeParam = depthai_examples::parameter<int>(node, "imuMode", 0);
    bool lrcheck = depthai_examples::parameter<bool>(node, "lrcheck", true);
    bool extended = depthai_examples::parameter<bool>(node, "extended", false);
    bool subpixel = depthai_examples::parameter<bool>(node, "subpixel", true);
    int rgbWidth = depthai_examples::parameter<int>(node, "rgbWidth", 640);
    int rgbHeight = depthai_examples::parameter<int>(node, "rgbHeight", 400);
    int monoWidth = depthai_examples::parameter<int>(node, "monoWidth", 640);
    int monoHeight = depthai_examples::parameter<int>(node, "monoHeight", 400);
    int stereoFPS = depthai_examples::parameter<int>(node, "stereoFPS", 30);
    bool manualExposure = depthai_examples::parameter<bool>(node, "manualExposure", false);
    int expTime = depthai_examples::parameter<int>(node, "expTime", 20000);
    int sensIso = depthai_examples::parameter<int>(node, "sensIso", 800);
    double angularVelCovariance = depthai_examples::parameter<double>(node, "angularVelCovariance", 0.02);
    double linearAccelCovariance = depthai_examples::parameter<double>(node, "linearAccelCovariance", 0.0);
    bool enableDotProjector = depthai_examples::parameter<bool>(node, "enableDotProjector", false);
    bool enableFloodLight = depthai_examples::parameter<bool>(node, "enableFloodLight", false);
    double dotProjectorIntensity = depthai_examples::parameter<double>(node, "dotProjectorIntensity", 0.5);
    double floodLightIntensity = depthai_examples::parameter<double>(node, "floodLightIntensity", 0.5);
    bool enableRosBaseTimeUpdate = depthai_examples::parameter<bool>(node, "enableRosBaseTimeUpdate", false);

    if(nnName.empty() || imuModeParam < 0 || imuModeParam > 2 || rgbWidth <= 0 || rgbHeight <= 0 || monoWidth <= 0 || monoHeight <= 0 || stereoFPS <= 0
       || (manualExposure && (expTime <= 0 || sensIso <= 0)) || !std::isfinite(angularVelCovariance) || angularVelCovariance < 0
       || !std::isfinite(linearAccelCovariance) || linearAccelCovariance < 0 || !std::isfinite(dotProjectorIntensity) || dotProjectorIntensity < 0
       || dotProjectorIntensity > 1 || !std::isfinite(floodLightIntensity) || floodLightIntensity < 0 || floodLightIntensity > 1) {
        RCLCPP_ERROR(node->get_logger(),
                     "Invalid example parameters: require nonempty model, IMU mode 0..2, positive image sizes/FPS/exposure, nonnegative covariance and IR "
                     "intensities in [0,1].");
        return 1;
    }
    if(enableDotProjector || enableFloodLight) {
        RCLCPP_ERROR(node->get_logger(), "This example does not implement IR control; use depthai_ros_driver for device-aware IR configuration.");
        return 1;
    }
    depthai_bridge::ImuSyncMethod imuMode = static_cast<depthai_bridge::ImuSyncMethod>(imuModeParam);
    auto device = depthai_examples::connect(node, mxId, ip);

    dai::Pipeline pipeline(device);
    PipelineOpts opts = {device->getPlatform(), nnName, lrcheck, extended, subpixel, stereoFPS, rgbWidth, rgbHeight, monoWidth, monoHeight};
    auto queues = createPipeline(pipeline, opts);

    // Set manual exposure
    if(manualExposure) {
        auto ctrl = std::make_shared<dai::CameraControl>();
        ctrl->setManualExposure(expTime, sensIso);
        queues.controlLeft->send(ctrl);
        queues.controlRight->send(ctrl);
        queues.controlRgb->send(ctrl);
    }

    pipeline.start();

    auto imuConverter = std::make_shared<depthai_bridge::ImuConverter>(
        depthai_bridge::getFrameName(tfPrefix, "imu_frame"), imuMode, linearAccelCovariance, angularVelCovariance);

    imuConverter->setClock(node->get_clock());
    auto imuPublish = std::make_unique<depthai_bridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData>>(
        queues.imuOut,
        node,
        "imu",
        [imuConverter](std::shared_ptr<dai::IMUData> msg, std::deque<sensor_msgs::msg::Imu>& rosMsgs) { imuConverter->toRosMsg(msg, rosMsgs); },
        30);

    imuPublish->addPublisherCallback();

    auto rgbConverter = std::make_shared<depthai_bridge::ImageConverter>(
        depthai_bridge::getOpticalFrameName(tfPrefix, depthai_bridge::getSocketName(dai::CameraBoardSocket::CAM_A, device->getDeviceName())), false);

    rgbConverter->setClock(node->get_clock());
    if(enableRosBaseTimeUpdate) {
        imuConverter->setUpdateRosBaseTimeOnToRosMsg();
        rgbConverter->setUpdateRosBaseTimeOnToRosMsg();
    }
    auto calibrationHandler = device->readCalibration();
    auto tfPub =
        std::make_unique<depthai_bridge::TFPublisher>(node, calibrationHandler, device->getConnectedCameraFeatures(), tfPrefix, device->getDeviceName());
    {
        auto pclConv = std::make_shared<depthai_bridge::PointCloudConverter>(
            depthai_bridge::getOpticalFrameName(tfPrefix, depthai_bridge::getSocketName(dai::CameraBoardSocket::CAM_A, device->getDeviceName())), false);
        pclConv->setClock(node->get_clock());
        pclConv->setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::METER);
        auto pclPublish = std::make_unique<depthai_bridge::BridgePublisher<sensor_msgs::msg::PointCloud2, dai::PointCloudData>>(
            queues.pclOut,
            node,
            "stereo/points",
            std::bind(&depthai_bridge::PointCloudConverter::toRosMsg, pclConv, std::placeholders::_1, std::placeholders::_2),
            30);
        pclPublish->addPublisherCallback();

        auto rgbCameraInfo = rgbConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, rgbWidth, rgbHeight);
        auto depthPublish = std::make_unique<depthai_bridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>>(
            queues.stereoOut,
            node,
            "stereo/depth",
            [rgbConverter](std::shared_ptr<dai::ImgFrame> msg, std::deque<sensor_msgs::msg::Image>& rosMsgs) { rgbConverter->toRosMsg(msg, rosMsgs); },
            30,
            rgbCameraInfo,
            "stereo");
        depthPublish->addPublisherCallback();

        auto imgQueue = queues.rgbOut;
        auto rgbPublish = std::make_unique<depthai_bridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>>(
            imgQueue,
            node,
            "color/image",
            [rgbConverter](std::shared_ptr<dai::ImgFrame> msg, std::deque<sensor_msgs::msg::Image>& rosMsgs) { rgbConverter->toRosMsg(msg, rosMsgs); },
            30,
            rgbCameraInfo,
            "color");
        rgbPublish->addPublisherCallback();

        auto previewQueue = queues.previewOut;
        auto detectionQueue = queues.detectionOut;
        auto previewCameraInfo = rgbConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A);

        auto previewPublish = std::make_unique<depthai_bridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>>(
            previewQueue,
            node,
            "rgb/preview/image",
            [rgbConverter](std::shared_ptr<dai::ImgFrame> msg, std::deque<sensor_msgs::msg::Image>& rosMsgs) { rgbConverter->toRosMsg(msg, rosMsgs); },
            30,
            previewCameraInfo,
            "rgb/preview");
        previewPublish->addPublisherCallback();

        auto detConverter = std::make_shared<depthai_bridge::SpatialDetectionConverter>(
            depthai_bridge::getOpticalFrameName(tfPrefix, depthai_bridge::getSocketName(dai::CameraBoardSocket::CAM_A, device->getDeviceName())), false);

        detConverter->setClock(node->get_clock());
        auto detectionPublish = std::make_unique<depthai_bridge::BridgePublisher<depthai_ros_msgs::msg::SpatialDetectionArray, dai::SpatialImgDetections>>(
            detectionQueue,
            node,
            "rgb/spatial_detections",
            std::bind(&depthai_bridge::SpatialDetectionConverter::toRosMsg, detConverter, std::placeholders::_1, std::placeholders::_2),
            30);
        detectionPublish->addPublisherCallback();
        depthai_examples::spinPipeline(node, pipeline);
    }
    return 0;
}
