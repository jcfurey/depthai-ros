#include <gtest/gtest.h>

#include <algorithm>
#include <thread>

#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/NeuralDepth.hpp"
#include "depthai/pipeline/node/Thermal.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"
#include "depthai_ros_driver/param_handlers/sensor_param_handler.hpp"
#include "depthai_ros_driver/param_handlers/stereo_param_handler.hpp"
#include "depthai_ros_driver/param_handlers/thermal_param_handler.hpp"

namespace depthai_ros_driver {
namespace dai_nodes::sensor_helpers {
class ImagePublisherTestAccess {
   public:
    static void configure(ImagePublisher& pub, bool encoded, bool compressed, bool lazy = false) {
        pub.encConfig.enabled = encoded;
        pub.pubConfig.publishCompressed = compressed;
        pub.pubConfig.lazyPub = lazy;
        pub.convConfig.tfPrefix = "camera_optical_frame";
        pub.convConfig.lowBandwidth = encoded;
        pub.createImageConverter(nullptr);
    }
    static void h264(ImagePublisher& pub) {
        pub.encConfig.profile = dai::VideoEncoderProperties::Profile::H264_MAIN;
    }
    static void infoPublisher(ImagePublisher& pub) {
        pub.infoPub = pub.node->create_publisher<sensor_msgs::msg::CameraInfo>("info", 1);
    }
};
}  // namespace dai_nodes::sensor_helpers

using namespace param_handlers;
using dai_nodes::sensor_helpers::ImagePublisher;
using dai_nodes::sensor_helpers::ImagePublisherTestAccess;

class DriverDataPathTest : public testing::Test {
   protected:
    void SetUp() override {
        context = std::make_shared<rclcpp::Context>();
        context->init(0, nullptr);
        node = std::make_shared<rclcpp::Node>("data_path_test", rclcpp::NodeOptions().context(context));
    }
    void TearDown() override {
        node.reset();
        context->shutdown("test complete");
    }
    std::unique_ptr<SensorParamHandler> sensor() {
        auto ph = std::make_unique<SensorParamHandler>(node, "rgb", "OAK-D", false, dai::CameraBoardSocket::CAM_A);
        ph->declareParams(std::make_shared<dai::node::Camera>(), true);
        return ph;
    }
    std::shared_ptr<dai::EncodedFrame> encodedFrame() {
        auto frame = std::make_shared<dai::EncodedFrame>();
        frame->transformation.setSize(640, 400);
        frame->transformation.setSourceSize(640, 400);
        frame->transformation.setIntrinsicMatrix({{{400, 0, 320}, {0, 400, 200}, {0, 0, 1}}});
        frame->setTimestamp(std::chrono::steady_clock::now());
        frame->setData(std::vector<uint8_t>{1, 2, 3, 4});  // Intentionally not a decodable image.
        return frame;
    }
    rclcpp::Context::SharedPtr context;
    std::shared_ptr<rclcpp::Node> node;
};

TEST_F(DriverDataPathTest, AtomicExposureUsesAllPendingValuesInEitherOrder) {
    auto ph = sensor();
    node->set_parameter(rclcpp::Parameter("rgb.r_set_man_exposure", true));
    std::shared_ptr<dai::CameraControl> control;
    auto callback = node->add_on_set_parameters_callback([&](const auto& params) {
        control = ph->setRuntimeParams(params);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    });
    std::vector<rclcpp::Parameter> params = {rclcpp::Parameter("rgb.r_exposure", 2000), rclcpp::Parameter("rgb.r_iso", 400)};
    ASSERT_TRUE(node->set_parameters_atomically(params).successful);
    EXPECT_EQ(control->getExposureTime().count(), 2000);
    EXPECT_EQ(control->getSensitivity(), 400);
    params = {rclcpp::Parameter("rgb.r_iso", 500), rclcpp::Parameter("rgb.r_exposure", 3000)};
    ASSERT_TRUE(node->set_parameters_atomically(params).successful);
    EXPECT_EQ(control->getExposureTime().count(), 3000);
    EXPECT_EQ(control->getSensitivity(), 500);
}

TEST_F(DriverDataPathTest, ManualModesCanBeEnabledTogetherWithValues) {
    auto ph = sensor();
    auto control = ph->setRuntimeParams({rclcpp::Parameter("rgb.r_exposure", 2200),
                                         rclcpp::Parameter("rgb.r_iso", 400),
                                         rclcpp::Parameter("rgb.r_set_man_exposure", true),
                                         rclcpp::Parameter("rgb.r_focus", 120),
                                         rclcpp::Parameter("rgb.r_set_man_focus", true),
                                         rclcpp::Parameter("rgb.r_whitebalance", 4500),
                                         rclcpp::Parameter("rgb.r_set_man_whitebalance", true)});
    EXPECT_EQ(control->getExposureTime().count(), 2200);
    EXPECT_EQ(control->getSensitivity(), 400);
    EXPECT_EQ(control->getLensPosition(), 120);
    EXPECT_EQ(control->wbColorTemp, 4500);
}

TEST_F(DriverDataPathTest, ExposureRegionUsesCompletePendingTuple) {
    auto ph = sensor();
    auto control = ph->setRuntimeParams({rclcpp::Parameter("rgb.r_set_auto_exp_region", true),
                                         rclcpp::Parameter("rgb.r_auto_exp_region_start_x", 10),
                                         rclcpp::Parameter("rgb.r_auto_exp_region_start_y", 20),
                                         rclcpp::Parameter("rgb.r_auto_exp_region_width", 100),
                                         rclcpp::Parameter("rgb.r_auto_exp_region_height", 80)});
    EXPECT_EQ(control->aeRegion.x, 10);
    EXPECT_EQ(control->aeRegion.y, 20);
    EXPECT_EQ(control->aeRegion.width, 100);
    EXPECT_EQ(control->aeRegion.height, 80);
}

TEST_F(DriverDataPathTest, NeuralDepthPreservesCompanionThresholdAndAcceptsZero) {
    StereoParamHandler ph(node, "stereo", "OAK-D", false);
    ph.declareParams(std::make_shared<dai::node::NeuralDepth>());
    node->set_parameters({rclcpp::Parameter("stereo.r_confidence_threshold", 200), rclcpp::Parameter("stereo.r_edge_threshold", 30)});
    EXPECT_EQ(ph.setRuntimeParams({rclcpp::Parameter("rgb.r_iso", 400)}), nullptr);
    auto conf = ph.setRuntimeParams({rclcpp::Parameter("stereo.r_confidence_threshold", 0)});
    ASSERT_NE(conf, nullptr);
    EXPECT_EQ(conf->getConfidenceThreshold(), 0);
    EXPECT_EQ(conf->getEdgeThreshold(), 30);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("stereo.r_edge_threshold", 256)).successful);
}

TEST_F(DriverDataPathTest, ThermalOverridesPopulateInitialDeviceConfig) {
    auto thermalNode = std::make_shared<dai::node::Thermal>();
    auto options = rclcpp::NodeOptions().context(context).parameter_overrides({rclcpp::Parameter("thermal.r_auto_ffc", true),
                                                                               rclcpp::Parameter("thermal.r_brightness_level", 120),
                                                                               rclcpp::Parameter("thermal.i_orientation", "MIRROR"),
                                                                               rclcpp::Parameter("thermal.r_time_noise_filter_level", 2)});
    auto thermalRosNode = std::make_shared<rclcpp::Node>("thermal_test", options);
    ThermalParamHandler ph(thermalRosNode, "thermal", "OAK-T", false);
    ph.declareParams(thermalNode);
    EXPECT_EQ(thermalNode->initialConfig->ffcParams.autoFFC, true);
    EXPECT_EQ(thermalNode->initialConfig->imageParams.brightnessLevel, 120);
    EXPECT_EQ(thermalNode->initialConfig->imageParams.orientation, dai::ThermalConfig::ThermalImageOrientation::Mirror);
    EXPECT_EQ(thermalNode->initialConfig->imageParams.timeNoiseFilterLevel, 2);
}

TEST_F(DriverDataPathTest, CompressedRawCombinationIsRejectedBeforeDeviceAccess) {
    ImagePublisher pub(node, nullptr, "rgb", nullptr);
    utils::ImgPublisherConfig config;
    config.publishCompressed = true;
    config.daiNodeName = "rgb";
    EXPECT_THROW(pub.setup(nullptr, {}, config), std::invalid_argument);
}

TEST_F(DriverDataPathTest, UnexpectedFrameTypesAreDroppedSafely) {
    ImagePublisher pub(node, nullptr, "rgb", nullptr);
    ImagePublisherTestAccess::configure(pub, true, false);
    EXPECT_EQ(pub.convertData(std::make_shared<dai::ImgFrame>()), nullptr);
    ImagePublisherTestAccess::configure(pub, false, false);
    EXPECT_EQ(pub.convertData(encodedFrame()), nullptr);
    EXPECT_NO_THROW(pub.publish(std::shared_ptr<dai_nodes::sensor_helpers::Image>{}));
}

TEST_F(DriverDataPathTest, CompressedPayloadPassesThroughWithoutDecoding) {
    ImagePublisher pub(node, nullptr, "rgb", nullptr);
    ImagePublisherTestAccess::configure(pub, true, true);
    auto frame = encodedFrame();
    auto result = pub.convertData(frame);
    ASSERT_NE(result, nullptr);
    ASSERT_NE(result->compressedImg, nullptr);
    EXPECT_TRUE(std::equal(result->compressedImg->data.begin(), result->compressedImg->data.end(), frame->getData().begin(), frame->getData().end()));
    EXPECT_EQ(result->compressedImg->header, result->info->header);
    EXPECT_EQ(result->info->width, 640u);
    EXPECT_EQ(result->image, nullptr);
}

TEST_F(DriverDataPathTest, FfmpegPayloadHasDimensionsAndTimestampWithoutDecoding) {
    ImagePublisher pub(node, nullptr, "rgb", nullptr);
    ImagePublisherTestAccess::configure(pub, true, true);
    ImagePublisherTestAccess::h264(pub);
    auto frame = encodedFrame();
    auto result = pub.convertData(frame);
    ASSERT_NE(result, nullptr);
    ASSERT_NE(result->ffmpegPacket, nullptr);
    EXPECT_TRUE(std::equal(result->ffmpegPacket->data.begin(), result->ffmpegPacket->data.end(), frame->getData().begin(), frame->getData().end()));
    EXPECT_EQ(result->ffmpegPacket->header, result->info->header);
    EXPECT_EQ(result->ffmpegPacket->width, 640u);
    EXPECT_EQ(result->ffmpegPacket->height, 400u);
    EXPECT_EQ(result->ffmpegPacket->pts, rclcpp::Time(result->info->header.stamp).nanoseconds());
}

TEST_F(DriverDataPathTest, LazyPublisherSkipsConversionWithoutSubscribers) {
    ImagePublisher pub(node, nullptr, "rgb", nullptr);
    ImagePublisherTestAccess::configure(pub, true, true, true);
    EXPECT_FALSE(pub.shouldPublish());
    EXPECT_NO_THROW(pub.publish(std::shared_ptr<dai::ADatatype>{}));
}

TEST_F(DriverDataPathTest, CompressedLazyPublisherIncludesCameraInfoSubscribers) {
    ImagePublisher pub(node, nullptr, "rgb", nullptr);
    ImagePublisherTestAccess::configure(pub, true, true, true);
    ImagePublisherTestAccess::infoPublisher(pub);
    EXPECT_FALSE(pub.shouldPublish());
    auto sub = node->create_subscription<sensor_msgs::msg::CameraInfo>("info", 1, [](sensor_msgs::msg::CameraInfo::ConstSharedPtr) {});
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while(!pub.shouldPublish() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(pub.shouldPublish());
}

}  // namespace depthai_ros_driver
