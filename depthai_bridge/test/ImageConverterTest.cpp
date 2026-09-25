#include <gtest/gtest.h>

#include <fstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/imgcodecs.hpp"

namespace depthai_bridge {

class ImageConverterTest : public ::testing::Test {
   protected:
    void SetUp() override {
        // Set up any necessary data or configurations here
    }

    void TearDown() override {
        // Clean up any resources if needed
    }
};

TEST_F(ImageConverterTest, ConstructorTest) {
    ImageConverter converter("test_frame", true, false);
    EXPECT_EQ(converter.getFrameName(), "test_frame");
    EXPECT_TRUE(converter.isDaiInterleaved());
    EXPECT_FALSE(converter.isGetBaseDeviceTimestamp());
}

TEST_F(ImageConverterTest, ConvertFromBitstreamTest) {
    ImageConverter converter("test_frame", true, false);
    converter.convertFromBitstream(dai::ImgFrame::Type::BGR888i);
    EXPECT_TRUE(converter.isFromBitstream());
    EXPECT_EQ(converter.getSrcType(), dai::ImgFrame::Type::BGR888i);
}

TEST_F(ImageConverterTest, ConvertDispToDepthTest) {
    ImageConverter converter("test_frame", true, false);
    converter.convertDispToDepth(0.1);
    EXPECT_TRUE(converter.isDispToDepth());
    EXPECT_EQ(converter.getBaseline(), 0.1);
}

TEST_F(ImageConverterTest, AddExposureOffsetTest) {
    ImageConverter converter("test_frame", true, false);
    dai::CameraExposureOffset offset;
    converter.addExposureOffset(offset);
    EXPECT_TRUE(converter.isAddExpOffset());
    EXPECT_EQ(converter.getExpOffset(), offset);
}

TEST_F(ImageConverterTest, ReverseStereoSocketOrderTest) {
    ImageConverter converter("test_frame", true, false);
    converter.reverseStereoSocketOrder();
    EXPECT_TRUE(converter.isReversedStereoSocketOrder());
}

TEST_F(ImageConverterTest, SetAlphaScalingTest) {
    ImageConverter converter("test_frame", true, false);
    converter.setAlphaScaling(0.5);
    EXPECT_TRUE(converter.isAlphaScalingEnabled());
    EXPECT_EQ(converter.getAlphaScalingFactor(), 0.5);
}

TEST_F(ImageConverterTest, SetFFMPEGEncodingTest) {
    ImageConverter converter("test_frame", true, false);
    converter.setFFMPEGEncoding("h264");
    EXPECT_EQ(converter.getFFMPEGEncoding(), "h264");
}

TEST_F(ImageConverterTest, ToRosMsgRawPtrTest) {
    ImageConverter converter("test_frame", true, false);
    std::shared_ptr<dai::ImgFrame> inData = std::make_shared<dai::ImgFrame>();
    inData->setWidth(640);
    inData->setHeight(480);
    inData->setType(dai::ImgFrame::Type::BGR888i);
    inData->setData(std::vector<uint8_t>(640 * 480 * 3, 128));
    sensor_msgs::msg::CameraInfo info;
    auto outImageMsg = converter.toRosMsgRawPtr(inData, info);
    EXPECT_EQ(outImageMsg.header.frame_id, "test_frame");
    EXPECT_EQ(outImageMsg.width, 640);
    EXPECT_EQ(outImageMsg.height, 480);
    EXPECT_EQ(outImageMsg.encoding, "bgr8");
    EXPECT_FALSE(outImageMsg.is_bigendian);
    EXPECT_EQ(outImageMsg.step, 640 * 3);
    EXPECT_EQ(outImageMsg.data.size(), 640 * 480 * 3);
}

TEST_F(ImageConverterTest, ThermalYuv422UsesNativeDimensionsAndByteOrder) {
    ImageConverter converter("thermal_frame", true, false);
    auto inData = std::make_shared<dai::ImgFrame>();
    inData->setWidth(256);
    inData->setHeight(192);
    inData->setStride(256 * 2);
    inData->setType(dai::ImgFrame::Type::YUV422i);
    inData->setData(std::vector<uint8_t>(256 * 192 * 2, 128));
    sensor_msgs::msg::CameraInfo info;

    auto outImageMsg = converter.toRosMsgRawPtr(inData, info);

    EXPECT_EQ(outImageMsg.header.frame_id, "thermal_frame");
    EXPECT_EQ(outImageMsg.width, 256);
    EXPECT_EQ(outImageMsg.height, 192);
    EXPECT_EQ(outImageMsg.encoding, "yuv422");
    EXPECT_FALSE(outImageMsg.is_bigendian);
    EXPECT_EQ(outImageMsg.step, 256 * 2);
    EXPECT_EQ(outImageMsg.data.size(), 256 * 192 * 2);
}

TEST_F(ImageConverterTest, ToRosCompressedMsgTest) {
    ImageConverter converter("test_frame", true, false);
    std::shared_ptr<dai::EncodedFrame> inData = std::make_shared<dai::EncodedFrame>();
    inData->setWidth(640);
    inData->setHeight(480);
    inData->setData(std::vector<uint8_t>(640 * 480 * 3, 128));
    std::deque<sensor_msgs::msg::CompressedImage> outImageMsgs;
    converter.toRosCompressedMsg(inData, outImageMsgs);
    auto outImageMsg = outImageMsgs.front();
    EXPECT_EQ(outImageMsg.header.frame_id, "test_frame");
    EXPECT_EQ(outImageMsg.format, "jpeg");
    EXPECT_EQ(outImageMsg.data.size(), 640 * 480 * 3);
}

TEST_F(ImageConverterTest, ToRosFFMPEGPacketTest) {
    ImageConverter converter("test_frame", true, false);
    std::shared_ptr<dai::EncodedFrame> inData = std::make_shared<dai::EncodedFrame>();
    inData->setWidth(640);
    inData->setHeight(480);
    inData->setData(std::vector<uint8_t>(640 * 480 * 3, 128));
    std::deque<FFMPEGMsgs::FFMPEGPacket> outImageMsgs;
    converter.toRosFFMPEGPacket(inData, outImageMsgs);
    auto outFrameMsg = outImageMsgs.front();

    EXPECT_EQ(outFrameMsg.header.frame_id, "test_frame");
    EXPECT_EQ(outFrameMsg.encoding, "libx264");
    EXPECT_EQ(outFrameMsg.data.size(), 640 * 480 * 3);
}

TEST_F(ImageConverterTest, ToRosMsgTest) {
    ImageConverter converter("test_frame", true, false);
    std::shared_ptr<dai::ImgFrame> inData = std::make_shared<dai::ImgFrame>();
    inData->setWidth(640);
    inData->setHeight(480);
    inData->setType(dai::ImgFrame::Type::BGR888i);
    inData->setData(std::vector<uint8_t>(640 * 480 * 3, 128));
    std::deque<sensor_msgs::msg::Image> outImageMsgs;
    converter.toRosMsg(inData, outImageMsgs);
    EXPECT_EQ(outImageMsgs.size(), 1);
    EXPECT_EQ(outImageMsgs.front().header.frame_id, "test_frame");
    EXPECT_EQ(outImageMsgs.front().width, 640);
    EXPECT_EQ(outImageMsgs.front().height, 480);
    EXPECT_EQ(outImageMsgs.front().encoding, "bgr8");
    EXPECT_EQ(outImageMsgs.front().step, 640 * 3);
    EXPECT_EQ(outImageMsgs.front().data.size(), 640 * 480 * 3);
}

TEST_F(ImageConverterTest, ToRosMsgPtrTest) {
    ImageConverter converter("test_frame", true, false);
    std::shared_ptr<dai::ImgFrame> inData = std::make_shared<dai::ImgFrame>();
    inData->setWidth(640);
    inData->setHeight(480);
    inData->setType(dai::ImgFrame::Type::BGR888i);
    inData->setData(std::vector<uint8_t>(640 * 480 * 3, 128));
    auto outImageMsg = converter.toRosMsgPtr(inData);
    EXPECT_EQ(outImageMsg->header.frame_id, "test_frame");
    EXPECT_EQ(outImageMsg->width, 640);
    EXPECT_EQ(outImageMsg->height, 480);
    EXPECT_EQ(outImageMsg->encoding, "bgr8");
    EXPECT_EQ(outImageMsg->step, 640 * 3);
    EXPECT_EQ(outImageMsg->data.size(), 640 * 480 * 3);
}

TEST_F(ImageConverterTest, ToDaiMsgTest) {
    ImageConverter converter("test_frame", true, false);
    sensor_msgs::msg::Image inMsg;
    inMsg.width = 640;
    inMsg.height = 480;
    inMsg.encoding = "bgr8";
    inMsg.step = 640 * 3;
    inMsg.data = std::vector<uint8_t>(640 * 480 * 3, 128);
    dai::ImgFrame outData;
    converter.toDaiMsg(inMsg, outData);
    EXPECT_EQ(outData.getWidth(), 640);
    EXPECT_EQ(outData.getHeight(), 480);
    EXPECT_EQ(outData.getType(), dai::ImgFrame::Type::BGR888i);
    EXPECT_EQ(outData.getData().size(), 640 * 480 * 3);
}

namespace {
std::shared_ptr<dai::EncodedFrame> encodedFrame(const cv::Mat& image) {
    std::vector<uint8_t> bytes;
    // PNG keeps the synthetic pixels exact while exercising the same imdecode path as MJPEG.
    EXPECT_TRUE(cv::imencode(".png", image, bytes));
    auto frame = std::make_shared<dai::EncodedFrame>();
    frame->setWidth(image.cols);
    frame->setHeight(image.rows);
    frame->setData(bytes);
    return frame;
}
}  // namespace

TEST_F(ImageConverterTest, BitstreamColorDecodesAsBgr8) {
    for(auto type : {dai::ImgFrame::Type::BGR888i, dai::ImgFrame::Type::RGB888i, dai::ImgFrame::Type::NV12}) {
        ImageConverter converter("test_frame", true, false);
        converter.convertFromBitstream(type);
        auto msg = converter.toRosMsgRawPtr(encodedFrame(cv::Mat(4, 6, CV_8UC3, cv::Scalar(10, 20, 30))));
        EXPECT_EQ(msg.encoding, "bgr8");
        EXPECT_EQ(msg.width, 6u);
        EXPECT_EQ(msg.height, 4u);
        ASSERT_EQ(msg.data.size(), 6u * 4u * 3u);
        EXPECT_EQ(msg.data[0], 10);
        EXPECT_EQ(msg.data[2], 30);
    }
}

TEST_F(ImageConverterTest, BitstreamRaw8DecodesAsMono8) {
    ImageConverter converter("test_frame", true, false);
    converter.convertFromBitstream(dai::ImgFrame::Type::RAW8);
    auto msg = converter.toRosMsgRawPtr(encodedFrame(cv::Mat(4, 6, CV_8UC1, cv::Scalar(42))));
    EXPECT_EQ(msg.encoding, "mono8");
    EXPECT_EQ(msg.step, 6u);
    ASSERT_EQ(msg.data.size(), 24u);
    EXPECT_EQ(msg.data[5], 42);
}

TEST_F(ImageConverterTest, BitstreamDisparityConvertsToMillimetreDepth) {
    ImageConverter converter("test_frame", true, false);
    converter.convertFromBitstream(dai::ImgFrame::Type::RAW8);
    converter.convertDispToDepth(7.5);  // cm
    cv::Mat disparity(1, 3, CV_8UC1);
    disparity.at<uint8_t>(0, 0) = 0;
    disparity.at<uint8_t>(0, 1) = 50;
    disparity.at<uint8_t>(0, 2) = 1;
    sensor_msgs::msg::CameraInfo info;
    info.p[0] = 1000.0;
    auto msg = converter.toRosMsgRawPtr(encodedFrame(disparity), info);
    EXPECT_EQ(msg.encoding, "16UC1");
    ASSERT_EQ(msg.data.size(), 6u);
    const auto* depth = reinterpret_cast<const uint16_t*>(msg.data.data());
    EXPECT_EQ(depth[0], 0);     // no disparity
    EXPECT_EQ(depth[1], 1500);  // 75 mm * 1000 px / 50 px
    EXPECT_EQ(depth[2], 0);     // 75000 mm does not fit in 16 bits
}

TEST_F(ImageConverterTest, UnsupportedImgFrameTypeThrows) {
    ImageConverter converter("test_frame", true, false);
    auto inData = std::make_shared<dai::ImgFrame>();
    inData->setWidth(4);
    inData->setHeight(4);
    inData->setType(dai::ImgFrame::Type::RAW10);
    inData->setData(std::vector<uint8_t>(32, 0));
    EXPECT_THROW(converter.toRosMsgRawPtr(inData), std::runtime_error);
}

TEST_F(ImageConverterTest, ToDaiMsgPlanarSplitsChannels) {
    ImageConverter converter("test_frame", false, false);
    sensor_msgs::msg::Image inMsg;
    inMsg.width = 2;
    inMsg.height = 1;
    inMsg.encoding = "bgr8";
    inMsg.step = 6;
    inMsg.data = {1, 2, 3, 4, 5, 6};
    dai::ImgFrame outData;
    converter.toDaiMsg(inMsg, outData);
    EXPECT_EQ(outData.getType(), dai::ImgFrame::Type::BGR888p);
    const std::vector<uint8_t> expected = {1, 4, 2, 5, 3, 6};
    ASSERT_EQ(outData.getData().size(), expected.size());
    EXPECT_TRUE(std::equal(expected.begin(), expected.end(), outData.getData().begin()));

    inMsg.encoding = "mono8";
    EXPECT_THROW(converter.toDaiMsg(inMsg, outData), std::runtime_error);
}

TEST_F(ImageConverterTest, RosMsgtoCvMatTest) {
    ImageConverter converter("test_frame", true, false);
    sensor_msgs::msg::Image inMsg;
    inMsg.encoding = "bgr8";
    inMsg.width = 3;
    inMsg.height = 3;
    inMsg.data = {
        0, 0, 0, 1, 1, 1, 2, 2, 2,  // Row 0
        3, 3, 3, 4, 4, 4, 5, 5, 5,  // Row 1
        6, 6, 6, 7, 7, 7, 8, 8, 8   // Row 2
    };
    auto cvMat = converter.rosMsgtoCvMat(inMsg);
    EXPECT_EQ(cvMat.rows, 3);
    EXPECT_EQ(cvMat.cols, 3);
    EXPECT_EQ(cvMat.type(), CV_8UC3);
    EXPECT_EQ(cvMat.at<cv::Vec3b>(0, 0), cv::Vec3b(0, 0, 0));
    EXPECT_EQ(cvMat.at<cv::Vec3b>(0, 1), cv::Vec3b(1, 1, 1));
    EXPECT_EQ(cvMat.at<cv::Vec3b>(0, 2), cv::Vec3b(2, 2, 2));
    EXPECT_EQ(cvMat.at<cv::Vec3b>(1, 0), cv::Vec3b(3, 3, 3));
    EXPECT_EQ(cvMat.at<cv::Vec3b>(1, 1), cv::Vec3b(4, 4, 4));
    EXPECT_EQ(cvMat.at<cv::Vec3b>(1, 2), cv::Vec3b(5, 5, 5));
    EXPECT_EQ(cvMat.at<cv::Vec3b>(2, 0), cv::Vec3b(6, 6, 6));
    EXPECT_EQ(cvMat.at<cv::Vec3b>(2, 1), cv::Vec3b(7, 7, 7));
    EXPECT_EQ(cvMat.at<cv::Vec3b>(2, 2), cv::Vec3b(8, 8, 8));
}

TEST_F(ImageConverterTest, CalibrationToCameraInfoTest) {
    ImageConverter converter("test_frame", true, false);
    std::ifstream f(ament_index_cpp::get_package_share_directory("depthai_bridge") + "/resources/cal.json");
    nlohmann::json data = nlohmann::json::parse(f);
    dai::CalibrationHandler calibHandler = dai::CalibrationHandler::fromJson(data);
    dai::CameraBoardSocket cameraId = dai::CameraBoardSocket::CAM_A;
    dai::Point2f topLeftPixelId = {0, 0};
    dai::Point2f bottomRightPixelId = {640, 480};
    auto cameraInfo = converter.calibrationToCameraInfo(calibHandler, cameraId, 640, 480, topLeftPixelId, bottomRightPixelId);
    EXPECT_EQ(cameraInfo.width, 640);
    EXPECT_EQ(cameraInfo.height, 480);
    EXPECT_EQ(cameraInfo.distortion_model, "rational_polynomial");
    EXPECT_EQ(cameraInfo.k.size(), 9);
    EXPECT_EQ(cameraInfo.d.size(), 8);
    EXPECT_EQ(cameraInfo.p.size(), 12);
    EXPECT_EQ(cameraInfo.r.size(), 9);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
}  // namespace depthai_bridge
