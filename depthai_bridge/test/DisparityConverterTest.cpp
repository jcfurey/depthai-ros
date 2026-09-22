#include <gtest/gtest.h>

#include <cstring>
#include <cv_bridge/cv_bridge.hpp>
#include <deque>
#include <memory>

#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"

namespace depthai_bridge {

TEST(DisparityConverterTest, ConstructorTest) {
    DisparityConverter converter("testFrame", 1.0f, 100.0f, 10.0f, 1000.0f, true);
    EXPECT_EQ(converter.getFocalLength(), 1.0f);
    EXPECT_EQ(converter.getBaseline(), 1.0f);
    EXPECT_EQ(converter.getMinDepth(), 0.1f);
    EXPECT_EQ(converter.getMaxDepth(), 10.0f);
}

TEST(DisparityConverterTest, ToRosMsgTest) {
    DisparityConverter converter("testFrame", 1.0f, 100.0f, 10.0f, 1000.0f, true);
    auto inData = std::make_shared<dai::ImgFrame>();
    inData->setType(dai::ImgFrame::Type::RAW8);
    inData->setData({1, 2, 3, 4});
    inData->setHeight(2);
    inData->setWidth(2);

    std::deque<DisparityMsgs::DisparityImage> outDispImageMsgs;
    converter.toRosMsg(inData, outDispImageMsgs);

    ASSERT_EQ(outDispImageMsgs.size(), 1);
    auto& outDispImageMsg = outDispImageMsgs.front();
    EXPECT_EQ(outDispImageMsg.f, 1.0f);
    EXPECT_EQ(outDispImageMsg.min_disparity, 0.1f);
    EXPECT_EQ(outDispImageMsg.max_disparity, 10.0f);
    EXPECT_EQ(outDispImageMsg.t, 1.0f);
    EXPECT_EQ(outDispImageMsg.image.encoding, sensor_msgs::image_encodings::TYPE_32FC1);
    EXPECT_EQ(outDispImageMsg.image.height, 2);
    EXPECT_EQ(outDispImageMsg.image.width, 2);
    EXPECT_EQ(outDispImageMsg.image.step, 8);
    EXPECT_FLOAT_EQ(cv_bridge::toCvCopy(outDispImageMsg.image)->image.at<float>(1, 1), 4.0f);
    EXPECT_EQ(outDispImageMsg.image.data.size(), 16);
}

TEST(DisparityConverterTest, ToRosMsgPtrTest) {
    DisparityConverter converter("testFrame", 1.0f, 100.0f, 10.0f, 1000.0f, true);
    auto inData = std::make_shared<dai::ImgFrame>();
    inData->setType(dai::ImgFrame::Type::RAW8);
    inData->setData({1, 2, 3, 4});
    inData->setHeight(2);
    inData->setWidth(2);

    auto outDispImagePtr = converter.toRosMsgPtr(inData);

    ASSERT_NE(outDispImagePtr, nullptr);
    EXPECT_EQ(outDispImagePtr->f, 1.0f);
    EXPECT_EQ(outDispImagePtr->min_disparity, 0.1f);
    EXPECT_EQ(outDispImagePtr->max_disparity, 10.0f);
    EXPECT_EQ(outDispImagePtr->t, 1.0f);
    EXPECT_EQ(outDispImagePtr->image.encoding, sensor_msgs::image_encodings::TYPE_32FC1);
    EXPECT_EQ(outDispImagePtr->image.height, 2);
    EXPECT_EQ(outDispImagePtr->image.width, 2);
    EXPECT_EQ(outDispImagePtr->image.step, 8);
    EXPECT_FLOAT_EQ(cv_bridge::toCvCopy(outDispImagePtr->image)->image.at<float>(0, 0), 1.0f);
    EXPECT_EQ(outDispImagePtr->image.data.size(), 16);
}

TEST(DisparityConverterTest, SubpixelAndMalformedInput) {
    DisparityConverter converter("frame", 100, 10, 10, 1000);
    auto input = std::make_shared<dai::ImgFrame>();
    input->setType(dai::ImgFrame::Type::RAW16);
    input->setSize(2, 1);
    input->setData({48, 0, 0, 1});
    auto output = converter.toRosMsgPtr(input);
    auto image = cv_bridge::toCvCopy(output->image)->image;
    EXPECT_FLOAT_EQ(image.at<float>(0, 0), 1.5f);
    EXPECT_FLOAT_EQ(image.at<float>(0, 1), 8.0f);
    EXPECT_FLOAT_EQ(output->f * output->t / image.at<float>(0, 1), 1.25f);
    converter.setSubpixelFractionalBits(3);
    EXPECT_FLOAT_EQ(converter.toRosMsgPtr(input)->delta_d, 0.125f);
    EXPECT_THROW(converter.setSubpixelFractionalBits(0), std::invalid_argument);
    input->setData({1});
    EXPECT_THROW(converter.toRosMsgPtr(input), std::invalid_argument);
    input->setSize(0, 0);
    EXPECT_THROW(converter.toRosMsgPtr(input), std::invalid_argument);
}
}  // namespace depthai_bridge
