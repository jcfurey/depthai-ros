#include <gtest/gtest.h>

#include "depthai_bridge/depthaiUtility.hpp"

namespace depthai_bridge {

TEST(DepthaiUtilityTest, NormalizesFramePrefix) {
    EXPECT_EQ(normalizeFramePrefix("oak"), "oak");
    EXPECT_EQ(normalizeFramePrefix("/robot1/oak"), "robot1/oak");
    EXPECT_EQ(normalizeFramePrefix("robot1/oak/"), "robot1/oak");
    EXPECT_EQ(normalizeFramePrefix("//robot1/oak//"), "robot1/oak");
    EXPECT_EQ(normalizeFramePrefix("///"), "");
}

TEST(DepthaiUtilityTest, BuildsValidFrameNameFromRosStylePrefix) {
    EXPECT_EQ(getFrameName("/robot1/oak/", "imu_frame"), "robot1/oak_imu_frame");
    EXPECT_EQ(getOpticalFrameName("/robot1/oak/", "rgb"), "robot1/oak_rgb_camera_optical_frame");
}

TEST(DepthaiUtilityTest, ResolvesEmptyNormalizedPrefixToFallback) {
    EXPECT_EQ(resolveFramePrefix("", "oak"), "oak");
    EXPECT_EQ(resolveFramePrefix("/", "oak"), "oak");
    EXPECT_EQ(resolveFramePrefix("///", "/robot1/oak/"), "robot1/oak");
    EXPECT_EQ(resolveFramePrefix("/robot2/oak/", "oak"), "robot2/oak");
}

TEST(DepthaiUtilityTest, BuildsFrameNameWithoutEmptyPrefixSeparator) {
    EXPECT_EQ(getFrameName("", "imu_frame"), "imu_frame");
    EXPECT_EQ(getFrameName("/", "imu_frame"), "imu_frame");
}

TEST(DepthaiUtilityTest, NormalizesLegacyOakDeviceNames) {
    EXPECT_EQ(normalizeDeviceModelName("BW1098OBC"), "OAK-D");
    EXPECT_EQ(normalizeDeviceModelName("BW1098"), "OAK-D");
    EXPECT_EQ(normalizeDeviceModelName("DM9095"), "OAK-D-LITE");
    EXPECT_EQ(normalizeDeviceModelName("DM9098"), "OAK-D-S2");
    EXPECT_EQ(normalizeDeviceModelName("OAK-D Lite AF"), "OAK-D-LITE");
    EXPECT_EQ(normalizeDeviceModelName("oak_d_pro_w_ov9782"), "OAK-D-PRO-W");
    EXPECT_EQ(normalizeDeviceModelName("OAK-D-PRO-POE-AF-C18"), "OAK-D-PRO-POE");
    EXPECT_EQ(normalizeDeviceModelName("OAK-D-PRO-FF-PB-FF#1"), "OAK-D-PRO");
}

TEST(DepthaiUtilityTest, ResolvesDeviceModelWithoutReplacingSpecificNames) {
    EXPECT_EQ(resolveDeviceModelName("OAK-D Lite AF", "DM9095", 3), "OAK-D-LITE");
    EXPECT_EQ(resolveDeviceModelName("", "BW1098OBC", 3), "OAK-D");
    EXPECT_EQ(resolveDeviceModelName("", "DM9098", 3), "OAK-D-S2");
    EXPECT_EQ(resolveDeviceModelName("custom-board", "", 3), "OAK-D");
    EXPECT_EQ(resolveDeviceModelName("", "", 1), "OAK-1");
    EXPECT_EQ(resolveDeviceModelName("", "", 0), "UNKNOWN");
}

TEST(DepthaiUtilityTest, RecognizesLegacyBno08xImus) {
    EXPECT_TRUE(isBno08x("BNO084"));
    EXPECT_TRUE(isBno08x("BNO085"));
    EXPECT_TRUE(isBno08x("bno086"));
    EXPECT_FALSE(isBno08x("BMI270"));
    EXPECT_FALSE(isBno08x("NONE"));
}

}  // namespace depthai_bridge
