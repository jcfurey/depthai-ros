#include "depthai_bridge/DisparityConverter.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "depthai_bridge/depthaiUtility.hpp"

namespace depthai_bridge {

DisparityConverter::DisparityConverter(
    const std::string frameName, float focalLength, float baseline, float minDepth, float maxDepth, bool getBaseDeviceTimestamp)
    : BaseConverter(std::move(frameName), getBaseDeviceTimestamp),
      focalLength(focalLength),
      baseline(baseline / 100.0),
      minDepth(minDepth / 100.0),
      maxDepth(maxDepth / 100.0) {
    if(!std::isfinite(focalLength) || focalLength <= 0 || !std::isfinite(baseline) || baseline <= 0 || !std::isfinite(minDepth) || minDepth <= 0
       || !std::isfinite(maxDepth) || maxDepth <= minDepth) {
        throw std::invalid_argument("Disparity calibration requires positive finite focal length/baseline and 0 < minDepth < maxDepth");
    }
}

DisparityConverter::~DisparityConverter() = default;

void DisparityConverter::toRosMsg(std::shared_ptr<dai::ImgFrame> inData, std::deque<DisparityMsgs::DisparityImage>& outDispImageMsgs) {
    if(!inData || inData->getWidth() <= 0 || inData->getHeight() <= 0) {
        throw std::invalid_argument("Disparity image must have nonzero dimensions");
    }
    const auto type = inData->getType();
    if(type != dai::ImgFrame::Type::RAW8 && type != dai::ImgFrame::Type::RAW16) {
        throw std::invalid_argument("Disparity input must be RAW8 or RAW16");
    }
    const size_t width = inData->getWidth(), height = inData->getHeight();
    const size_t bytesPerPixel = type == dai::ImgFrame::Type::RAW8 ? 1 : 2;
    if(width > std::numeric_limits<uint32_t>::max() / sizeof(float) || height > std::numeric_limits<size_t>::max() / width / sizeof(float)
       || inData->getData().size() != width * height * bytesPerPixel) {
        throw std::invalid_argument("Disparity payload does not match its dimensions");
    }
    DisparityMsgs::DisparityImage outDispImageMsg;
    outDispImageMsg.header = getRosHeader(inData);
    outDispImageMsg.f = focalLength;
    outDispImageMsg.t = baseline;
    outDispImageMsg.min_disparity = focalLength * baseline / maxDepth;
    outDispImageMsg.max_disparity = focalLength * baseline / minDepth;
    outDispImageMsg.delta_d = bytesPerPixel == 1 ? 1.0f : 1.0f / (1u << subpixelFractionalBits);
    auto& image = outDispImageMsg.image;
    image.header = outDispImageMsg.header;
    image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    image.width = width;
    image.height = height;
    image.step = width * sizeof(float);
    image.data.resize(image.step * height);
    const uint16_t endian = 1;
    image.is_bigendian = *reinterpret_cast<const uint8_t*>(&endian) == 0;
    const auto& input = inData->getData();
    for(size_t i = 0; i < width * height; ++i) {
        // DepthAI RAW16 disparity samples are unsigned little-endian fixed point.
        const uint16_t raw = bytesPerPixel == 1 ? input[i] : uint16_t(input[2 * i]) | (uint16_t(input[2 * i + 1]) << 8);
        const float disparity = raw * outDispImageMsg.delta_d;
        std::memcpy(image.data.data() + i * sizeof(float), &disparity, sizeof(float));
    }
    outDispImageMsgs.push_back(outDispImageMsg);
    return;
}

DisparityImagePtr DisparityConverter::toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData) {
    std::deque<DisparityMsgs::DisparityImage> msgQueue;
    toRosMsg(inData, msgQueue);
    auto msg = msgQueue.front();

    DisparityImagePtr ptr = std::make_shared<DisparityMsgs::DisparityImage>(msg);

    return ptr;
}

void DisparityConverter::setSubpixelFractionalBits(unsigned int bits) {
    if(bits < 3 || bits > 5) throw std::invalid_argument("Subpixel disparity requires 3, 4 or 5 fractional bits");
    subpixelFractionalBits = bits;
}

// Getter methods
float DisparityConverter::getFocalLength() const {
    return focalLength;
}

float DisparityConverter::getBaseline() const {
    return baseline;
}

float DisparityConverter::getMinDepth() const {
    return minDepth;
}

float DisparityConverter::getMaxDepth() const {
    return maxDepth;
}
}  // namespace depthai_bridge
