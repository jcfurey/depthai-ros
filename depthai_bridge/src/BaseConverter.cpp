#include "depthai_bridge/BaseConverter.hpp"

#include <memory>
#include <stdexcept>

#include "depthai/common/CameraExposureOffset.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai_bridge/depthaiUtility.hpp"

namespace depthai_bridge {

BaseConverter::BaseConverter(std::string frameName, bool getBaseDeviceTimestamp)
    : frameName(std::move(frameName)),
      steadyBaseTime(std::chrono::steady_clock::now()),
      rosBaseTime(rosClock->now()),
      getBaseDeviceTimestamp(getBaseDeviceTimestamp),
      totalNsChange(0),
      updateRosBaseTimeOnToRosMsg(false) {}

BaseConverter::~BaseConverter() = default;

void BaseConverter::updateRosBaseTime() {
    std::lock_guard<std::mutex> lock(*clockMutex);
    steadyBaseTime = std::chrono::steady_clock::now();
    rosBaseTime = rosClock->now();
}

void BaseConverter::setClock(rclcpp::Clock::SharedPtr clock) {
    if(!clock) throw std::invalid_argument("Converter clock cannot be null");
    std::lock_guard<std::mutex> lock(*clockMutex);
    rosClock = std::move(clock);
    steadyBaseTime = std::chrono::steady_clock::now();
    rosBaseTime = rosClock->now();
}

rclcpp::Time BaseConverter::toRosTime(std::chrono::steady_clock::time_point timestamp) {
    std::lock_guard<std::mutex> lock(*clockMutex);
    const auto now = rosClock->now();
    if(rosClock->get_clock_type() == RCL_ROS_TIME && rosClock->ros_time_is_active()) {
        // Includes zero before the first /clock tick, pauses and clock jumps.
        return now;
    }
    const auto steadyNow = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(steadyNow - steadyBaseTime).count();
    const auto expected = rosBaseTime.nanoseconds() + elapsed;
    const auto shift = now.nanoseconds() - expected;
    if(updateRosBaseTimeOnToRosMsg || std::abs(shift) > 1000000000LL || rosBaseTime.get_clock_type() != now.get_clock_type()) {
        steadyBaseTime = steadyNow;
        rosBaseTime = now;
    }
    const auto offset = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - steadyBaseTime).count();
    return rclcpp::Time(std::max<int64_t>(0, rosBaseTime.nanoseconds() + offset), rosClock->get_clock_type());
}

std_msgs::msg::Header BaseConverter::getRosHeader(const std::shared_ptr<dai::Buffer>& inData, bool addExpOffset, dai::CameraExposureOffset offset) {
    if(!inData) throw std::invalid_argument("Converter input cannot be null");
    std_msgs::msg::Header header;
    header.frame_id = frameName;
    auto tstamp = getBaseDeviceTimestamp ? inData->getTimestampDevice() : inData->getTimestamp();
    if(addExpOffset) {
        std::chrono::microseconds exposure;
        if(const auto image = std::dynamic_pointer_cast<dai::ImgFrame>(inData))
            exposure = image->getExposureTime();
        else if(const auto encoded = std::dynamic_pointer_cast<dai::EncodedFrame>(inData))
            exposure = encoded->getExposureTime();
        else
            throw std::invalid_argument("Exposure offset requires an image or encoded frame");
        if(offset == dai::CameraExposureOffset::START)
            tstamp -= exposure;
        else if(offset == dai::CameraExposureOffset::MIDDLE)
            tstamp -= exposure / 2;
    }
    header.stamp = toRosTime(tstamp);
    return header;
}
}  // namespace depthai_bridge
