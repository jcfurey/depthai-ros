#pragma once

#include <chrono>
#include <deque>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>

#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai_bridge/BaseConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "depthai_ros_msgs/msg/imu_with_magnetic_field.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

namespace depthai_bridge {

namespace ImuMsgs = sensor_msgs::msg;
using ImuPtr = ImuMsgs::Imu::SharedPtr;

enum class ImuSyncMethod { COPY, LINEAR_INTERPOLATE_GYRO, LINEAR_INTERPOLATE_ACCEL };

class ImuConverter : public BaseConverter {
   public:
    // magnetic_field_cov is the ROS magnetic-field variance in tesla squared.
    explicit ImuConverter(const std::string& frameName,
                          ImuSyncMethod syncMode = ImuSyncMethod::COPY,
                          double linear_accel_cov = 0.0,
                          double angular_velocity_cov = 0.0,
                          double rotation_cov = 0.0,
                          double magnetic_field_cov = 0.0,
                          bool enable_rotation = false,
                          bool enable_magn = false,
                          bool getBaseDeviceTimestamp = false);
    ~ImuConverter();

    void toRosMsg(std::shared_ptr<dai::IMUData> inData, std::deque<ImuMsgs::Imu>& outImuMsgs);
    void toRosDaiMsg(std::shared_ptr<dai::IMUData> inData, std::deque<depthai_ros_msgs::msg::ImuWithMagneticField>& outImuMsgs);

    template <typename T>
    T lerp(const T& a, const T& b, const double t) {
        return a * (1.0 - t) + b * t;
    }

    template <typename T>
    T lerpImu(const T& a, const T& b, const double t) {
        T res;
        res.x = lerp(a.x, b.x, t);
        res.y = lerp(a.y, b.y, t);
        res.z = lerp(a.z, b.z, t);
        return res;
    }

   private:
    std::deque<dai::IMUReportAccelerometer> accelHist;
    std::deque<dai::IMUReportGyroscope> gyroHist;
    std::deque<dai::IMUReportRotationVectorWAcc> rotationHist;
    std::deque<dai::IMUReportMagneticField> magnHist;
    template <typename T>
    void FillImuData_LinearInterpolation(std::vector<dai::IMUPacket>& imuPackets, std::deque<T>& imuMsgs) {
        for(const auto& packet : imuPackets) {
            appendIfNew(accelHist, packet.acceleroMeter);
            appendIfNew(gyroHist, packet.gyroscope);
            if(enable_rotation) {
                appendIfNew(rotationHist, packet.rotationVector);
            }
            if(enable_magn) {
                appendIfNew(magnHist, packet.magneticField);
            }

            if(syncMode == ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL) {
                if(accelHist.size() < 3) {
                    continue;
                }
                interpolate(accelHist, gyroHist, imuMsgs);
            } else if(syncMode == ImuSyncMethod::LINEAR_INTERPOLATE_GYRO) {
                if(gyroHist.size() < 3) {
                    continue;
                }
                interpolate(gyroHist, accelHist, imuMsgs);
            }
        }
    }

    uint32_t sequenceNum;
    double linear_accel_cov, angular_velocity_cov, rotation_cov, magnetic_field_cov;
    bool enable_rotation;
    bool enable_magn;
    ImuSyncMethod syncMode;
    static void setOrientationUnknown(ImuMsgs::Imu& msg);
    void fillImuMsg(ImuMsgs::Imu& msg, dai::IMUReportAccelerometer report);
    void fillImuMsg(ImuMsgs::Imu& msg, dai::IMUReportGyroscope report);
    void fillImuMsg(ImuMsgs::Imu& msg, dai::IMUReportRotationVectorWAcc report);
    void fillImuMsg(ImuMsgs::Imu& msg, dai::IMUReportMagneticField report);

    void fillImuMsg(depthai_ros_msgs::msg::ImuWithMagneticField& msg, dai::IMUReportAccelerometer report);
    void fillImuMsg(depthai_ros_msgs::msg::ImuWithMagneticField& msg, dai::IMUReportGyroscope report);
    void fillImuMsg(depthai_ros_msgs::msg::ImuWithMagneticField& msg, dai::IMUReportRotationVectorWAcc report);
    void fillImuMsg(depthai_ros_msgs::msg::ImuWithMagneticField& msg, dai::IMUReportMagneticField report);

    template <typename I, typename S, typename T, typename F, typename M>
    void CreateUnitMessage(M& msg, std::chrono::steady_clock::time_point timestamp, I first, S second, T third, F fourth) {
        fillImuMsg(msg, first);
        fillImuMsg(msg, second);
        fillImuMsg(msg, third);
        fillImuMsg(msg, fourth);

        msg.header.frame_id = frameName;

        msg.header.stamp = toRosTime(timestamp);
    }

    template <typename I, typename S, typename T, typename M>
    void CreateUnitMessage(M& msg, std::chrono::steady_clock::time_point timestamp, I first, S second, T third) {
        fillImuMsg(msg, first);
        fillImuMsg(msg, second);
        fillImuMsg(msg, third);

        msg.header.frame_id = frameName;

        msg.header.stamp = toRosTime(timestamp);
    }

    template <typename I, typename S, typename M>
    void CreateUnitMessage(M& msg, std::chrono::steady_clock::time_point timestamp, I first, S second) {
        fillImuMsg(msg, first);
        fillImuMsg(msg, second);

        msg.header.frame_id = frameName;

        msg.header.stamp = toRosTime(timestamp);
    }

    template <typename M>
    static ImuMsgs::Imu& imuOf(M& msg) {
        return msg.imu;
    }
    static ImuMsgs::Imu& imuOf(ImuMsgs::Imu& msg) {
        return msg;
    }

    template <typename R>
    static void appendIfNew(std::deque<R>& hist, const R& report) {
        if(hist.empty() || hist.back().sequence != report.sequence) {
            hist.push_back(report);
        }
    }

    // Returns the newest report generated at or before ts, discarding older history.
    template <typename R>
    static const R* latestAtOrBefore(std::deque<R>& hist, std::chrono::steady_clock::time_point ts) {
        while(hist.size() > 1 && hist[1].getTimestamp() <= ts) {
            hist.pop_front();
        }
        if(hist.empty() || hist.front().getTimestamp() > ts) {
            return nullptr;
        }
        return &hist.front();
    }

    // Rotation and magnetic field arrive at their own rates, so they are sampled-and-held by
    // timestamp rather than paired positionally with the interpolated streams.
    template <typename M>
    void fillAuxiliary(M& msg, std::chrono::steady_clock::time_point ts) {
        const dai::IMUReportRotationVectorWAcc* rot = enable_rotation ? latestAtOrBefore(rotationHist, ts) : nullptr;
        if(rot != nullptr) {
            fillImuMsg(msg, *rot);
        } else {
            setOrientationUnknown(imuOf(msg));
        }
        if(enable_magn) {
            if(const auto* magn = latestAtOrBefore(magnHist, ts)) {
                fillImuMsg(msg, *magn);
            }
        }
    }

    template <typename I, typename S, typename M>
    void interpolate(std::deque<I>& interpolated, std::deque<S>& second, std::deque<M>& imuMsgs) {
        I interp0, interp1;
        S currSecond;
        interp0.sequence = -1;
        while(interpolated.size()) {
            if(interp0.sequence == -1) {
                interp0 = interpolated.front();
                interpolated.pop_front();
            } else {
                interp1 = interpolated.front();
                interpolated.pop_front();
                // remove std::milli to get in seconds
                std::chrono::duration<double, std::milli> duration_ms = interp1.timestamp.get() - interp0.timestamp.get();
                double dt = duration_ms.count();
                while(second.size()) {
                    currSecond = second.front();
                    if(currSecond.timestamp.get() > interp0.timestamp.get() && currSecond.timestamp.get() <= interp1.timestamp.get()) {
                        // remove std::milli to get in seconds
                        std::chrono::duration<double, std::milli> diff = currSecond.timestamp.get() - interp0.timestamp.get();
                        const double alpha = diff.count() / dt;
                        I interp = lerpImu(interp0, interp1, alpha);
                        M msg;
                        std::chrono::steady_clock::time_point tstamp;
                        if(getBaseDeviceTimestamp)
                            tstamp = currSecond.getTimestampDevice();
                        else
                            tstamp = currSecond.getTimestamp();
                        CreateUnitMessage(msg, tstamp, interp, currSecond);
                        fillAuxiliary(msg, currSecond.getTimestamp());
                        imuMsgs.push_back(std::move(msg));
                        second.pop_front();
                    } else if(currSecond.timestamp.get() > interp1.timestamp.get()) {
                        interp0 = interp1;
                        if(interpolated.size()) {
                            interp1 = interpolated.front();
                            interpolated.pop_front();
                            duration_ms = interp1.timestamp.get() - interp0.timestamp.get();
                            dt = duration_ms.count();
                        } else {
                            break;
                        }
                    } else {
                        second.pop_front();
                    }
                }
                interp0 = interp1;
            }
        }
        interpolated.push_back(interp0);
    }
};

}  // namespace depthai_bridge
