#pragma once

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstddef>
#include <iterator>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/time.hpp"

namespace depthai_bridge {

enum LogLevel { DEBUG, INFO, WARN, ERROR, FATAL };

#define DEPTHAI_ROS_LOG_STREAM(loggerName, level, isOnce, args)                 \
    switch(level) {                                                             \
        case depthai_bridge::LogLevel::DEBUG:                                   \
            if(isOnce) {                                                        \
                RCLCPP_DEBUG_STREAM_ONCE(rclcpp::get_logger(loggerName), args); \
            } else {                                                            \
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger(loggerName), args);      \
            }                                                                   \
            break;                                                              \
        case depthai_bridge::LogLevel::INFO:                                    \
            if(isOnce) {                                                        \
                RCLCPP_INFO_STREAM_ONCE(rclcpp::get_logger(loggerName), args);  \
            } else {                                                            \
                RCLCPP_INFO_STREAM(rclcpp::get_logger(loggerName), args);       \
            }                                                                   \
            break;                                                              \
        case depthai_bridge::LogLevel::WARN:                                    \
            if(isOnce) {                                                        \
                RCLCPP_WARN_STREAM_ONCE(rclcpp::get_logger(loggerName), args);  \
            } else {                                                            \
                RCLCPP_WARN_STREAM(rclcpp::get_logger(loggerName), args);       \
            }                                                                   \
            break;                                                              \
        case depthai_bridge::LogLevel::ERROR:                                   \
            if(isOnce) {                                                        \
                RCLCPP_ERROR_STREAM_ONCE(rclcpp::get_logger(loggerName), args); \
            } else {                                                            \
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(loggerName), args);      \
            }                                                                   \
            break;                                                              \
        case depthai_bridge::LogLevel::FATAL:                                   \
            if(isOnce) {                                                        \
                RCLCPP_FATAL_STREAM_ONCE(rclcpp::get_logger(loggerName), args); \
            } else {                                                            \
                RCLCPP_FATAL_STREAM(rclcpp::get_logger(loggerName), args);      \
            }                                                                   \
            break;                                                              \
    }

// DEBUG stream macros on top of ROS logger
#define DEPTHAI_ROS_DEBUG_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::DEBUG, false, args)

#define DEPTHAI_ROS_DEBUG_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::DEBUG, true, args)

// INFO stream macros on top of ROS logger
#define DEPTHAI_ROS_INFO_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::INFO, false, args)

#define DEPTHAI_ROS_INFO_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::INFO, true, args)

// WARN stream macros on top of ROS logger
#define DEPTHAI_ROS_WARN_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::WARN, false, args)

#define DEPTHAI_ROS_WARN_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::WARN, true, args)

// ERROR stream macros on top of ROS logger
#define DEPTHAI_ROS_ERROR_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::ERROR, false, args)

#define DEPTHAI_ROS_ERROR_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::ERROR, true, args)

// FATAL stream macros on top of ROS logger
#define DEPTHAI_ROS_FATAL_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::FATAL, false, args)

#define DEPTHAI_ROS_FATAL_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::FATAL, true, args)

static const int64_t ZERO_TIME_DELTA_NS{100};

inline rclcpp::Time getFrameTime(rclcpp::Time rclBaseTime,
                                 std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime,
                                 std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> currTimePoint) {
    auto elapsedTime = currTimePoint - steadyBaseTime;
    auto rclStamp = rclBaseTime + elapsedTime;
    return rclStamp;
}

inline void updateBaseTime(std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime,
                           rclcpp::Time& rclBaseTime,
                           int64_t& totalNsChange,
                           const rclcpp::Clock::SharedPtr& clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)) {
    rclcpp::Time currentRosTime = clock->now();
    std::chrono::time_point<std::chrono::steady_clock> currentSteadyTime = std::chrono::steady_clock::now();
    // In nanoseconds
    auto expectedOffset = std::chrono::duration_cast<std::chrono::nanoseconds>(currentSteadyTime - steadyBaseTime).count();
    int64_t previousBaseTimeNs = rclBaseTime.nanoseconds();
    rclBaseTime = rclcpp::Time(std::max<int64_t>(0, currentRosTime.nanoseconds() - expectedOffset), clock->get_clock_type());
    int64_t newBaseTimeNs = rclBaseTime.nanoseconds();
    int64_t diff = static_cast<int64_t>(newBaseTimeNs - previousBaseTimeNs);
    totalNsChange += diff;
    if(::abs(diff) > ZERO_TIME_DELTA_NS) {
        // Has been updated
        DEPTHAI_ROS_DEBUG_STREAM("ROS BASE TIME CHANGE: ",
                                 "ROS base time changed by " << std::to_string(diff) << " ns. Total change: " << std::to_string(totalNsChange)
                                                             << " ns. New time: " << std::to_string(rclBaseTime.nanoseconds()) << " ns.");
    }
}
enum class DeviceNames { OAK_1, OAK_D, OAK_D_PRO, OAK_D_POE, OAK_THERMAL, OAK_SR, OAK_D_SR_POE };

const std::unordered_map<DeviceNames, std::string> deviceNameMap = {
    {DeviceNames::OAK_1, "OAK-1"},
    {DeviceNames::OAK_D, "OAK-D"},
    {DeviceNames::OAK_D_PRO, "OAK-D-PRO"},
    {DeviceNames::OAK_D_POE, "OAK-D-POE"},
    {DeviceNames::OAK_THERMAL, "OAK-T"},
    {DeviceNames::OAK_SR, "OAK-SR"},
    {DeviceNames::OAK_D_SR_POE, "OAK-D-SR-POE"},
};

const std::unordered_map<dai::CameraBoardSocket, std::string> defaultSocketMap = {
    {dai::CameraBoardSocket::AUTO, "rgb"},
    {dai::CameraBoardSocket::CAM_A, "rgb"},
    {dai::CameraBoardSocket::CAM_B, "left"},
    {dai::CameraBoardSocket::CAM_C, "right"},
    {dai::CameraBoardSocket::CAM_D, "left_back"},
    {dai::CameraBoardSocket::CAM_E, "right_back"},
};

const std::unordered_map<dai::CameraBoardSocket, std::string> letterSocketMap = {
    {dai::CameraBoardSocket::AUTO, "auto"},
    {dai::CameraBoardSocket::CAM_A, "cam_a"},
    {dai::CameraBoardSocket::CAM_B, "cam_b"},
    {dai::CameraBoardSocket::CAM_C, "cam_c"},
    {dai::CameraBoardSocket::CAM_D, "cam_d"},
    {dai::CameraBoardSocket::CAM_E, "cam_e"},
};

const std::unordered_map<dai::CameraBoardSocket, std::string> srDPoeSocketMap = {
    {dai::CameraBoardSocket::AUTO, "tof"},
    {dai::CameraBoardSocket::CAM_A, "tof"},
    {dai::CameraBoardSocket::CAM_B, "left"},
    {dai::CameraBoardSocket::CAM_C, "right"},
};

const std::unordered_map<dai::CameraBoardSocket, std::string> thermalSocketMap = {
    {dai::CameraBoardSocket::AUTO, "rgb"},
    {dai::CameraBoardSocket::CAM_A, "rgb"},
    {dai::CameraBoardSocket::CAM_B, "left"},
    {dai::CameraBoardSocket::CAM_C, "right"},
    {dai::CameraBoardSocket::CAM_D, "left_back"},
    {dai::CameraBoardSocket::CAM_E, "thermal"},
};

const std::unordered_map<dai::CameraBoardSocket, std::string> rsSocketNameMap = {
    {dai::CameraBoardSocket::AUTO, "color"},
    {dai::CameraBoardSocket::CAM_A, "color"},
    {dai::CameraBoardSocket::CAM_B, "infra2"},
    {dai::CameraBoardSocket::CAM_C, "infra1"},
    {dai::CameraBoardSocket::CAM_E, "infra4"},
    {dai::CameraBoardSocket::CAM_D, "infra3"},
};

inline std::string normalizeFramePrefix(const std::string& prefix) {
    const auto first = prefix.find_first_not_of('/');
    if(first == std::string::npos) {
        return "";
    }
    return prefix.substr(first, prefix.find_last_not_of('/') - first + 1);
}

inline std::string resolveFramePrefix(const std::string& prefix, const std::string& fallback) {
    const auto normalizedPrefix = normalizeFramePrefix(prefix);
    return normalizedPrefix.empty() ? normalizeFramePrefix(fallback) : normalizedPrefix;
}

inline std::string normalizeDeviceModelName(const std::string& modelName) {
    std::string normalized;
    normalized.reserve(modelName.size());
    for(const auto c : modelName) {
        const auto value = static_cast<unsigned char>(c);
        if(std::isspace(value) || c == '_') {
            if(!normalized.empty() && normalized.back() != '-') {
                normalized.push_back('-');
            }
        } else {
            normalized.push_back(static_cast<char>(std::toupper(value)));
        }
    }
    while(!normalized.empty() && normalized.back() == '-') {
        normalized.pop_back();
    }

    static const std::unordered_map<std::string, std::string> legacyBoardNames = {
        {"BW1098", "OAK-D"}, {"BW1098OBC", "OAK-D"}, {"DM9095", "OAK-D-LITE"}, {"DM9098", "OAK-D-S2"}};
    const auto legacyName = legacyBoardNames.find(normalized);
    if(legacyName != legacyBoardNames.end()) {
        return legacyName->second;
    }

    if(normalized.rfind("OAK-", 0) != 0 && normalized.rfind("OAK4-", 0) != 0) {
        return normalized;
    }

    static const std::unordered_set<std::string> variantTokens = {"AF", "FF", "97", "9782", "OV9782", "PB", "CUSTOM", "DEV"};
    std::string canonical;
    size_t start = 0;
    while(start <= normalized.size()) {
        const auto end = normalized.find('-', start);
        const auto token = normalized.substr(start, end == std::string::npos ? std::string::npos : end - start);
        const bool configurationCode = token.size() > 1 && token.front() == 'C' && std::all_of(token.begin() + 1, token.end(), [](const auto value) {
                                           return std::isdigit(static_cast<unsigned char>(value));
                                       });
        const bool numberedFocusVariant = token.rfind("AF#", 0) == 0 || token.rfind("FF#", 0) == 0;
        if(!token.empty() && variantTokens.count(token) == 0 && !configurationCode && !numberedFocusVariant) {
            if(!canonical.empty()) {
                canonical.push_back('-');
            }
            canonical += token;
        }
        if(end == std::string::npos) {
            break;
        }
        start = end + 1;
    }
    return canonical;
}

inline bool isOakDeviceModel(const std::string& modelName) {
    return modelName.rfind("OAK-", 0) == 0 || modelName.rfind("OAK4-", 0) == 0;
}

inline std::string resolveDeviceModelName(const std::string& deviceName, const std::string& boardName, size_t cameraCount) {
    const auto normalizedDeviceName = normalizeDeviceModelName(deviceName);
    if(isOakDeviceModel(normalizedDeviceName)) {
        return normalizedDeviceName;
    }

    const auto normalizedBoardName = normalizeDeviceModelName(boardName);
    if(isOakDeviceModel(normalizedBoardName)) {
        return normalizedBoardName;
    }

    // Very old or custom EEPROMs may expose only an internal board code. Keep
    // these devices usable by falling back to their connected camera topology.
    if(cameraCount >= 3) {
        return "OAK-D";
    }
    if(cameraCount == 1) {
        return "OAK-1";
    }
    if(!normalizedDeviceName.empty()) {
        return normalizedDeviceName;
    }
    if(!normalizedBoardName.empty()) {
        return normalizedBoardName;
    }
    return "UNKNOWN";
}

inline bool isBno08x(const std::string& imuName) {
    std::string normalized;
    normalized.reserve(imuName.size());
    std::transform(imuName.begin(), imuName.end(), std::back_inserter(normalized), [](const auto value) {
        return static_cast<char>(std::toupper(static_cast<unsigned char>(value)));
    });
    return normalized.rfind("BNO08", 0) == 0;
}

inline std::string getFrameName(const std::string& prefix, const std::string& frameName) {
    const auto normalizedPrefix = normalizeFramePrefix(prefix);
    return normalizedPrefix.empty() ? frameName : normalizedPrefix + "_" + frameName;
}

inline std::string getOpticalFrameName(const std::string& prefix, const std::string& frameName, bool rsCompat = false) {
    std::string suffix = "_camera_optical_frame";
    if(rsCompat) {
        suffix = "_optical_frame";
    }
    return getFrameName(prefix, frameName) + suffix;
}

inline std::string getSocketName(dai::CameraBoardSocket socketNum, const std::string& deviceName = "", bool rsCompat = false, bool useSocketNames = false) {
    std::string name = "";
    try {
        name = defaultSocketMap.at(socketNum);
        if(rsCompat) {
            name = rsSocketNameMap.at(socketNum);
        }
        if(deviceName.empty()) {
            if(useSocketNames) {
                name = letterSocketMap.at(socketNum);
            }
        } else {
            if(deviceName == deviceNameMap.at(DeviceNames::OAK_D_SR_POE)) {
                name = srDPoeSocketMap.at(socketNum);
            } else if(deviceName == deviceNameMap.at(DeviceNames::OAK_THERMAL)) {
                name = thermalSocketMap.at(socketNum);
            }
        }
    } catch(std::out_of_range) {
        DEPTHAI_ROS_ERROR_STREAM("depthai_bridge", "Couldn't find socket name for device: " << deviceName << ". Socket ID: " << static_cast<int>(socketNum));
        throw std::runtime_error("Socket name not found");
    }
    return name;
}

}  // namespace depthai_bridge
