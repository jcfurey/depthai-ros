#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_ros_driver_v3/dai_nodes/base_node.hpp"
#include "depthai_ros_driver_v3/param_handlers/driver_param_handler.hpp"
#include "depthai_ros_driver_v3/pipeline/pipeline_generator.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/node.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace dai {
class Pipeline;
class Device;
enum class Platform;
}  // namespace dai

namespace depthai_ros_driver {
using Trigger = std_srvs::srv::Trigger;
class Driver : public rclcpp::Node {
   public:
    explicit Driver(const rclcpp::NodeOptions& options);
    ~Driver();
    /**
     * @brief Creates the pipeline and starts the device. Also sets up parameter callback and services.
     */
    void onConfigure();

   private:
    /**
     * @brief      Print information about the device type.
     * @return     false if the device could not be started (e.g. shutdown was requested while waiting for one).
     */
    bool getDeviceType();
    /**
     * @brief      Create the pipeline by using PipelineGenerator.
     */
    void createPipeline();
    /**
     * @brief      Connect either to a first available device or to a device with a specific USB port, MXID or IP. Loops until a device is found or shutdown is
     * requested.
     * @return     false if shutdown was requested before a device was found.
     */
    bool startDevice();
    /**
     * @brief      Sets up the queues and creates publishers for the nodes in the pipeline.
     */
    void setupQueues();
    /**
     * @brief Sets IR floodlight and dot pattern projector.
     */
    void setIR();
    /**
     * @brief Saves pipeline as a json to a file.
     */
    void savePipeline();
    /**
     * @brief Saves calibration data to a json file.
     */
    void saveCalib();
    /**
     * @brief Loads calibration data from a path.
     * @param path Path to the calibration file.
     */
    void loadCalib(const std::string& path);
    rcl_interfaces::msg::SetParametersResult parameterCB(const std::vector<rclcpp::Parameter>& params);
    OnSetParametersCallbackHandle::SharedPtr paramCBHandle;
    std::unique_ptr<param_handlers::DriverParamHandler> ph;
    rclcpp::Service<Trigger>::SharedPtr startSrv, stopSrv, savePipelineSrv, saveCalibSrv;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagSub;
    /*
     * Closes all the queues, clears the configured BaseNodes, stops the pipeline and resets the device. Thread-safe.
     */
    void stop();
    /*
     * Runs onConfigure(); Thread-safe.
     */
    void start();
    void restart();
    /*
     * Unsynchronized implementations; callers must hold lifecycleMtx.
     */
    void stopImpl();
    void startImpl();
    void diagCB(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);

    void startCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res);
    void stopCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res);
    void saveCalibCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res);
    void savePipelineCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res);
    std::vector<std::string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};
    std::shared_ptr<dai::Pipeline> pipeline;
    std::shared_ptr<dai::Device> device;
    std::string deviceName;
    std::unique_ptr<pipeline_gen::PipelineGenerator> generator;
    dai::Platform platform;
    std::atomic<bool> camRunning = false;
    std::atomic<bool> starting = false;
    std::unique_ptr<depthai_bridge::TFPublisher> tfPub;
    rclcpp::TimerBase::SharedPtr startTimer;
    rclcpp::CallbackGroup::SharedPtr srvGroup;
    // Serializes start/stop/restart across the reentrant service group, the shutdown callback and the destructor.
    std::mutex lifecycleMtx;
    // Kept so the on_shutdown callback (which captures `this`) can be deregistered on destruction.
    rclcpp::Context::SharedPtr rclContext;
    rclcpp::OnShutdownCallbackHandle shutdownCBHandle;
};
}  // namespace depthai_ros_driver
