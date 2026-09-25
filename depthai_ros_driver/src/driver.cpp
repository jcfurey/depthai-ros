#include "depthai_ros_driver/driver.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_ros_driver/managed_lifecycle.hpp"
#include "depthai_ros_driver/pipeline/pipeline_generator.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/version.h"

namespace depthai_ros_driver {
namespace {
using LifecycleTransition = lifecycle_msgs::msg::Transition;
using LifecycleState = lifecycle_msgs::msg::State;

// Post-set callbacks also run for our own defaults. Only user changes should
// take a stream out of automatic transport management. Keep this per thread so
// an unrelated parameter request cannot be mistaken for an internal update.
thread_local const Driver* transportDefaultsOwner = nullptr;

struct TransportDefaultsScope {
    explicit TransportDefaultsScope(const Driver* driver) : previous(transportDefaultsOwner) {
        transportDefaultsOwner = driver;
    }
    ~TransportDefaultsScope() {
        transportDefaultsOwner = previous;
    }
    const Driver* previous;
};

bool isConnectableState(XLinkDeviceState_t state) {
    return state == X_LINK_ANY_STATE || state == X_LINK_UNBOOTED || state == X_LINK_BOOTLOADER || state == X_LINK_FLASH_BOOTED || state == X_LINK_GATE
           || state == X_LINK_GATE_SETUP;
}

bool isBootedState(XLinkDeviceState_t state) {
    return state == X_LINK_BOOTED || state == X_LINK_BOOTED_NON_EXCLUSIVE || state == X_LINK_GATE_BOOTED;
}

}  // namespace

Driver::Driver(const rclcpp::NodeOptions& options) : rclcpp::Node("oak", options) {
    paramCBHandle = this->add_on_set_parameters_callback(std::bind(&Driver::parameterCB, this, std::placeholders::_1));
    postParamCBHandle = this->add_post_set_parameters_callback(std::bind(&Driver::parametersAppliedCB, this, std::placeholders::_1));
    //  Since we cannot use shared_from this before the object is initialized, we need to use a timer to start the device.
    // Close DepthAI queues while ROS publishers and logging are still valid. Setting
    // shutdownRequested first also lets a pending device-discovery loop unwind.
    rclContext = options.context();
    managedLifecycle = std::make_unique<ManagedLifecycle>(*this, lifecycleMtx, [this](uint8_t transition) { return lifecycleAction(transition); });
    rcl_interfaces::msg::ParameterDescriptor autostartDescriptor;
    autostartDescriptor.read_only = true;
    autostartDescriptor.description = "Automatically configure and activate at startup. Set false for external lifecycle management.";
    const bool autostart = has_parameter("driver.i_autostart") ? get_parameter("driver.i_autostart").as_bool()
                                                               : declare_parameter<bool>("driver.i_autostart", true, autostartDescriptor);
    srvGroup = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

#if RCLCPP_VERSION_MAJOR >= 28
    startSrv = this->create_service<Trigger>(
        "~/start_driver", std::bind(&Driver::startCB, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), srvGroup);
    stopSrv = this->create_service<Trigger>(
        "~/stop_driver", std::bind(&Driver::stopCB, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), srvGroup);
    startAliasSrv = this->create_service<Trigger>(
        "~/start", std::bind(&Driver::startCB, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), srvGroup);
    stopAliasSrv = this->create_service<Trigger>(
        "~/stop", std::bind(&Driver::stopCB, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), srvGroup);
    savePipelineSrv = this->create_service<Trigger>(
        "~/save_pipeline", std::bind(&Driver::savePipelineCB, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), srvGroup);
    saveCalibSrv = this->create_service<Trigger>(
        "~/save_calibration", std::bind(&Driver::saveCalibCB, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), srvGroup);
#else
    startSrv = this->create_service<Trigger>("~/start_driver",
                                             std::bind(&Driver::startCB, this, std::placeholders::_1, std::placeholders::_2),
                                             rclcpp::ServicesQoS().get_rmw_qos_profile(),
                                             srvGroup);
    stopSrv = this->create_service<Trigger>(
        "~/stop_driver", std::bind(&Driver::stopCB, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS().get_rmw_qos_profile(), srvGroup);
    startAliasSrv = this->create_service<Trigger>(
        "~/start", std::bind(&Driver::startCB, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS().get_rmw_qos_profile(), srvGroup);
    stopAliasSrv = this->create_service<Trigger>(
        "~/stop", std::bind(&Driver::stopCB, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS().get_rmw_qos_profile(), srvGroup);
    savePipelineSrv = this->create_service<Trigger>("~/save_pipeline",
                                                    std::bind(&Driver::savePipelineCB, this, std::placeholders::_1, std::placeholders::_2),
                                                    rclcpp::ServicesQoS().get_rmw_qos_profile(),
                                                    srvGroup);
    saveCalibSrv = this->create_service<Trigger>("~/save_calibration",
                                                 std::bind(&Driver::saveCalibCB, this, std::placeholders::_1, std::placeholders::_2),
                                                 rclcpp::ServicesQoS().get_rmw_qos_profile(),
                                                 srvGroup);
#endif

    diagSub = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10, std::bind(&Driver::diagCB, this, std::placeholders::_1));
    statusPublisher = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    statusTimer = create_wall_timer(std::chrono::seconds(1), [this]() { publishStatus(); });
    parameterTimer = create_wall_timer(std::chrono::milliseconds(20), [this]() { applyPendingParameters(); });
    startTimer = create_wall_timer(std::chrono::seconds(1), [this, autostart]() {
        startTimer->cancel();
        if(autostart && !shutdownRequested) start();
    });
    RCLCPP_INFO(get_logger(), "Driver lifecycle services ready (%s).", autostart ? "autostart enabled" : "waiting for configure/activate");
    preShutdownCBHandle = rclContext->add_pre_shutdown_callback([this]() {
        shutdownRequested = true;
        try {
            stop();
        } catch(const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to stop driver during shutdown: %s", e.what());
        }
    });
}

Driver::~Driver() {
    startTimer->cancel();
    parameterTimer->cancel();
    statusTimer->cancel();
    if(rclContext) {
        rclContext->remove_pre_shutdown_callback(preShutdownCBHandle);
    }
    shutdownRequested = true;
    stop();
    managedLifecycle.reset();
}

std::shared_ptr<rclcpp::Node> Driver::getNodeHandle() {
    // The Driver owns these helpers and stops/drains their callbacks before
    // destroying them. Borrow the node to avoid Driver -> helper -> Driver
    // ownership cycles. This handle must not escape the Driver's lifetime.
    return std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {});
}

void Driver::onConfigure() {
    std::lock_guard<std::recursive_mutex> lock(lifecycleMtx);
    ph = std::make_unique<param_handlers::DriverParamHandler>(getNodeHandle(), "driver");
    ph->declareParams();
    if(!getDeviceType()) {
        RCLCPP_WARN(get_logger(), "Shutdown requested before a device was found, aborting startup.");
        return;
    }
    configureTransportDefaults();
    createPipeline();
    setupQueues();
    // If model name not set get one from the device
    std::string camModel = ph->getParam<std::string>("i_tf_device_model");
    if(camModel.empty()) {
        camModel = deviceName;
    }

    if(ph->getParam<bool>("i_publish_tf_from_calibration")) {
        try {
            tfPub = std::make_unique<depthai_bridge::TFPublisher>(getNodeHandle(),
                                                                  device->getCalibration(),
                                                                  device->getConnectedCameraFeatures(),
                                                                  ph->getParam<std::string>("i_tf_device_name"),
                                                                  camModel,
                                                                  ph->getParam<std::string>("i_tf_base_frame"),
                                                                  ph->getParam<std::string>("i_tf_parent_frame"),
                                                                  ph->getParam<std::string>("i_tf_cam_pos_x"),
                                                                  ph->getParam<std::string>("i_tf_cam_pos_y"),
                                                                  ph->getParam<std::string>("i_tf_cam_pos_z"),
                                                                  ph->getParam<std::string>("i_tf_cam_roll"),
                                                                  ph->getParam<std::string>("i_tf_cam_pitch"),
                                                                  ph->getParam<std::string>("i_tf_cam_yaw"),
                                                                  ph->getParam<std::string>("i_tf_imu_from_descr"),
                                                                  ph->getParam<std::string>("i_tf_custom_urdf_location"),
                                                                  ph->getParam<std::string>("i_tf_custom_xacro_args"),
                                                                  ph->getParam<bool>("i_rs_compat"),
                                                                  ph->getParam<std::string>("i_tf_prefix"));
        } catch(const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Could not publish TF from device calibration: %s. Camera streaming will continue without calibration TF.", e.what());
        }
    }
    configurationDirty = false;
    const auto* rosDistro = std::getenv("ROS_DISTRO");
    RCLCPP_INFO(get_logger(),
                "If you detect any issues with %s release, please report "
                "issues to GH: https://github.com/luxonis/depthai-ros/issues/719",
                rosDistro != nullptr ? rosDistro : "the current ROS");
}

void Driver::diagCB(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    std::lock_guard<std::recursive_mutex> lock(lifecycleMtx);
    if(!ph || shutdownRequested || !camRunning) {
        return;
    }
    for(const auto& status : msg->status) {
        const std::string suffix = ": sys_logger";
        const std::string hardwarePrefix = std::string(get_fully_qualified_name()) + "_";
        if(status.name.size() >= suffix.size() && status.name.compare(status.name.size() - suffix.size(), suffix.size(), suffix) == 0
           && status.hardware_id.rfind(hardwarePrefix, 0) == 0) {
            if(status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
                RCLCPP_ERROR(get_logger(), "Driver diagnostics error: %s", status.message.c_str());
                if(ph->getParam<bool>("i_restart_on_diagnostics_error")) {
                    restart();
                };
            }
        }
    }
}

void Driver::start() {
    std::lock_guard<std::recursive_mutex> lock(lifecycleMtx);
    startImpl();
}

void Driver::startImpl() {
    if(shutdownRequested || !managedLifecycle) return;
    if(managedLifecycle->state() == LifecycleState::PRIMARY_STATE_UNCONFIGURED) {
        if(!managedLifecycle->change(LifecycleTransition::TRANSITION_CONFIGURE)) return;
    }
    if(managedLifecycle->state() == LifecycleState::PRIMARY_STATE_INACTIVE) {
        managedLifecycle->change(LifecycleTransition::TRANSITION_ACTIVATE);
    }
}

bool Driver::lifecycleAction(uint8_t transition) {
    if(transition == LifecycleTransition::TRANSITION_CONFIGURE) {
        onConfigure();
        if(!pipeline || shutdownRequested) throw std::runtime_error("Camera configuration was interrupted");
    } else if(transition == LifecycleTransition::TRANSITION_ACTIVATE) {
        if(shutdownRequested) return false;
        if(configurationDirty || !pipeline) {
            stopImpl();
            onConfigure();
        }
        if(!pipeline || shutdownRequested) throw std::runtime_error("Camera activation was interrupted");
        setIR();
        pipeline->start();
        camRunning = true;
        parameterApplyError.clear();
        RCLCPP_INFO(get_logger(), "Driver ready! Camera streaming.");
    } else {
        // SDK pipelines are rebuilt on reactivation. Fully drain producers and
        // release device ownership while inactive, cleaned up, or in error.
        stopImpl();
    }
    return true;
}

void Driver::stop() {
    std::lock_guard<std::recursive_mutex> lock(lifecycleMtx);
    if(managedLifecycle) {
        const auto state = managedLifecycle->state();
        if(shutdownRequested && state >= LifecycleState::PRIMARY_STATE_UNCONFIGURED && state <= LifecycleState::PRIMARY_STATE_ACTIVE) {
            managedLifecycle->change(LifecycleTransition::TRANSITION_UNCONFIGURED_SHUTDOWN + state - LifecycleState::PRIMARY_STATE_UNCONFIGURED);
            return;
        }
        if(state == LifecycleState::PRIMARY_STATE_ACTIVE) {
            managedLifecycle->change(LifecycleTransition::TRANSITION_DEACTIVATE);
            return;
        }
    }
    stopImpl();
}

void Driver::stopImpl() {
    const bool hadResources = camRunning || generator || pipeline || device || tfPub;
    if(!hadResources) {
        if(rclcpp::ok()) {
            RCLCPP_INFO(get_logger(), "Driver is already stopped.");
        }
        return;
    }

    if(rclcpp::ok()) {
        RCLCPP_INFO(get_logger(), "Stopping driver.");
    }
    if(generator) {
        try {
            generator->closeQueues();
        } catch(const std::exception& e) {
            if(rclcpp::ok()) {
                RCLCPP_WARN(get_logger(), "Failed to close driver queues: %s", e.what());
            }
        }
    }
    if(pipeline) {
        try {
            pipeline->stop();
            pipeline->wait();
        } catch(const std::exception& e) {
            if(rclcpp::ok()) {
                RCLCPP_WARN(get_logger(), "Failed to stop the DepthAI pipeline: %s", e.what());
            }
        }
    }
    {
        std::lock_guard<std::mutex> lock(pendingParamsMtx);
        pendingParams.clear();
    }
    tfPub.reset();
    generator.reset();
    pipeline.reset();
    device.reset();
    camRunning = false;
    if(rclcpp::ok()) {
        RCLCPP_INFO(get_logger(), "Driver stopped!");
    }
}

void Driver::restart() {
    std::lock_guard<std::recursive_mutex> lock(lifecycleMtx);
    RCLCPP_WARN(get_logger(), "Restarting driver");
    ++restartCount;
    stop();
    startImpl();
    if(!camRunning) {
        ++restartFailures;
        RCLCPP_ERROR(get_logger(), "Restarting driver failed.");
    }
}

void Driver::saveCalib() {
    // Save the calibration in use, including any external or auto-calibration override.
    auto calibHandler = device->getCalibration();
    std::stringstream savePath;
    savePath << "/tmp/" << device->getDeviceId().c_str() << "_calibration.json";
    RCLCPP_INFO(get_logger(), "Saving calibration to: %s", savePath.str().c_str());
    calibHandler.eepromToJsonFile(savePath.str());
}

void Driver::loadCalib(const std::string& path) {
    RCLCPP_INFO(get_logger(), "Reading calibration from: %s", path.c_str());
    dai::CalibrationHandler cH(path);
    pipeline->setCalibrationData(cH);
}

void Driver::saveCalibCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res) {
    std::lock_guard<std::recursive_mutex> lock(lifecycleMtx);
    try {
        if(!camRunning || !device) {
            throw std::runtime_error("Driver is not running.");
        }
        saveCalib();
        res->success = true;
    } catch(const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Saving calibration failed: %s", e.what());
        res->success = false;
        res->message = e.what();
    }
}

void Driver::savePipeline() {
    std::stringstream savePath;
    savePath << "/tmp/" << device->getDeviceId().c_str() << "_pipeline.json";
    RCLCPP_INFO(get_logger(), "Saving pipeline schema to: %s", savePath.str().c_str());
    std::ofstream file(savePath.str());
    file << pipeline->serializeToJson()["pipeline"];
    file.close();
}

void Driver::savePipelineCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res) {
    std::lock_guard<std::recursive_mutex> lock(lifecycleMtx);
    try {
        if(!camRunning || !device || !pipeline) {
            throw std::runtime_error("Driver is not running.");
        }
        savePipeline();
        res->success = true;
    } catch(const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Saving pipeline failed: %s", e.what());
        res->success = false;
        res->message = e.what();
    }
}

void Driver::startCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res) {
    std::lock_guard<std::recursive_mutex> lock(lifecycleMtx);
    try {
        start();
        res->success = camRunning;
        if(camRunning) {
            res->message = "Driver started.";
        } else {
            res->message = managedLifecycle->lastError();
            if(res->message.empty()) res->message = "Driver did not start; check the logs for details.";
        }
    } catch(const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Starting driver failed: %s", e.what());
        res->success = false;
        res->message = e.what();
    }
}
void Driver::stopCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res) {
    try {
        stop();
        res->success = true;
        res->message = "Driver stopped.";
    } catch(const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Stopping driver failed: %s", e.what());
        res->success = false;
        res->message = e.what();
    }
}
bool Driver::getDeviceType() {
    if(!startDevice()) {
        return false;
    }
    platform = device->getPlatform();
    std::string boardID;
    try {
        boardID = device->readCalibrationOrDefault().getEepromData().boardName;
    } catch(const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Could not read the device EEPROM board name: %s. Falling back to detected camera capabilities.", e.what());
    }
    if(boardID.empty()) {
        RCLCPP_WARN(get_logger(), "Device EEPROM has no board name; using detected camera capabilities.");
    } else {
        RCLCPP_INFO(get_logger(), "Board ID: %s", boardID.c_str());
    }
    pipeline = std::make_shared<dai::Pipeline>(device);
    std::string reportedDeviceName;
    try {
        reportedDeviceName = device->getDeviceName();
    } catch(const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Could not read the device model name: %s. Falling back to the board name and camera capabilities.", e.what());
    }
    deviceName = depthai_bridge::resolveDeviceModelName(reportedDeviceName, boardID, device->getConnectedCameraFeatures().size());
    RCLCPP_INFO(get_logger(), "Device type: %s", deviceName.c_str());
    for(auto& sensor : device->getCameraSensorNames()) {
        RCLCPP_DEBUG(get_logger(), "Socket %d - %s", static_cast<int>(sensor.first), sensor.second.c_str());
    }
    // not working on OAK4 right now
    if(platform == dai::Platform::RVC2) {
        try {
            const auto irDrivers = device->getIrDrivers();
            if(irDrivers.empty()) {
                RCLCPP_INFO(get_logger(), "Device has no IR drivers; IR controls will remain disabled.");
            } else {
                RCLCPP_DEBUG(get_logger(), "IR Drivers present");
            }
        } catch(const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Could not query IR capabilities: %s. IR controls will remain disabled.", e.what());
        }
    }
    return true;
}

void Driver::configureTransportDefaults() {
    const TransportDefaultsScope updatingDefaults(this);
    const auto requestedProfile = utils::getUpperCaseStr(ph->getParam<std::string>("i_transport_profile"));
    bool lowBandwidth = false;
    if(requestedProfile == "AUTO") {
        lowBandwidth = constrainedTransport;
    } else if(requestedProfile == "LOW_BANDWIDTH" || requestedProfile == "LOW-BANDWIDTH" || requestedProfile == "COMPRESSED") {
        lowBandwidth = true;
    } else if(requestedProfile != "RAW") {
        throw std::invalid_argument("Invalid driver.i_transport_profile value '" + requestedProfile + "'. Use AUTO, RAW, or LOW_BANDWIDTH.");
    }

    // Declare shared stream parameters early so the transport-aware default is
    // visible to all built-in pipelines. Explicit YAML/CLI overrides still win.
    // RVC2 cannot encode StereoDepth RAW8 disparity directly. Thermal's YUV422
    // and FP16 outputs and ToF's RAW16 depth also need to remain raw.
    static const std::vector<std::string> streamNames = {"rgb", "color", "left", "right", "stereo", "depth", "infra1", "infra2", "tof"};
    const auto& parameterOverrides = get_node_parameters_interface()->get_parameter_overrides();
    for(const auto& streamName : streamNames) {
        const auto parameterName = streamName + ".i_low_bandwidth";
        const bool rawDepth = streamName == "tof" || (platform == dai::Platform::RVC2 && (streamName == "stereo" || streamName == "depth"));
        const bool streamLowBandwidth = lowBandwidth && !rawDepth;
        if(!has_parameter(parameterName)) {
            declare_parameter<bool>(parameterName, streamLowBandwidth);
            if(parameterOverrides.count(parameterName) == 0) {
                std::lock_guard<std::mutex> lock(transportParamsMtx);
                transportManagedParams.insert(parameterName);
            }
        } else {
            bool managed;
            {
                std::lock_guard<std::mutex> lock(transportParamsMtx);
                managed = transportManagedParams.count(parameterName) != 0;
            }
            if(managed) {
                const auto result = set_parameter(rclcpp::Parameter(parameterName, streamLowBandwidth));
                if(!result.successful) {
                    throw std::runtime_error("Could not apply transport default for " + parameterName + ": " + result.reason);
                }
            }
        }
    }

    if(lowBandwidth) {
        RCLCPP_INFO(get_logger(),
                    "Image transport profile: LOW_BANDWIDTH%s. Device-side encoding is the default for compatible image streams; explicit per-stream "
                    "overrides still apply.",
                    requestedProfile == "AUTO" ? " (selected automatically for PoE/USB2)" : "");
        if(platform == dai::Platform::RVC2) {
            RCLCPP_INFO(get_logger(), "RVC2 stereo depth remains raw because its disparity format is not supported by the video encoder.");
        } else {
            RCLCPP_INFO(get_logger(), "Encoded stereo uses integer disparity; override stereo.i_low_bandwidth:=false for subpixel depth.");
        }
    } else {
        RCLCPP_INFO(get_logger(), "Image transport profile: RAW%s.", requestedProfile == "AUTO" ? " (selected automatically for USB3)" : "");
        if(constrainedTransport) {
            RCLCPP_WARN(get_logger(),
                        "RAW image transport was selected on PoE/USB2 and may reduce frame rate. Use driver.i_transport_profile:=LOW_BANDWIDTH for full-rate "
                        "streaming.");
        }
    }
}

void Driver::createPipeline() {
    generator = std::make_unique<pipeline_gen::PipelineGenerator>();
    const auto autoCalibrationMode = ph->getPipelineAutoCalibrationMode();
    if(!autoCalibrationMode.empty()) {
        pipeline->setAutoCalibrationMode(utils::parsePipelineAutoCalibrationMode(autoCalibrationMode));
    }
    if(!ph->getParam<std::string>("i_external_calibration_path").empty()) {
        loadCalib(ph->getParam<std::string>("i_external_calibration_path"));
    }
    generator->createPipeline(getNodeHandle(), device, pipeline, ph->getParam<bool>("i_rs_compat"));
    if(ph->getParam<bool>("i_pipeline_dump")) {
        savePipeline();
    }
    if(ph->getParam<bool>("i_calibration_dump")) {
        saveCalib();
    }
}

void Driver::setupQueues() {}

bool Driver::startDevice() {
    rclcpp::Rate r(1.0);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(ph->getParam<int>("i_connection_timeout"));
    while(rclcpp::ok(rclContext) && !shutdownRequested && !device) {
        if(std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("Device discovery/connection deadline exceeded");
        const auto deviceId = ph->getParam<std::string>("i_device_id");
        const auto ip = ph->getParam<std::string>("i_ip");
        const auto usbId = ph->getParam<std::string>("i_usb_port_id");
        try {
            if(deviceId.empty() && ip.empty() && usbId.empty()) {
                RCLCPP_INFO(get_logger(), "No ip/ID specified, connecting to the next available device.");
                const auto [found, info] = dai::Device::getAnyAvailableDevice();
                if(!found) {
                    throw std::runtime_error("No available devices detected.");
                }
                device = std::make_shared<dai::Device>(info, ph->getUSBSpeed());
            } else {
                auto availableDevices = dai::Device::getAllConnectedDevices();
                if(availableDevices.empty()) {
                    // autodiscovery might not work so try connecting via IP directly if set
                    if(!ip.empty()) {
                        dai::DeviceInfo info(ip);
                        RCLCPP_INFO(this->get_logger(), "No devices detected by autodiscovery, trying to connect to device via IP: %s", ip.c_str());
                        availableDevices.push_back(info);
                    } else {
                        throw std::runtime_error("No devices detected!");
                    }
                }
                const auto speed = ph->getUSBSpeed();
                bool matchingDeviceFound = false;
                for(const auto& info : availableDevices) {
                    if(!deviceId.empty() && info.getDeviceId() == deviceId) {
                        matchingDeviceFound = true;
                        RCLCPP_INFO(get_logger(), "Connecting to the device using ID: %s", deviceId.c_str());
                        if(isConnectableState(info.state)) {
                            device = std::make_shared<dai::Device>(info, speed);
                        } else if(isBootedState(info.state)) {
                            throw std::runtime_error("Device is already booted in different process.");
                        } else {
                            throw std::runtime_error("Device is in an unsupported connection state.");
                        }
                    } else if(!ip.empty() && info.name == ip) {
                        matchingDeviceFound = true;
                        RCLCPP_INFO(get_logger(), "Connecting to the device using ip: %s", ip.c_str());
                        if(isConnectableState(info.state)) {
                            device = std::make_shared<dai::Device>(info);
                        } else if(isBootedState(info.state)) {
                            throw std::runtime_error("Device is already booted in different process.");
                        } else {
                            throw std::runtime_error("Device is in an unsupported connection state.");
                        }
                    } else if(!usbId.empty() && info.name == usbId) {
                        matchingDeviceFound = true;
                        RCLCPP_INFO(get_logger(), "Connecting to the device using USB ID: %s", usbId.c_str());
                        if(isConnectableState(info.state)) {
                            device = std::make_shared<dai::Device>(info, speed);
                        } else if(isBootedState(info.state)) {
                            throw std::runtime_error("Device is already booted in different process.");
                        } else {
                            throw std::runtime_error("Device is in an unsupported connection state.");
                        }
                    }
                    if(device) {
                        break;
                    }
                }
                if(!matchingDeviceFound) {
                    throw std::runtime_error("The requested device was not found.");
                }
            }
        } catch(const std::exception& e) {
            device.reset();
            RCLCPP_ERROR(get_logger(), "%s", e.what());
        }
        if(!device) {
            r.sleep();
        }
    }

    if(!device) {
        return false;
    }

    RCLCPP_INFO(get_logger(), "Driver with ID: %s and Name: %s connected!", device->getDeviceId().c_str(), device->getDeviceInfo().name.c_str());
    const auto protocol = device->getDeviceInfo().getXLinkDeviceDesc().protocol;
    constrainedTransport = protocol == XLinkProtocol_t::X_LINK_TCP_IP;

    if(protocol != XLinkProtocol_t::X_LINK_TCP_IP) {
        const auto usbSpeed = device->getUsbSpeed();
        const auto speedIndex = static_cast<int32_t>(usbSpeed);
        const auto speed = speedIndex >= 0 && static_cast<size_t>(speedIndex) < usbStrings.size() ? usbStrings[speedIndex] : "UNKNOWN";
        RCLCPP_INFO(get_logger(), "USB SPEED: %s", speed.c_str());
        if(usbSpeed == dai::UsbSpeed::LOW || usbSpeed == dai::UsbSpeed::FULL || usbSpeed == dai::UsbSpeed::HIGH) {
            constrainedTransport = true;
            RCLCPP_INFO(get_logger(), "USB2 device detected; AUTO transport will use low-bandwidth image streams.");
        }
    } else {
        RCLCPP_INFO(get_logger(), "PoE device detected; AUTO transport will use low-bandwidth image streams.");
    }
    return true;
}

void Driver::setIR() {
    bool hasIR = true;
    if(platform == dai::Platform::RVC2) {
        try {
            hasIR = !device->getIrDrivers().empty();
        } catch(const std::exception& e) {
            hasIR = false;
            RCLCPP_WARN(get_logger(), "Could not query IR capabilities: %s. IR controls will remain disabled.", e.what());
        }
    }
    if(ph->getParam<bool>("i_enable_ir") && hasIR) {
        float laserdotIntensity = ph->getParam<float>("r_laser_dot_intensity");
        float floodlightIntensity = ph->getParam<float>("r_floodlight_intensity");
        device->setIrLaserDotProjectorIntensity(laserdotIntensity);
        device->setIrFloodLightIntensity(floodlightIntensity);
    }
}

rcl_interfaces::msg::SetParametersResult Driver::parameterCB(const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult res;
    // ROS holds its parameter mutex here. Waiting for lifecycleMtx could
    // deadlock against startup/teardown, which also reads ROS parameters.
    std::unique_lock<std::recursive_mutex> lock(lifecycleMtx, std::try_to_lock);
    if(!lock.owns_lock()) {
        res.successful = false;
        res.reason = "Driver lifecycle transition in progress; retry the parameter update.";
        return res;
    }
    res.successful = true;
    try {
        for(const auto& p : params) {
            if(camRunning && (p.get_name().find(".i_") != std::string::npos || p.get_name().rfind("diagnostics.", 0) == 0) && transportDefaultsOwner != this) {
                throw std::invalid_argument(p.get_name() + " changes require an inactive driver; deactivate first");
            }
            if(p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE && !std::isfinite(p.as_double())) {
                throw std::invalid_argument(p.get_name() + " must be finite");
            }
        }
        // Build/validate all control messages before ROS commits anything. No SDK writes here.
        if(generator && camRunning) generator->validateParams(params);
    } catch(const std::exception& e) {
        res.successful = false;
        res.reason = e.what();
    }
    return res;
}

void Driver::parametersAppliedCB(const std::vector<rclcpp::Parameter>& params) {
    if(transportDefaultsOwner == this) {
        return;
    }
    // Track only committed changes, including an explicit value equal to the
    // current default. Do not take lifecycleMtx while ROS holds its parameter
    // mutex: startup may already be waiting for that mutex.
    {
        std::lock_guard<std::mutex> lock(transportParamsMtx);
        for(const auto& param : params) transportManagedParams.erase(param.get_name());
    }
    std::lock_guard<std::mutex> lock(pendingParamsMtx);
    for(const auto& param : params) {
        const bool runtimeParam = param.get_name().find(".r_") != std::string::npos;
        if(param.get_name().find(".i_") != std::string::npos || param.get_name().rfind("diagnostics.", 0) == 0) configurationDirty = true;
        // A configured-but-inactive pipeline captured runtime values as its initial controls;
        // rebuild on activation so changes made while inactive are not silently dropped.
        if(!camRunning && runtimeParam) configurationDirty = true;
        if(camRunning && runtimeParam) {
            // Retain only the latest committed value; bound pending work by parameter count.
            auto it = std::find_if(pendingParams.begin(), pendingParams.end(), [&](const auto& p) { return p.get_name() == param.get_name(); });
            if(it == pendingParams.end())
                pendingParams.push_back(param);
            else
                *it = param;
        }
    }
}

void Driver::publishStatus() {
    std::lock_guard<std::recursive_mutex> lock(lifecycleMtx);
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = std::string(get_name()) + ": driver";
    status.hardware_id = get_fully_qualified_name();
    const bool connected = camRunning && pipeline && pipeline->isRunning();
    status.level = parameterApplyError.empty() && (!camRunning || connected) ? status.OK : status.ERROR;
    status.message = !parameterApplyError.empty() ? "Hardware parameter apply failed: " + parameterApplyError
                     : connected                  ? "Streaming"
                     : camRunning                 ? "Pipeline stopped unexpectedly"
                                                  : "Not streaming";
    auto add = [&](const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue entry;
        entry.key = key;
        entry.value = value;
        status.values.push_back(entry);
    };
    add("lifecycle_state_id", std::to_string(managedLifecycle->state()));
    add("connected", connected ? "true" : "false");
    add("restarts", std::to_string(restartCount));
    add("restart_failures", std::to_string(restartFailures));
    message.status.push_back(status);
    statusPublisher->publish(message);
    if(camRunning && !connected && ph && ph->getParam<bool>("i_restart_on_diagnostics_error")) restart();
}

void Driver::applyPendingParameters() {
    std::lock_guard<std::recursive_mutex> lock(lifecycleMtx);
    std::vector<rclcpp::Parameter> params;
    {
        std::lock_guard<std::mutex> pendingLock(pendingParamsMtx);
        params.swap(pendingParams);
    }
    if(params.empty() || !camRunning || !device || !generator) return;
    try {
        generator->validateParams(params);
        if(std::any_of(params.begin(), params.end(), [](const auto& p) {
               return p.get_name() == "driver.r_laser_dot_intensity" || p.get_name() == "driver.r_floodlight_intensity";
           }))
            setIR();
        generator->updateParams(params);
        parameterApplyError.clear();
    } catch(const std::exception& error) {
        parameterApplyError = error.what();
        RCLCPP_ERROR(get_logger(), "Committed parameter values could not be applied to hardware; deactivating: %s", error.what());
        stop();
    }
}

}  // namespace depthai_ros_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ros_driver::Driver);
