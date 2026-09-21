#include "depthai_ros_driver/driver.hpp"

#include <cstdlib>
#include <fstream>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_ros_driver/pipeline/pipeline_generator.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/version.h"

namespace depthai_ros_driver {
namespace {

bool isConnectableState(XLinkDeviceState_t state) {
    return state == X_LINK_ANY_STATE || state == X_LINK_UNBOOTED || state == X_LINK_BOOTLOADER || state == X_LINK_FLASH_BOOTED || state == X_LINK_GATE
           || state == X_LINK_GATE_SETUP;
}

bool isBootedState(XLinkDeviceState_t state) {
    return state == X_LINK_BOOTED || state == X_LINK_BOOTED_NON_EXCLUSIVE || state == X_LINK_GATE_BOOTED;
}

}  // namespace

Driver::Driver(const rclcpp::NodeOptions& options) : rclcpp::Node("oak", options) {
    //  Since we cannot use shared_from this before the object is initialized, we need to use a timer to start the device.
    // Close DepthAI queues while ROS publishers and logging are still valid. Setting
    // shutdownRequested first also lets a pending device-discovery loop unwind.
    rclContext = options.context();
    preShutdownCBHandle = rclContext->add_pre_shutdown_callback([this]() {
        shutdownRequested = true;
        try {
            stop();
        } catch(const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to stop driver during shutdown: %s", e.what());
        }
    });
    startTimer = this->create_wall_timer(std::chrono::seconds(1), [this]() {
        // Prevent starting multiple times when not using a static executor.
        if(!starting.exchange(true)) {
            try {
                start();
            } catch(const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Driver startup failed: %s", e.what());
                starting = false;
                return;
            }
            if(!camRunning) {
                starting = false;
                return;
            }
            srvGroup = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

            paramCBHandle = this->add_on_set_parameters_callback(std::bind(&Driver::parameterCB, this, std::placeholders::_1));
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
            stopSrv = this->create_service<Trigger>("~/stop_driver",
                                                    std::bind(&Driver::stopCB, this, std::placeholders::_1, std::placeholders::_2),
                                                    rclcpp::ServicesQoS().get_rmw_qos_profile(),
                                                    srvGroup);
            startAliasSrv = this->create_service<Trigger>("~/start",
                                                          std::bind(&Driver::startCB, this, std::placeholders::_1, std::placeholders::_2),
                                                          rclcpp::ServicesQoS().get_rmw_qos_profile(),
                                                          srvGroup);
            stopAliasSrv = this->create_service<Trigger>("~/stop",
                                                         std::bind(&Driver::stopCB, this, std::placeholders::_1, std::placeholders::_2),
                                                         rclcpp::ServicesQoS().get_rmw_qos_profile(),
                                                         srvGroup);
            savePipelineSrv = this->create_service<Trigger>("~/save_pipeline",
                                                            std::bind(&Driver::savePipelineCB, this, std::placeholders::_1, std::placeholders::_2),
                                                            rclcpp::ServicesQoS().get_rmw_qos_profile(),
                                                            srvGroup);
            saveCalibSrv = this->create_service<Trigger>("~/save_calibration",
                                                         std::bind(&Driver::saveCalibCB, this, std::placeholders::_1, std::placeholders::_2),
                                                         rclcpp::ServicesQoS().get_rmw_qos_profile(),
                                                         srvGroup);
#endif

            diagSub =
                this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10, std::bind(&Driver::diagCB, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "Driver ready!");
            startTimer->cancel();
        }
    });
}

Driver::~Driver() {
    if(rclContext) {
        rclContext->remove_pre_shutdown_callback(preShutdownCBHandle);
    }
    shutdownRequested = true;
    stop();
}

void Driver::onConfigure() {
    ph = std::make_unique<param_handlers::DriverParamHandler>(shared_from_this(), "driver");
    ph->declareParams();
    if(!getDeviceType()) {
        RCLCPP_WARN(get_logger(), "Shutdown requested before a device was found, aborting startup.");
        return;
    }
    configureTransportDefaults();
    createPipeline();
    setupQueues();
    setIR();
    // If model name not set get one from the device
    std::string camModel = ph->getParam<std::string>("i_tf_device_model");
    if(camModel.empty()) {
        camModel = deviceName;
    }

    if(ph->getParam<bool>("i_publish_tf_from_calibration")) {
        try {
            tfPub = std::make_unique<depthai_bridge::TFPublisher>(shared_from_this(),
                                                                  device->readCalibration(),
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
    pipeline->start();
    camRunning = true;
    const auto* rosDistro = std::getenv("ROS_DISTRO");
    RCLCPP_INFO(get_logger(),
                "If you detect any issues with %s release, please report "
                "issues to GH: https://github.com/luxonis/depthai-ros/issues/719",
                rosDistro != nullptr ? rosDistro : "the current ROS");
}

void Driver::diagCB(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    for(const auto& status : msg->status) {
        if(status.name == get_name() + std::string(": sys_logger")) {
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
    std::lock_guard<std::mutex> lock(lifecycleMtx);
    startImpl();
}

void Driver::startImpl() {
    RCLCPP_INFO(this->get_logger(), "Starting driver.");
    if(camRunning) {
        RCLCPP_INFO(this->get_logger(), "Driver is already running.");
        return;
    }
    try {
        onConfigure();
    } catch(...) {
        stopImpl();
        throw;
    }
}

void Driver::stop() {
    std::lock_guard<std::mutex> lock(lifecycleMtx);
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
        } catch(const std::exception& e) {
            if(rclcpp::ok()) {
                RCLCPP_WARN(get_logger(), "Failed to stop the DepthAI pipeline: %s", e.what());
            }
        }
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
    std::lock_guard<std::mutex> lock(lifecycleMtx);
    RCLCPP_WARN(get_logger(), "Restarting driver");
    stopImpl();
    startImpl();
    if(!camRunning) {
        RCLCPP_ERROR(get_logger(), "Restarting driver failed.");
    }
}

void Driver::saveCalib() {
    auto calibHandler = device->readCalibration();
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
    try {
        start();
        res->success = camRunning;
        if(camRunning) {
            res->message = "Driver started.";
        } else {
            res->message = "Driver did not start; check the logs for details.";
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
    // Thermal outputs are YUV422 and FP16 temperature frames. Neither format
    // can be consumed directly by the RVC2 video encoder, so keep thermal
    // transport raw while allowing the other streams (notably RGB on OAK-T)
    // to use the constrained-link default.
    static const std::vector<std::string> streamNames = {"rgb", "color", "left", "right", "stereo", "depth", "infra1", "infra2", "tof"};
    const auto& parameterOverrides = get_node_parameters_interface()->get_parameter_overrides();
    for(const auto& streamName : streamNames) {
        const auto parameterName = streamName + ".i_low_bandwidth";
        if(!has_parameter(parameterName)) {
            declare_parameter<bool>(parameterName, lowBandwidth);
            if(parameterOverrides.count(parameterName) == 0) {
                transportManagedParams.insert(parameterName);
            }
        } else if(transportManagedParams.count(parameterName) != 0) {
            set_parameter(rclcpp::Parameter(parameterName, lowBandwidth));
        }
    }

    if(lowBandwidth) {
        RCLCPP_INFO(get_logger(),
                    "Image transport profile: LOW_BANDWIDTH%s. Device-side encoding is the default for published image streams; explicit per-stream "
                    "overrides still apply.",
                    requestedProfile == "AUTO" ? " (selected automatically for PoE/USB2)" : "");
        RCLCPP_INFO(get_logger(),
                    "When low-bandwidth encoding is applied to stereo, it uses integer disparity; select RAW or override stereo.i_low_bandwidth:=false if "
                    "subpixel depth is required.");
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
    generator->createPipeline(shared_from_this(), device, pipeline, ph->getParam<bool>("i_rs_compat"));
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
    while(rclcpp::ok() && !shutdownRequested && !device) {
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
    res.successful = true;
    if(!camRunning || !device || !generator) {
        RCLCPP_DEBUG(get_logger(), "Driver is not running; parameter changes will be applied on next start.");
        return res;
    }
    try {
        bool hasIR = true;
        if(platform == dai::Platform::RVC2) {
            try {
                hasIR = !device->getIrDrivers().empty();
            } catch(const std::exception& e) {
                hasIR = false;
                RCLCPP_WARN(get_logger(), "Could not query IR capabilities: %s. IR controls will remain disabled.", e.what());
            }
        }
        for(const auto& p : params) {
            if(ph->getParam<bool>("i_enable_ir") && hasIR) {
                if(p.get_name() == ph->getFullParamName("r_laser_dot_intensity")) {
                    const float laserdotIntensity = p.get_value<float>();
                    device->setIrLaserDotProjectorIntensity(laserdotIntensity);
                } else if(p.get_name() == ph->getFullParamName("r_floodlight_intensity")) {
                    const float floodlightIntensity = p.get_value<float>();
                    device->setIrFloodLightIntensity(floodlightIntensity);
                }
            }
        }
        generator->updateParams(params);
    } catch(const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Parameter update failed: %s", e.what());
        res.successful = false;
        res.reason = e.what();
    }
    return res;
}

}  // namespace depthai_ros_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ros_driver::Driver);
