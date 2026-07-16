#include "depthai_ros_driver_v3/dai_nodes/sys_logger.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/SystemInformation.hpp"
#include "depthai/pipeline/node/SystemLogger.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
SysLogger::SysLogger(
    const std::string& daiNodeName, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<dai::Pipeline> pipeline, std::string deviceName, bool rsCompat)
    : BaseNode(daiNodeName, node, pipeline, deviceName, rsCompat) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    sysNode = pipeline->create<dai::node::SystemLogger>();
    setInOut(pipeline);
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
}
SysLogger::~SysLogger() = default;

void SysLogger::setNames() {
    loggerQName = getName() + "_queue";
}

void SysLogger::setInOut(std::shared_ptr<dai::Pipeline> /* pipeline */) {}

void SysLogger::setupQueues(std::shared_ptr<dai::Device> device) {
    loggerQ = sysNode->out.createOutputQueue(8, false);
    // Grace period so the first updater tick doesn't report ERROR before the 1 Hz logger produced a sample.
    {
        std::lock_guard<std::mutex> lock(sysInfoMtx);
        setupTime = std::chrono::steady_clock::now();
    }
    // Cache samples as they arrive; produceDiagnostics runs on the node's executor and must not block on the queue.
    loggerQ->addCallback([this](const std::shared_ptr<dai::ADatatype>& data) {
        if(auto sysInfo = std::dynamic_pointer_cast<dai::SystemInformation>(data)) {
            std::lock_guard<std::mutex> lock(sysInfoMtx);
            lastSysInfo = sysInfo;
            lastSysInfoTime = std::chrono::steady_clock::now();
        }
    });
    updater = std::make_shared<diagnostic_updater::Updater>(getROSNode());
    updater->setHardwareID(getROSNode()->get_fully_qualified_name() + std::string("_") + device->getDeviceId() + std::string("_") + device->getDeviceName());
    updater->add("sys_logger", std::bind(&SysLogger::produceDiagnostics, this, std::placeholders::_1));
}

void SysLogger::closeQueues() {
    if(loggerQ) {
        loggerQ->close();
    }
}

std::string SysLogger::sysInfoToString(const dai::SystemInformation& sysInfo) {
    std::stringstream ss;
    ss << "System Information: " << std::endl;
    ss << "  Leon CSS CPU Usage: " << sysInfo.leonCssCpuUsage.average * 100 << std::endl;
    ss << "  Leon MSS CPU Usage: " << sysInfo.leonMssCpuUsage.average * 100 << std::endl;
    ss << " Ddr Memory Usage: " << sysInfo.ddrMemoryUsage.used / (1024.0f * 1024.0f) << std::endl;
    ss << " Ddr Memory Total: " << sysInfo.ddrMemoryUsage.total / (1024.0f * 1024.0f) << std::endl;
    ss << " Cmx Memory Usage: " << sysInfo.cmxMemoryUsage.used / (1024.0f * 1024.0f) << std::endl;
    ss << " Cmx Memory Total: " << sysInfo.cmxMemoryUsage.total << std::endl;
    ss << " Leon CSS Memory Usage: " << sysInfo.leonCssMemoryUsage.used / (1024.0f * 1024.0f) << std::endl;
    ss << " Leon CSS Memory Total: " << sysInfo.leonCssMemoryUsage.total / (1024.0f * 1024.0f) << std::endl;
    ss << " Leon MSS Memory Usage: " << sysInfo.leonMssMemoryUsage.used / (1024.0f * 1024.0f) << std::endl;
    ss << " Leon MSS Memory Total: " << sysInfo.leonMssMemoryUsage.total / (1024.0f * 1024.0f) << std::endl;
    ss << " Average Chip Temperature: " << sysInfo.chipTemperature.average << std::endl;
    ss << " Leon CSS Chip Temperature: " << sysInfo.chipTemperature.css << std::endl;
    ss << " Leon MSS Chip Temperature: " << sysInfo.chipTemperature.mss << std::endl;
    ss << " UPA Chip Temperature: " << sysInfo.chipTemperature.upa << std::endl;
    ss << " DSS Chip Temperature: " << sysInfo.chipTemperature.dss << std::endl;

    return ss.str();
}

void SysLogger::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    try {
        std::shared_ptr<dai::SystemInformation> logData;
        bool warmingUp = false;
        {
            std::lock_guard<std::mutex> lock(sysInfoMtx);
            constexpr auto staleAfter = std::chrono::seconds(5);
            auto now = std::chrono::steady_clock::now();
            if(lastSysInfo && (now - lastSysInfoTime) < staleAfter) {
                logData = lastSysInfo;
            } else if(!lastSysInfo && (now - setupTime) < staleAfter) {
                warmingUp = true;
            }
        }
        if(warmingUp) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for first sample");
            return;
        }
        if(logData) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "System Information");
            const dai::SystemInformation& sysInfo = *logData;
            stat.add("Leon CSS CPU Usage", sysInfo.leonCssCpuUsage.average * 100);
            stat.add("Leon MSS CPU Usage", sysInfo.leonMssCpuUsage.average * 100);
            stat.add("Ddr Memory Usage", sysInfo.ddrMemoryUsage.used / (1024.0f * 1024.0f));
            stat.add("Ddr Memory Total", sysInfo.ddrMemoryUsage.total / (1024.0f * 1024.0f));
            stat.add("Cmx Memory Usage", sysInfo.cmxMemoryUsage.used / (1024.0f * 1024.0f));
            stat.add("Cmx Memory Total", sysInfo.cmxMemoryUsage.total);
            stat.add("Leon CSS Memory Usage", sysInfo.leonCssMemoryUsage.used / (1024.0f * 1024.0f));
            stat.add("Leon CSS Memory Total", sysInfo.leonCssMemoryUsage.total / (1024.0f * 1024.0f));
            stat.add("Leon MSS Memory Usage", sysInfo.leonMssMemoryUsage.used / (1024.0f * 1024.0f));
            stat.add("Leon MSS Memory Total", sysInfo.leonMssMemoryUsage.total / (1024.0f * 1024.0f));
            stat.add("Average Chip Temperature", sysInfo.chipTemperature.average);
            stat.add("Leon CSS Chip Temperature", sysInfo.chipTemperature.css);
            stat.add("Leon MSS Chip Temperature", sysInfo.chipTemperature.mss);
            stat.add("UPA Chip Temperature", sysInfo.chipTemperature.upa);
            stat.add("DSS Chip Temperature", sysInfo.chipTemperature.dss);
        } else {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No Data");
        }
    } catch(const std::exception& e) {
        RCLCPP_ERROR(getROSNode()->get_logger(), "Producing diagnostics failed: %s", e.what());
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, e.what());
    }
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
