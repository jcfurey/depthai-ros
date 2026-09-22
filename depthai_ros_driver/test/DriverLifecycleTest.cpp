#include <gtest/gtest.h>

#include <chrono>
#include <future>

#include "depthai/device/Platform.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "depthai_ros_driver/driver.hpp"

namespace depthai_ros_driver {

// Exercise the real parameter callbacks and lifecycle lock without spinning the
// startup timer or connecting a camera.
class DriverTestAccess {
   public:
    static void running(Driver& driver, bool value) {
        driver.camRunning = value;
    }
    static void diagnostic(Driver& driver, const diagnostic_msgs::msg::DiagnosticArray::SharedPtr& message) {
        driver.diagCB(message);
    }
    static uint64_t restarts(Driver& driver) {
        return driver.restartCount;
    }
    static size_t pending(Driver& driver) {
        std::lock_guard<std::mutex> lock(driver.pendingParamsMtx);
        return driver.pendingParams.size();
    }
    static void initialize(const std::shared_ptr<Driver>& driver) {
        driver->startTimer->cancel();
        driver->ph = std::make_unique<param_handlers::DriverParamHandler>(driver->getNodeHandle(), "driver");
        driver->ph->declareParams();
    }
    static void configure(Driver& driver, dai::Platform platform = dai::Platform::RVC2, bool constrained = true) {
        auto lock = lockLifecycle(driver);
        driver.platform = platform;
        driver.constrainedTransport = constrained;
        driver.configureTransportDefaults();
    }
    static std::unique_lock<std::recursive_mutex> lockLifecycle(Driver& driver) {
        return std::unique_lock<std::recursive_mutex>(driver.lifecycleMtx);
    }
    static bool save(Driver& driver, bool calibration) {
        auto response = std::make_shared<Trigger::Response>();
        if(calibration) {
            driver.saveCalibCB(nullptr, response);
        } else {
            driver.savePipelineCB(nullptr, response);
        }
        return response->success;
    }
};

class DriverLifecycleTest : public testing::Test {
   protected:
    void SetUp() override {
        context = std::make_shared<rclcpp::Context>();
        context->init(0, nullptr);
        driver = std::make_shared<Driver>(rclcpp::NodeOptions().context(context));
        DriverTestAccess::initialize(driver);
    }
    void TearDown() override {
        if(driver) {
            driver.reset();
        }
        if(context && context->is_valid()) {
            context->shutdown("test complete");
        }
    }
    bool lowBandwidth(const std::string& stream) {
        return driver->get_parameter(stream + ".i_low_bandwidth").as_bool();
    }
    void profile(const std::string& value) {
        ASSERT_TRUE(driver->set_parameter(rclcpp::Parameter("driver.i_transport_profile", value)).successful);
    }
    rclcpp::Context::SharedPtr context;
    std::shared_ptr<Driver> driver;
};

TEST_F(DriverLifecycleTest, DiagnosticsNeverAutoActivateInactiveDriver) {
    ASSERT_TRUE(driver->set_parameter(rclcpp::Parameter("driver.i_restart_on_diagnostics_error", true)).successful);
    auto message = std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "oak: sys_logger";
    status.hardware_id = "/oak_test_device";
    status.level = status.ERROR;
    message->status.push_back(status);
    DriverTestAccess::diagnostic(*driver, message);
    EXPECT_EQ(DriverTestAccess::restarts(*driver), 0u);
}

TEST_F(DriverLifecycleTest, ConstrainedRvc2KeepsDepthRawAndEncodesColor) {
    for(const auto& value : {"AUTO", "LOW_BANDWIDTH"}) {
        profile(value);
        DriverTestAccess::configure(*driver);
        EXPECT_TRUE(lowBandwidth("rgb"));
        EXPECT_TRUE(lowBandwidth("color"));
        EXPECT_FALSE(lowBandwidth("stereo"));
        EXPECT_FALSE(lowBandwidth("depth"));
        EXPECT_FALSE(lowBandwidth("tof"));
        EXPECT_FALSE(driver->has_parameter("thermal.i_low_bandwidth"));
    }
}

TEST_F(DriverLifecycleTest, AutoTracksTransportAndPlatformAcrossRestarts) {
    DriverTestAccess::configure(*driver, dai::Platform::RVC2, false);
    EXPECT_FALSE(lowBandwidth("rgb"));
    DriverTestAccess::configure(*driver, dai::Platform::RVC4, true);
    EXPECT_TRUE(lowBandwidth("rgb"));
    EXPECT_TRUE(lowBandwidth("stereo"));
    EXPECT_FALSE(lowBandwidth("tof"));
    DriverTestAccess::configure(*driver, dai::Platform::RVC2, true);
    EXPECT_FALSE(lowBandwidth("stereo"));
}

TEST_F(DriverLifecycleTest, RuntimeOverrideSurvivesRestartAndProfileChanges) {
    DriverTestAccess::configure(*driver);
    ASSERT_TRUE(driver->set_parameter(rclcpp::Parameter("rgb.i_low_bandwidth", false)).successful);
    DriverTestAccess::configure(*driver);
    EXPECT_FALSE(lowBandwidth("rgb"));
    EXPECT_TRUE(lowBandwidth("left"));
    profile("RAW");
    DriverTestAccess::configure(*driver);
    EXPECT_FALSE(lowBandwidth("left"));
    profile("LOW_BANDWIDTH");
    DriverTestAccess::configure(*driver);
    EXPECT_FALSE(lowBandwidth("rgb"));
    EXPECT_TRUE(lowBandwidth("left"));
}

TEST_F(DriverLifecycleTest, ExplicitValueEqualToDefaultAlsoSurvivesProfileChange) {
    DriverTestAccess::configure(*driver);
    ASSERT_TRUE(driver->set_parameter(rclcpp::Parameter("rgb.i_low_bandwidth", true)).successful);
    profile("RAW");
    DriverTestAccess::configure(*driver);
    EXPECT_TRUE(lowBandwidth("rgb"));
    EXPECT_FALSE(lowBandwidth("left"));
}

TEST_F(DriverLifecycleTest, StartupOverridesStillTakePrecedence) {
    auto options = rclcpp::NodeOptions().context(context);
    options.parameter_overrides({rclcpp::Parameter("rgb.i_low_bandwidth", false)});
    auto configuredDriver = std::make_shared<Driver>(options);
    DriverTestAccess::initialize(configuredDriver);
    DriverTestAccess::configure(*configuredDriver);
    EXPECT_FALSE(configuredDriver->get_parameter("rgb.i_low_bandwidth").as_bool());
    DriverTestAccess::configure(*configuredDriver);
    EXPECT_FALSE(configuredDriver->get_parameter("rgb.i_low_bandwidth").as_bool());
    EXPECT_TRUE(configuredDriver->get_parameter("left.i_low_bandwidth").as_bool());
}

TEST_F(DriverLifecycleTest, RejectedAtomicUpdateDoesNotPinTransportDefault) {
    DriverTestAccess::configure(*driver);
    driver->declare_parameter<bool>("reject_update", false);
    auto reject = driver->add_on_set_parameters_callback([](const auto& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for(const auto& param : params) {
            if(param.get_name() == "reject_update" && param.as_bool()) {
                result.successful = false;
            }
        }
        return result;
    });
    EXPECT_FALSE(driver->set_parameters_atomically({rclcpp::Parameter("rgb.i_low_bandwidth", false), rclcpp::Parameter("reject_update", true)}).successful);
    profile("RAW");
    DriverTestAccess::configure(*driver);
    EXPECT_FALSE(lowBandwidth("rgb"));
    profile("LOW_BANDWIDTH");
    DriverTestAccess::configure(*driver);
    EXPECT_TRUE(lowBandwidth("rgb"));
}

TEST_F(DriverLifecycleTest, ParameterUpdateDuringTransitionReturnsWithoutDeadlocking) {
    DriverTestAccess::configure(*driver);
    auto lock = DriverTestAccess::lockLifecycle(*driver);
    auto update = std::async(std::launch::async, [&]() { return driver->set_parameter(rclcpp::Parameter("rgb.i_low_bandwidth", false)); });
    const auto status = update.wait_for(std::chrono::seconds(2));
    // Always unlock before joining so a regression fails rather than hanging.
    lock.unlock();
    EXPECT_EQ(status, std::future_status::ready);
    const auto result = update.get();
    EXPECT_FALSE(result.successful);
    EXPECT_NE(result.reason.find("retry"), std::string::npos);
    EXPECT_TRUE(lowBandwidth("rgb"));
    EXPECT_TRUE(driver->set_parameter(rclcpp::Parameter("rgb.i_low_bandwidth", false)).successful);
}

TEST_F(DriverLifecycleTest, SaveRequestsWaitForLifecycleTransition) {
    for(bool calibration : {false, true}) {
        auto lock = DriverTestAccess::lockLifecycle(*driver);
        std::promise<void> started;
        auto ready = started.get_future();
        auto save = std::async(std::launch::async, [&]() {
            started.set_value();
            return DriverTestAccess::save(*driver, calibration);
        });
        ready.wait();
        EXPECT_EQ(save.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);
        lock.unlock();
        EXPECT_FALSE(save.get());
    }
}

TEST_F(DriverLifecycleTest, TopicSimulationFailsBeforeAccessingCameraNodes) {
    driver->declare_parameter<bool>("left.i_simulate_from_topic", true);
    driver->declare_parameter<bool>("left.i_disable_node", true);
    // No pipeline is needed: unsupported input must fail before creating nodes.
    EXPECT_THROW(dai_nodes::SensorWrapper("left", driver, nullptr, "OAK-D", false, dai::CameraBoardSocket::CAM_B), std::invalid_argument);
}

TEST_F(DriverLifecycleTest, ReleasingConfiguredDriverRunsDestructor) {
    std::weak_ptr<Driver> weak = driver;
    driver.reset();
    EXPECT_TRUE(weak.expired());
}
TEST_F(DriverLifecycleTest, InitializationSettingsRequireInactiveState) {
    DriverTestAccess::running(*driver, true);
    EXPECT_FALSE(driver->set_parameter(rclcpp::Parameter("driver.i_transport_profile", "RAW")).successful);
    DriverTestAccess::running(*driver, false);
    EXPECT_TRUE(driver->set_parameter(rclcpp::Parameter("driver.i_transport_profile", "RAW")).successful);
}

TEST_F(DriverLifecycleTest, OnlyCommittedRuntimeUpdatesAreQueued) {
    DriverTestAccess::running(*driver, true);
    EXPECT_FALSE(
        driver->set_parameters_atomically({rclcpp::Parameter("driver.r_laser_dot_intensity", 0.2), rclcpp::Parameter("driver.r_floodlight_intensity", 2.0)})
            .successful);
    EXPECT_EQ(DriverTestAccess::pending(*driver), 0u);
    EXPECT_TRUE(
        driver->set_parameters_atomically({rclcpp::Parameter("driver.r_laser_dot_intensity", 0.2), rclcpp::Parameter("driver.r_floodlight_intensity", 0.3)})
            .successful);
    EXPECT_EQ(DriverTestAccess::pending(*driver), 2u);
    EXPECT_TRUE(driver->set_parameter(rclcpp::Parameter("driver.r_laser_dot_intensity", 0.4)).successful);
    EXPECT_EQ(DriverTestAccess::pending(*driver), 2u);
    DriverTestAccess::running(*driver, false);
}

}  // namespace depthai_ros_driver
