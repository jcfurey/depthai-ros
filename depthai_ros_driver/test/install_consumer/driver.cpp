#include "depthai_ros_driver/driver.hpp"
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    {
        depthai_ros_driver::Driver driver(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("driver.i_autostart", false)}));
    }
    rclcpp::shutdown();
}
