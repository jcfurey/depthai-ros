#include "depthai_filters/features_3d.hpp"
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    {
        depthai_filters::Features3D filter{rclcpp::NodeOptions()};
    }
    rclcpp::shutdown();
}
