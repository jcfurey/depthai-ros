#pragma once

#include <functional>
#include <mutex>
#include <string>

#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_available_states.hpp"
#include "lifecycle_msgs/srv/get_available_transitions.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rcl_lifecycle/rcl_lifecycle.h"
#include "rclcpp/rclcpp.hpp"

namespace depthai_ros_driver {
// Standard ROS lifecycle interface on the existing composable Node. Using the
// rcl state machine retains its canonical transitions without changing the
// rclcpp::Node API consumed by pipeline plugins and image_transport helpers.
class ManagedLifecycle {
   public:
    using Action = std::function<bool(uint8_t)>;
    ManagedLifecycle(rclcpp::Node& node, std::recursive_mutex& mutex, Action action);
    ~ManagedLifecycle();
    ManagedLifecycle(const ManagedLifecycle&) = delete;
    ManagedLifecycle& operator=(const ManagedLifecycle&) = delete;
    bool change(uint8_t transition);
    uint8_t state() const;
    std::string lastError() const;

   private:
    void trigger(uint8_t transition);
    lifecycle_msgs::msg::State stateMessage(const rcl_lifecycle_state_t& state) const;
    lifecycle_msgs::msg::TransitionDescription transitionMessage(const rcl_lifecycle_transition_t& transition) const;
    rclcpp::Node& node;
    std::recursive_mutex& mutex;
    Action action;
    std::string lastTransitionError;
    rcl_lifecycle_state_machine_t machine;
    rclcpp::Publisher<lifecycle_msgs::msg::TransitionEvent>::SharedPtr events;
    rclcpp::Service<lifecycle_msgs::srv::ChangeState>::SharedPtr changeService;
    rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr stateService;
    rclcpp::Service<lifecycle_msgs::srv::GetAvailableStates>::SharedPtr statesService;
    rclcpp::Service<lifecycle_msgs::srv::GetAvailableTransitions>::SharedPtr transitionsService, graphService;
};
}  // namespace depthai_ros_driver
