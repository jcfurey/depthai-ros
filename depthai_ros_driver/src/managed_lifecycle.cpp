#include "depthai_ros_driver/managed_lifecycle.hpp"

#include <stdexcept>

#include "rcl/error_handling.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace depthai_ros_driver {
namespace {
using Transition = lifecycle_msgs::msg::Transition;
using ChangeState = lifecycle_msgs::srv::ChangeState;
using GetState = lifecycle_msgs::srv::GetState;
using GetStates = lifecycle_msgs::srv::GetAvailableStates;
using GetTransitions = lifecycle_msgs::srv::GetAvailableTransitions;
}  // namespace
ManagedLifecycle::ManagedLifecycle(rclcpp::Node& node, std::recursive_mutex& mutex, Action action)
    : node(node), mutex(mutex), action(std::move(action)), machine(rcl_lifecycle_get_zero_initialized_state_machine()) {
    auto options = rcl_lifecycle_get_default_state_machine_options();
    options.enable_com_interface = false;  // rclcpp owns the services/publisher below.
    const auto result = rcl_lifecycle_state_machine_init(&machine,
                                                         node.get_node_base_interface()->get_rcl_node_handle(),
                                                         node.get_clock()->get_clock_handle(),
                                                         rosidl_typesupport_cpp::get_message_type_support_handle<lifecycle_msgs::msg::TransitionEvent>(),
                                                         rosidl_typesupport_cpp::get_service_type_support_handle<ChangeState>(),
                                                         rosidl_typesupport_cpp::get_service_type_support_handle<GetState>(),
                                                         rosidl_typesupport_cpp::get_service_type_support_handle<GetStates>(),
                                                         rosidl_typesupport_cpp::get_service_type_support_handle<GetTransitions>(),
                                                         rosidl_typesupport_cpp::get_service_type_support_handle<GetTransitions>(),
                                                         &options);
    if(result != RCL_RET_OK) {
        const std::string error = rcl_get_error_string().str;
        rcl_reset_error();
        throw std::runtime_error("Cannot initialize driver lifecycle: " + error);
    }
    try {
        rclcpp::PublisherOptions eventOptions;
        eventOptions.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
        events = node.create_publisher<lifecycle_msgs::msg::TransitionEvent>("~/transition_event", rclcpp::QoS(10).transient_local(), eventOptions);
        stateService = node.create_service<GetState>("~/get_state", [this](const GetState::Request::SharedPtr, GetState::Response::SharedPtr response) {
            std::lock_guard<std::recursive_mutex> lock(this->mutex);
            response->current_state = stateMessage(*machine.current_state);
        });
        statesService =
            node.create_service<GetStates>("~/get_available_states", [this](const GetStates::Request::SharedPtr, GetStates::Response::SharedPtr response) {
                std::lock_guard<std::recursive_mutex> lock(this->mutex);
                for(unsigned int i = 0; i < machine.transition_map.states_size; ++i)
                    response->available_states.push_back(stateMessage(machine.transition_map.states[i]));
            });
        transitionsService = node.create_service<GetTransitions>(
            "~/get_available_transitions", [this](const GetTransitions::Request::SharedPtr, GetTransitions::Response::SharedPtr response) {
                std::lock_guard<std::recursive_mutex> lock(this->mutex);
                for(unsigned int i = 0; i < machine.current_state->valid_transition_size; ++i)
                    response->available_transitions.push_back(transitionMessage(machine.current_state->valid_transitions[i]));
            });
        graphService = node.create_service<GetTransitions>(
            "~/get_transition_graph", [this](const GetTransitions::Request::SharedPtr, GetTransitions::Response::SharedPtr response) {
                std::lock_guard<std::recursive_mutex> lock(this->mutex);
                for(unsigned int i = 0; i < machine.transition_map.transitions_size; ++i)
                    response->available_transitions.push_back(transitionMessage(machine.transition_map.transitions[i]));
            });
        changeService = node.create_service<ChangeState>("~/change_state",
                                                         [this](const ChangeState::Request::SharedPtr request, ChangeState::Response::SharedPtr response) {
                                                             std::lock_guard<std::recursive_mutex> lock(this->mutex);
                                                             uint8_t id = request->transition.id;
                                                             if(id == 0 && !request->transition.label.empty()) {
                                                                 for(unsigned int i = 0; i < machine.current_state->valid_transition_size; ++i) {
                                                                     const auto& candidate = machine.current_state->valid_transitions[i];
                                                                     if(request->transition.label == candidate.label) id = candidate.id;
                                                                 }
                                                             }
                                                             response->success = change(id);
                                                         });
    } catch(...) {
        if(rcl_lifecycle_state_machine_fini(&machine, node.get_node_base_interface()->get_rcl_node_handle()) != RCL_RET_OK) rcl_reset_error();
        throw;
    }
}
ManagedLifecycle::~ManagedLifecycle() {
    if(rcl_lifecycle_state_machine_fini(&machine, node.get_node_base_interface()->get_rcl_node_handle()) != RCL_RET_OK) rcl_reset_error();
}
lifecycle_msgs::msg::State ManagedLifecycle::stateMessage(const rcl_lifecycle_state_t& state) const {
    lifecycle_msgs::msg::State result;
    result.id = state.id;
    result.label = state.label;
    return result;
}
lifecycle_msgs::msg::TransitionDescription ManagedLifecycle::transitionMessage(const rcl_lifecycle_transition_t& transition) const {
    lifecycle_msgs::msg::TransitionDescription result;
    result.transition.id = transition.id;
    result.transition.label = transition.label;
    result.start_state = stateMessage(*transition.start);
    result.goal_state = stateMessage(*transition.goal);
    return result;
}
uint8_t ManagedLifecycle::state() const {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    return machine.current_state->id;
}
std::string ManagedLifecycle::lastError() const {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    return lastTransitionError;
}
void ManagedLifecycle::trigger(uint8_t id) {
    lifecycle_msgs::msg::TransitionEvent event;
    event.stamp = node.now();
    event.start_state = stateMessage(*machine.current_state);
    for(unsigned int i = 0; i < machine.current_state->valid_transition_size; ++i) {
        const auto& candidate = machine.current_state->valid_transitions[i];
        if(candidate.id == id) event.transition = transitionMessage(candidate).transition;
    }
    if(rcl_lifecycle_trigger_transition_by_id(&machine, id, false) != RCL_RET_OK) {
        const std::string error = rcl_get_error_string().str;
        rcl_reset_error();
        throw std::runtime_error(error);
    }
    event.goal_state = stateMessage(*machine.current_state);
    if(node.get_node_base_interface()->get_context()->is_valid()) events->publish(event);
}
bool ManagedLifecycle::change(uint8_t id) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    if(id < Transition::TRANSITION_CONFIGURE || id > Transition::TRANSITION_ACTIVE_SHUTDOWN) return false;
    bool allowed = false;
    for(unsigned int i = 0; i < machine.current_state->valid_transition_size; ++i) allowed |= machine.current_state->valid_transitions[i].id == id;
    if(!allowed) {
        lastTransitionError = "Transition is not allowed from state " + std::string(machine.current_state->label);
        return false;
    }
    lastTransitionError.clear();
    trigger(id);
    const uint8_t completion = id <= 4 ? id * 10 : Transition::TRANSITION_ON_SHUTDOWN_SUCCESS;
    try {
        const bool success = action(id);
        if(!success) lastTransitionError = "Lifecycle callback rejected the transition";
        trigger(completion + (success ? 0 : 1));
        return success;
    } catch(const std::exception& error) {
        lastTransitionError = error.what();
        RCLCPP_ERROR(node.get_logger(), "Lifecycle transition failed: %s", error.what());
    } catch(...) {
        lastTransitionError = "Lifecycle transition failed with an unknown exception";
        RCLCPP_ERROR(node.get_logger(), "%s", lastTransitionError.c_str());
    }
    trigger(completion + 2);
    try {
        const bool recovered = action(Transition::TRANSITION_ON_ERROR_SUCCESS);
        trigger(recovered ? Transition::TRANSITION_ON_ERROR_SUCCESS : Transition::TRANSITION_ON_ERROR_FAILURE);
    } catch(...) {
        trigger(Transition::TRANSITION_ON_ERROR_ERROR);
    }
    return false;
}
}  // namespace depthai_ros_driver
