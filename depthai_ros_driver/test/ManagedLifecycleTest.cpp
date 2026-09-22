#include <gtest/gtest.h>

#include "depthai_ros_driver/managed_lifecycle.hpp"

namespace depthai_ros_driver {
using Transition = lifecycle_msgs::msg::Transition;
using State = lifecycle_msgs::msg::State;
class ManagedLifecycleTest : public testing::Test {
   protected:
    void SetUp() override {
        context = std::make_shared<rclcpp::Context>();
        context->init(0, nullptr);
        node = std::make_shared<rclcpp::Node>("managed_camera", "/robot", rclcpp::NodeOptions().context(context));
        lifecycle = std::make_unique<ManagedLifecycle>(*node, mutex, [this](uint8_t id) {
            actions.push_back(id);
            if(id == throwOn) throw std::runtime_error("injected device failure");
            return id != rejectOn;
        });
    }
    void TearDown() override {
        lifecycle.reset();
        node.reset();
        context->shutdown("test complete");
    }
    std::shared_ptr<rclcpp::Context> context;
    std::shared_ptr<rclcpp::Node> node;
    std::unique_ptr<ManagedLifecycle> lifecycle;
    std::recursive_mutex mutex;
    std::vector<uint8_t> actions;
    uint8_t throwOn = 0, rejectOn = 0;
};
TEST_F(ManagedLifecycleTest, StandardTransitionsAndInvalidRequests) {
    EXPECT_EQ(lifecycle->state(), State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_FALSE(lifecycle->change(Transition::TRANSITION_ACTIVATE));
    EXPECT_TRUE(lifecycle->change(Transition::TRANSITION_CONFIGURE));
    EXPECT_EQ(lifecycle->state(), State::PRIMARY_STATE_INACTIVE);
    EXPECT_TRUE(lifecycle->change(Transition::TRANSITION_ACTIVATE));
    EXPECT_EQ(lifecycle->state(), State::PRIMARY_STATE_ACTIVE);
    EXPECT_FALSE(lifecycle->change(Transition::TRANSITION_CLEANUP));
    EXPECT_TRUE(lifecycle->change(Transition::TRANSITION_DEACTIVATE));
    EXPECT_EQ(lifecycle->state(), State::PRIMARY_STATE_INACTIVE);
    EXPECT_TRUE(lifecycle->change(Transition::TRANSITION_CLEANUP));
    EXPECT_TRUE(lifecycle->change(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN));
    EXPECT_EQ(lifecycle->state(), State::PRIMARY_STATE_FINALIZED);
    EXPECT_FALSE(lifecycle->change(Transition::TRANSITION_CONFIGURE));
}
TEST_F(ManagedLifecycleTest, ErrorRecoveryAndRetry) {
    throwOn = Transition::TRANSITION_CONFIGURE;
    EXPECT_FALSE(lifecycle->change(Transition::TRANSITION_CONFIGURE));
    EXPECT_EQ(lifecycle->state(), State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(actions.back(), Transition::TRANSITION_ON_ERROR_SUCCESS);
    EXPECT_EQ(lifecycle->lastError(), "injected device failure");
    throwOn = 0;
    EXPECT_TRUE(lifecycle->change(Transition::TRANSITION_CONFIGURE));
    EXPECT_TRUE(lifecycle->lastError().empty());
    rejectOn = Transition::TRANSITION_ACTIVATE;
    EXPECT_FALSE(lifecycle->change(Transition::TRANSITION_ACTIVATE));
    EXPECT_EQ(lifecycle->lastError(), "Lifecycle callback rejected the transition");
    EXPECT_EQ(lifecycle->state(), State::PRIMARY_STATE_INACTIVE);
    rejectOn = 0;
    EXPECT_TRUE(lifecycle->change(Transition::TRANSITION_ACTIVATE));
}
TEST_F(ManagedLifecycleTest, NamespacedStandardServices) {
    auto client = node->create_client<lifecycle_msgs::srv::GetState>("/robot/managed_camera/get_state");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
    auto future = client->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    rclcpp::ExecutorOptions options;
    options.context = context;
    rclcpp::executors::SingleThreadedExecutor executor(options);
    executor.add_node(node);
    ASSERT_EQ(executor.spin_until_future_complete(future, std::chrono::seconds(2)), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(future.get()->current_state.id, State::PRIMARY_STATE_UNCONFIGURED);
}
}  // namespace depthai_ros_driver
