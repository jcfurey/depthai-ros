#pragma once

#include <string>

#include "rclcpp/qos.hpp"

// message_filters ships .hpp headers from jazzy on and drops the .h
// compatibility headers in lyrical; humble has only the .h variants.
#if __has_include("message_filters/subscriber.hpp")
    #include "message_filters/subscriber.hpp"
    #include "message_filters/sync_policies/approximate_time.hpp"
    #include "message_filters/synchronizer.hpp"
#else
    #include "message_filters/subscriber.h"
    #include "message_filters/sync_policies/approximate_time.h"
    #include "message_filters/synchronizer.h"
#endif

namespace depthai_filters {
// message_filters >= 6 (lyrical) drops the Node* + defaulted-rmw-profile subscribe
// overload for NodeInterfaces + a mandatory rclcpp::QoS. QoS(10) matches the old
// default profile. DEPTHAI_ROS_MF_NEEDS_QOS is set from message_filters_VERSION
// in CMake.
template <typename Sub, typename NodeT>
inline void subscribeCompat(Sub& sub, NodeT* node, const std::string& topic) {
#ifdef DEPTHAI_ROS_MF_NEEDS_QOS
    sub.subscribe(*node, topic, rclcpp::QoS(10));
#else
    sub.subscribe(node, topic);
#endif
}
}  // namespace depthai_filters
