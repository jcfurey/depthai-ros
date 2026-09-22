#pragma once
#include <atomic>
#include <deque>
#include <mutex>
#include <rclcpp/clock.hpp>
#include <stdexcept>
#include <tf2_ros/transform_broadcaster.hpp>
#include <thread>
#include <type_traits>
#include <typeinfo>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace depthai_bridge {

namespace ImageMsgs = sensor_msgs::msg;
namespace FFMPEGMsgs = ffmpeg_image_transport_msgs::msg;
using ImagePtr = ImageMsgs::Image::SharedPtr;
using FFMPEGImagePtr = FFMPEGMsgs::FFMPEGPacket::SharedPtr;

template <class RosMsg, class DaiMsg>
class BridgePublisher {
   public:
    using ConvertFunc = std::function<void(std::shared_ptr<DaiMsg>, std::deque<RosMsg>&)>;
    using CustomPublisher = std::conditional_t<std::is_same_v<RosMsg, ImageMsgs::Image>,
                                               std::shared_ptr<image_transport::CameraPublisher>,
                                               std::conditional_t<std::is_same_v<RosMsg, geometry_msgs::msg::TransformStamped>,
                                                                  std::shared_ptr<tf2_ros::TransformBroadcaster>,
                                                                  typename rclcpp::Publisher<RosMsg>::SharedPtr>>;

    /**
     * @brief Constructor for BridgePublisher with default QoS settings.
     *
     * @param daiMessageQueue Shared pointer to the DepthAI message queue.
     * @param node Shared pointer to the ROS 2 node.
     * @param rosTopic The ROS topic to publish messages to.
     * @param converter Function to convert DepthAI messages to ROS messages.
     * @param qosSetting QoS settings for the ROS publisher.
     * @param lazyPublisher Flag to enable lazy publishing.
     */
    BridgePublisher(std::shared_ptr<dai::MessageQueue> daiMessageQueue,
                    std::shared_ptr<rclcpp::Node> node,
                    std::string rosTopic,
                    ConvertFunc converter,
                    rclcpp::QoS qosSetting = rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
                    bool lazyPublisher = true);

    /**
     * @brief Constructor for BridgePublisher with custom QoS history depth.
     *
     * @param daiMessageQueue Shared pointer to the DepthAI message queue.
     * @param node Shared pointer to the ROS 2 node.
     * @param rosTopic The ROS topic to publish messages to.
     * @param converter Function to convert DepthAI messages to ROS messages.
     * @param qosHistoryDepth History depth for the QoS settings.
     * @param cameraParamUri URI for the camera parameters.
     * @param cameraName Name of the camera.
     * @param lazyPublisher Flag to enable lazy publishing.
     */
    BridgePublisher(std::shared_ptr<dai::MessageQueue> daiMessageQueue,
                    std::shared_ptr<rclcpp::Node> node,
                    std::string rosTopic,
                    ConvertFunc converter,
                    size_t qosHistoryDepth,
                    std::string cameraParamUri = "",
                    std::string cameraName = "",
                    bool lazyPublisher = true);

    /**
     * @brief Constructor for BridgePublisher with custom camera info data.
     *
     * @param daiMessageQueue Shared pointer to the DepthAI message queue.
     * @param node Shared pointer to the ROS 2 node.
     * @param rosTopic The ROS topic to publish messages to.
     * @param converter Function to convert DepthAI messages to ROS messages.
     * @param qosHistoryDepth History depth for the QoS settings.
     * @param cameraInfoData Camera info data for the ROS messages.
     * @param cameraName Name of the camera.
     * @param lazyPublisher Flag to enable lazy publishing.
     */
    BridgePublisher(std::shared_ptr<dai::MessageQueue> daiMessageQueue,
                    std::shared_ptr<rclcpp::Node> node,
                    std::string rosTopic,
                    ConvertFunc converter,
                    size_t qosHistoryDepth,
                    ImageMsgs::CameraInfo cameraInfoData,
                    std::string cameraName,
                    bool lazyPublisher = true);

    /**
     * Tag Dispacher function to to overload the Publisher to ImageTransport Publisher
     */
    std::shared_ptr<image_transport::CameraPublisher> advertise(int queueSize, std::true_type);

    /**
     * Tag Dispacher function to to overload the Publisher to use Default ros::Publisher
     */
    typename rclcpp::Publisher<RosMsg>::SharedPtr advertise(int queueSize, std::false_type);

    BridgePublisher(const BridgePublisher&) = delete;
    BridgePublisher& operator=(const BridgePublisher&) = delete;
    // Stop and drain before destroying any state captured by the converter.
    // Do not call stop() or destroy this object from its own conversion callback.
    void stop();

    void addPublisherCallback();

    void enableTransformPub() {
        pubTransform = true;
        tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    }

    void publishHelper(std::shared_ptr<DaiMsg> inData);

    void startPublisherThread();

    ~BridgePublisher();

   private:
    /**
     * adding this callback will allow you to still be able to consume
     * the data for other processing using get() function .
     */
    void daiCallback(const std::string& name, std::shared_ptr<dai::ADatatype> data);

    void publishTransform(std::shared_ptr<tf2_ros::TransformBroadcaster> tfPub, const geometry_msgs::msg::TransformStamped& transform);
    static const std::string LOG_TAG;
    std::shared_ptr<dai::MessageQueue> daiMessageQueue;
    ConvertFunc converter;

    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoPublisher;

    ImageMsgs::CameraInfo cameraInfoData;
    CustomPublisher rosPublisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    bool pubTransform = false;

    struct CallbackState {
        std::mutex mutex;
        BridgePublisher* owner = nullptr;
    };
    std::shared_ptr<CallbackState> callbackState = std::make_shared<CallbackState>();
    std::mutex controlMutex;
    std::atomic<bool> stopRequested{false};
    int callbackId = -1;
    std::thread readingThread;
    std::string rosTopic, camInfoFrameId, cameraName, cameraParamUri;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camInfoManager;
    bool isCallbackAdded = false;
    bool isImageMessage = false;  // used to enable camera info manager
    bool lazyPublisher = true;
};

template <class RosMsg, class DaiMsg>
const std::string BridgePublisher<RosMsg, DaiMsg>::LOG_TAG = "BridgePublisher";

template <class RosMsg, class DaiMsg>
BridgePublisher<RosMsg, DaiMsg>::BridgePublisher(std::shared_ptr<dai::MessageQueue> daiMessageQueue,
                                                 std::shared_ptr<rclcpp::Node> node,
                                                 std::string rosTopic,
                                                 ConvertFunc converter,
                                                 rclcpp::QoS qosSetting,
                                                 bool lazyPublisher)
    : daiMessageQueue(daiMessageQueue), node(node), converter(converter), rosTopic(rosTopic), lazyPublisher(lazyPublisher) {
    if constexpr(std::is_same_v<RosMsg, geometry_msgs::msg::TransformStamped>) {
        this->lazyPublisher = false;
        rosPublisher = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    } else if constexpr(std::is_same_v<RosMsg, ImageMsgs::Image>) {
        camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(node.get(), "camera");
        rclcpp::PublisherOptions options;
        options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
        rosPublisher = std::make_shared<image_transport::CameraPublisher>(image_transport::create_camera_publisher(*node, rosTopic, qosSetting, options));
    } else {
        rclcpp::PublisherOptions options;
        options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
        rosPublisher = node->create_publisher<RosMsg>(rosTopic, qosSetting, options);
    }
}

template <class RosMsg, class DaiMsg>
BridgePublisher<RosMsg, DaiMsg>::BridgePublisher(std::shared_ptr<dai::MessageQueue> daiMessageQueue,
                                                 std::shared_ptr<rclcpp::Node> node,
                                                 std::string rosTopic,
                                                 ConvertFunc converter,
                                                 size_t qosHistoryDepth,
                                                 std::string cameraParamUri,
                                                 std::string cameraName,
                                                 bool lazyPublisher)
    : daiMessageQueue(daiMessageQueue),
      node(node),
      converter(converter),
      rosTopic(rosTopic),
      cameraParamUri(cameraParamUri),
      cameraName(cameraName),
      lazyPublisher(lazyPublisher) {
    if constexpr(std::is_same_v<RosMsg, geometry_msgs::msg::TransformStamped>) {
        this->lazyPublisher = false;
        rosPublisher = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    } else {
        rosPublisher = advertise(qosHistoryDepth, std::is_same<RosMsg, ImageMsgs::Image>{});
    }
}

template <class RosMsg, class DaiMsg>
BridgePublisher<RosMsg, DaiMsg>::BridgePublisher(std::shared_ptr<dai::MessageQueue> daiMessageQueue,
                                                 std::shared_ptr<rclcpp::Node> node,
                                                 std::string rosTopic,
                                                 ConvertFunc converter,
                                                 size_t qosHistoryDepth,
                                                 ImageMsgs::CameraInfo cameraInfoData,
                                                 std::string cameraName,
                                                 bool lazyPublisher)
    : daiMessageQueue(daiMessageQueue),
      node(node),
      converter(converter),
      rosTopic(rosTopic),
      cameraInfoData(cameraInfoData),
      cameraName(cameraName),
      lazyPublisher(lazyPublisher) {
    if constexpr(std::is_same_v<RosMsg, geometry_msgs::msg::TransformStamped>) {
        this->lazyPublisher = false;
        rosPublisher = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    } else {
        rosPublisher = advertise(qosHistoryDepth, std::is_same<RosMsg, ImageMsgs::Image>{});
    }
}

template <class RosMsg, class DaiMsg>
typename rclcpp::Publisher<RosMsg>::SharedPtr BridgePublisher<RosMsg, DaiMsg>::advertise(int queueSize, std::false_type) {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    if(!cameraName.empty()) {
        isImageMessage = true;
        camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(node.get(), cameraName, cameraParamUri);
        if(cameraParamUri.empty()) {
            camInfoManager->setCameraInfo(cameraInfoData);
        }
        cameraInfoPublisher = node->create_publisher<ImageMsgs::CameraInfo>(cameraName + "/camera_info", queueSize, options);
    }
    return node->create_publisher<RosMsg>(rosTopic, queueSize, options);
}

template <class RosMsg, class DaiMsg>
std::shared_ptr<image_transport::CameraPublisher> BridgePublisher<RosMsg, DaiMsg>::advertise(int queueSize, std::true_type) {
    camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(node.get(), cameraName.empty() ? "camera" : cameraName, cameraParamUri);
    if(cameraParamUri.empty()) camInfoManager->setCameraInfo(cameraInfoData);
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    return std::make_shared<image_transport::CameraPublisher>(image_transport::create_camera_publisher(*node, rosTopic, rclcpp::QoS(queueSize), options));
}

template <class RosMsg, class DaiMsg>
void BridgePublisher<RosMsg, DaiMsg>::daiCallback(const std::string& name, std::shared_ptr<dai::ADatatype> data) {
    auto daiDataPtr = std::dynamic_pointer_cast<DaiMsg>(data);
    publishHelper(daiDataPtr);
}

template <class RosMsg, class DaiMsg>
void BridgePublisher<RosMsg, DaiMsg>::startPublisherThread() {
    std::lock_guard<std::mutex> lock(controlMutex);
    if(isCallbackAdded || readingThread.joinable()) throw std::logic_error("BridgePublisher is already started");
    if(!daiMessageQueue) throw std::invalid_argument("BridgePublisher requires a queue");
    stopRequested = false;
    readingThread = std::thread([this]() {
        while(!stopRequested && rclcpp::ok(node->get_node_base_interface()->get_context())) {
            try {
                bool timedOut = false;
                auto data = daiMessageQueue->template get<DaiMsg>(std::chrono::milliseconds(50), timedOut);
                if(data && !stopRequested) publishHelper(data);
            } catch(const std::exception& e) {
                if(!stopRequested) RCLCPP_WARN(node->get_logger(), "Bridge worker stopped: %s", e.what());
                break;
            }
        }
    });
}

template <class RosMsg, class DaiMsg>
void BridgePublisher<RosMsg, DaiMsg>::addPublisherCallback() {
    std::lock_guard<std::mutex> lock(controlMutex);
    if(isCallbackAdded || readingThread.joinable()) throw std::logic_error("BridgePublisher is already started");
    if(!daiMessageQueue) throw std::invalid_argument("BridgePublisher requires a queue");
    stopRequested = false;
    {
        std::lock_guard<std::mutex> callbackLock(callbackState->mutex);
        callbackState->owner = this;
    }
    callbackId = daiMessageQueue->addCallback([state = callbackState](const std::string& name, std::shared_ptr<dai::ADatatype> data) {
        std::lock_guard<std::mutex> callbackLock(state->mutex);
        if(!state->owner) return;
        try {
            state->owner->daiCallback(name, data);
        } catch(const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(state->owner->node->get_logger(), *state->owner->node->get_clock(), 5000, "Bridge conversion failed: %s", e.what());
        }
    });
    isCallbackAdded = true;
}

// Add this method to the BridgePublisher class
template <class RosMsg, class DaiMsg>
void BridgePublisher<RosMsg, DaiMsg>::publishTransform(std::shared_ptr<tf2_ros::TransformBroadcaster> tfPub,
                                                       const geometry_msgs::msg::TransformStamped& transform) {
    tfPub->sendTransform(transform);
}
template <class RosMsg, class DaiMsg>
void BridgePublisher<RosMsg, DaiMsg>::publishHelper(std::shared_ptr<DaiMsg> inDataPtr) {
    if(!inDataPtr) return;
    size_t mainSubCount = 0;
    const size_t infoSubCount = cameraInfoPublisher ? cameraInfoPublisher->get_subscription_count() : 0;
    if constexpr(std::is_same_v<RosMsg, ImageMsgs::Image>) {
        mainSubCount = rosPublisher->getNumSubscribers();
    } else if constexpr(std::is_same_v<RosMsg, geometry_msgs::msg::TransformStamped>) {
        mainSubCount = 1;
    } else {
        mainSubCount = rosPublisher->get_subscription_count();
    }
    if(lazyPublisher && mainSubCount == 0 && infoSubCount == 0) return;
    std::deque<RosMsg> opMsgs;
    converter(inDataPtr, opMsgs);
    for(auto& currMsg : opMsgs) {
        if(!lazyPublisher || mainSubCount > 0) {
            if constexpr(std::is_same_v<RosMsg, ImageMsgs::Image>) {
                auto info = camInfoManager->getCameraInfo();
                info.header = currMsg.header;
                rosPublisher->publish(currMsg, info);
            } else if constexpr(std::is_same_v<RosMsg, geometry_msgs::msg::TransformStamped>) {
                publishTransform(rosPublisher, currMsg);
            } else {
                if constexpr(std::is_same_v<RosMsg, nav_msgs::msg::Odometry>) {
                    if(pubTransform) {
                        geometry_msgs::msg::TransformStamped transform;
                        transform.header = currMsg.header;
                        transform.child_frame_id = currMsg.child_frame_id;
                        transform.transform.translation.x = currMsg.pose.pose.position.x;
                        transform.transform.translation.y = currMsg.pose.pose.position.y;
                        transform.transform.translation.z = currMsg.pose.pose.position.z;
                        transform.transform.rotation = currMsg.pose.pose.orientation;
                        publishTransform(tfBroadcaster, transform);
                    }
                }
                // Publish after using the header below; no intermediate message copy.
            }
        }
        if constexpr(std::is_same_v<RosMsg, ImageMsgs::CompressedImage> || std::is_same_v<RosMsg, FFMPEGMsgs::FFMPEGPacket>) {
            if(infoSubCount > 0) {
                auto info = camInfoManager->getCameraInfo();
                info.header = currMsg.header;
                cameraInfoPublisher->publish(info);
            }
        }
        if constexpr(!std::is_same_v<RosMsg, ImageMsgs::Image> && !std::is_same_v<RosMsg, geometry_msgs::msg::TransformStamped>) {
            if(!lazyPublisher || mainSubCount > 0) rosPublisher->publish(std::make_unique<RosMsg>(std::move(currMsg)));
        }
    }
}

template <class RosMsg, class DaiMsg>
BridgePublisher<RosMsg, DaiMsg>::~BridgePublisher() {
    stop();
}

template <class RosMsg, class DaiMsg>
void BridgePublisher<RosMsg, DaiMsg>::stop() {
    std::lock_guard<std::mutex> lock(controlMutex);
    stopRequested = true;
    {
        // Drain in-flight work, then disable even callbacks already copied by a producer.
        std::lock_guard<std::mutex> callbackLock(callbackState->mutex);
        callbackState->owner = nullptr;
    }
    // Never hold the callback gate while acquiring the SDK callback registry lock.
    if(callbackId >= 0) {
        daiMessageQueue->removeCallback(callbackId);
        callbackId = -1;
        isCallbackAdded = false;
    }
    if(readingThread.joinable()) readingThread.join();
}

}  // namespace depthai_bridge
