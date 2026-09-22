#include <condition_variable>
#include <cstdio>
#include <functional>
#include <memory>
#include <random>

#include "depthai_examples/common.hpp"

#if __has_include("cv_bridge/cv_bridge.hpp")
    #include "cv_bridge/cv_bridge.hpp"
#else
    #include "cv_bridge/cv_bridge.h"
#endif
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/host/HostNode.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "rclcpp/node.hpp"

cv::Mat generateRandomImage(int width, int height) {
    cv::Mat randomImage(height, width, CV_8UC3);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);

    for(int i = 0; i < height; ++i) {
        for(int j = 0; j < width; ++j) {
            randomImage.at<cv::Vec3b>(i, j) = cv::Vec3b(dis(gen), dis(gen), dis(gen));
        }
    }
    return randomImage;
}

class HostDisplay : public dai::node::CustomNode<HostDisplay> {
   public:
    HostDisplay() {
        sendProcessingToPipeline(false);
    }

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> message) override {
        if(message == nullptr) return nullptr;

        auto frame = message->get<dai::ImgFrame>("frame");
        if(frame == nullptr) return nullptr;

        cv::imshow("HostDisplay", frame->getCvFrame());
        int key = cv::waitKey(1);
        if(key == 'q') {
            std::cout << "Detected 'q' - stopping the pipeline..." << std::endl;
            stopPipeline();
        }

        return nullptr;
    }
};
class ImgSubscriber : public dai::NodeCRTP<dai::node::ThreadedHostNode, ImgSubscriber> {
   public:
    std::shared_ptr<depthai_bridge::ImageConverter> conv;
    Output output = dai::Node::Output{*this, {}};
    void createSubScription(std::shared_ptr<rclcpp::Node> node) {
        imgSub = node->create_subscription<sensor_msgs::msg::Image>(
            "rgb_image", rclcpp::SensorDataQoS(), std::bind(&ImgSubscriber::subCB, this, std::placeholders::_1));
    }
    void run() override {
        while(isRunning()) {
            sensor_msgs::msg::Image::SharedPtr message;
            {
                std::unique_lock<std::mutex> lock(imgMutex);
                ready.wait_for(lock, std::chrono::milliseconds(50), [this]() { return rosImg != nullptr; });
                if(!isRunning()) break;
                message = std::move(rosImg);
            }
            if(!message) continue;
            dai::ImgFrame daiImg;
            daiImg.setInstanceNum(static_cast<int>(dai::CameraBoardSocket::CAM_A));
            conv->toDaiMsg(*message, daiImg);
            output.send(std::make_shared<dai::ImgFrame>(std::move(daiImg)));
        }
    }

   private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSub;
    sensor_msgs::msg::Image::SharedPtr rosImg;
    std::mutex imgMutex;
    std::condition_variable ready;
    void subCB(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lck(imgMutex);
        rosImg = msg;
        ready.notify_one();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::string tfPrefix = "oak";
    auto node = rclcpp::Node::make_shared(tfPrefix);
    tfPrefix = depthai_examples::framePrefix(node);

    dai::Pipeline pipeline;

    auto rosSubscriberNode = pipeline.create<ImgSubscriber>();
    rosSubscriberNode->createSubScription(node);
    auto display = pipeline.create<HostDisplay>();
    rosSubscriberNode->output.link(display->inputs["frame"]);
    // Create a bridge publisher for RGB images
    auto rgbConverter = std::make_shared<depthai_bridge::ImageConverter>(tfPrefix + "_rgb_camera_optical_frame", true);
    rgbConverter->setClock(node->get_clock());
    rosSubscriberNode->conv = rgbConverter;

    auto random_image_publisher = node->create_publisher<sensor_msgs::msg::Image>("rgb_image", 5);

    auto timer_callback = [&]() {
        cv::Mat randomImage = generateRandomImage(640, 480);

        sensor_msgs::msg::Image::SharedPtr randomImageMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", randomImage).toImageMsg();
        random_image_publisher->publish(*randomImageMsg);
    };

    rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(std::chrono::seconds(1), timer_callback);
    pipeline.start();

    depthai_examples::spinPipeline(node, pipeline);

    return 0;
}
