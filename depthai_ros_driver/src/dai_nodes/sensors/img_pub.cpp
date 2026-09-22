#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"

#include <rclcpp/logging.hpp>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/properties/VideoEncoderProperties.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/stream_diagnostics.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace sensor_helpers {
ImagePublisher::ImagePublisher(std::shared_ptr<rclcpp::Node> node,
                               std::shared_ptr<dai::Pipeline> pipeline,
                               const std::string& qName,
                               dai::Node::Output* out,
                               bool synced,
                               bool ipcEnabled,
                               const utils::VideoEncoderConfig& encoderConfig)
    : node(node), pipeline(pipeline), encConfig(encoderConfig), out(out), qName(qName), ipcEnabled(ipcEnabled), synced(synced) {
    if(encoderConfig.enabled) {
        encoder = createEncoder(pipeline, encoderConfig);
        this->out->link(encoder->input);
    }
}
void ImagePublisher::setup(std::shared_ptr<dai::Device> device, const utils::ImgConverterConfig& convConf, const utils::ImgPublisherConfig& pubConf) {
    if(pubConf.publishCompressed && !encConfig.enabled) {
        throw std::invalid_argument(pubConf.daiNodeName + ".i_publish_compressed requires i_low_bandwidth=true. "
                                    "For raw device transport, use an image_transport compressed subscriber instead.");
    }
    convConfig = convConf;
    pubConfig = pubConf;
    createImageConverter(device);
    createInfoManager(device);
    if(pubConfig.topicName.empty()) {
        throw std::runtime_error("Topic name cannot be empty!");
    }
    rclcpp::PublisherOptions pubOptions;
    pubOptions.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    if(pubConfig.publishCompressed) {
        if(encConfig.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
            compressedImgPub =
                node->create_publisher<sensor_msgs::msg::CompressedImage>(pubConfig.topicName + pubConfig.compressedTopicSuffix, rclcpp::QoS(10), pubOptions);
        } else {
            ffmpegPub = node->create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
                pubConfig.topicName + pubConfig.compressedTopicSuffix, rclcpp::QoS(10), pubOptions);
        }
        infoPub =
            node->create_publisher<sensor_msgs::msg::CameraInfo>(pubConfig.topicName + pubConfig.infoSuffix + "/camera_info", rclcpp::QoS(10), pubOptions);
    } else {
        imgPubIT = image_transport::create_camera_publisher(*node, pubConfig.topicName + pubConfig.topicSuffix, rclcpp::QoS(1), pubOptions);
    }
    diagnostics = std::make_unique<StreamDiagnostics>(node, "stream: " + qName, [this]() {
        const auto current = pipeline.lock();
        return current && current->isRunning() && shouldPublish();
    });
    if(!synced) {
        if(encConfig.enabled) {
            dataQ = encoder->out.createOutputQueue(pubConf.maxQSize, pubConf.qBlocking);
        } else {
            dataQ = out->createOutputQueue(pubConf.maxQSize, pubConf.qBlocking);
        }
        addQueueCB();
    }
}

void ImagePublisher::createImageConverter(std::shared_ptr<dai::Device> device) {
    converter = std::make_shared<depthai_bridge::ImageConverter>(convConfig.tfPrefix, convConfig.interleaved, convConfig.getBaseDeviceTimestamp);
    converter->setClock(node->get_clock());
    converter->setUpdateRosBaseTimeOnToRosMsg(convConfig.updateROSBaseTimeOnRosMsg);
    if(convConfig.lowBandwidth) {
        converter->convertFromBitstream(convConfig.encoding);
        if(convConfig.isStereo && !convConfig.outputDisparity) {
            try {
                auto calHandler = device->readCalibration();
                double baseline = calHandler.getBaselineDistance(pubConfig.leftSocket, pubConfig.rightSocket, false);
                if(convConfig.reverseSocketOrder) {
                    baseline = calHandler.getBaselineDistance(pubConfig.rightSocket, pubConfig.leftSocket, false);
                }
                converter->convertDispToDepth(baseline);
            } catch(const std::exception& e) {
                RCLCPP_DEBUG(node->get_logger(), "Failed to convert disparity to depth: %s", e.what());
            }
        }
    }
    if(convConfig.addExposureOffset) {
        converter->addExposureOffset(convConfig.expOffset);
    }
    if(convConfig.reverseSocketOrder) {
        converter->reverseStereoSocketOrder();
    }
    if(convConfig.alphaScalingEnabled) {
        converter->setAlphaScaling(convConfig.alphaScaling);
    }
    if(convConfig.isStereo && !convConfig.outputDisparity) {
        auto calHandler = device->readCalibration();
        double baseline = calHandler.getBaselineDistance(pubConfig.leftSocket, pubConfig.rightSocket, false);
        if(convConfig.reverseSocketOrder) {
            baseline = calHandler.getBaselineDistance(pubConfig.rightSocket, pubConfig.leftSocket, false);
        }
        converter->convertDispToDepth(baseline);
    }
    converter->setFFMPEGEncoding(convConfig.ffmpegEncoder);
}

std::shared_ptr<dai::node::VideoEncoder> ImagePublisher::createEncoder(std::shared_ptr<dai::Pipeline> pipeline,
                                                                       const utils::VideoEncoderConfig& encoderConfig) {
    auto enc = pipeline->create<dai::node::VideoEncoder>();
    enc->setQuality(encoderConfig.quality);
    enc->setProfile(encoderConfig.profile);
    if(encoderConfig.profile != dai::VideoEncoderProperties::Profile::MJPEG) {
        enc->setBitrate(encoderConfig.bitrate);
        enc->setKeyframeFrequency(encoderConfig.frameFreq);
    }
    return enc;
}
void ImagePublisher::createInfoManager(std::shared_ptr<dai::Device> device) {
    infoManager = std::make_shared<camera_info_manager::CameraInfoManager>(
        node->create_sub_node(std::string(node->get_name()) + "/" + pubConfig.daiNodeName).get(), "/" + pubConfig.daiNodeName + pubConfig.infoMgrSuffix);
    if(pubConfig.calibrationFile.empty()) {
        auto calHandler = device->readCalibration();
        auto info = sensor_helpers::getCalibInfo(node->get_logger(), converter, calHandler, pubConfig.socket, pubConfig.width, pubConfig.height);
        if(pubConfig.rectified) {
            std::fill(info.d.begin(), info.d.end(), 0.0);
            info.r[0] = info.r[4] = info.r[8] = 1.0;
        }
        infoManager->setCameraInfo(info);
    } else {
        infoManager->loadCameraInfo(pubConfig.calibrationFile);
    }
};
ImagePublisher::~ImagePublisher() {
    closeQueue();
};

void ImagePublisher::closeQueue() {
    if(dataQ && !dataQ->isClosed()) {
        dataQ->removeCallback(cbID);
        dataQ->close();
    }
    diagnostics.reset();
}
void ImagePublisher::link(dai::Node::Input& in) {
    if(encConfig.enabled) {
        encoder->out.link(in);
    } else {
        out->link(in);
    }
}
std::shared_ptr<dai::MessageQueue> ImagePublisher::getQueue() {
    return dataQ;
}
bool ImagePublisher::isSynced() {
    return synced;
}
void ImagePublisher::addQueueCB() {
    cbID = dataQ->addCallback([this](const std::shared_ptr<dai::ADatatype>& data) { publish(data); });
}

std::string ImagePublisher::getQueueName() {
    return qName;
}
std::shared_ptr<Image> ImagePublisher::convertData(const std::shared_ptr<dai::ADatatype>& data) {
    if(diagnostics) {
        if(auto buffer = std::dynamic_pointer_cast<dai::Buffer>(data)) diagnostics->record(buffer->getTimestamp(), buffer->getSequenceNum());
    }
    sensor_msgs::msg::CameraInfo info;
    auto img = std::make_shared<Image>();
    if(encConfig.enabled) {
        auto daiImg = std::dynamic_pointer_cast<dai::EncodedFrame>(data);
        if(!daiImg) {
            RCLCPP_ERROR(node->get_logger(), "Expected an EncodedFrame on %s", qName.c_str());
            return nullptr;
        }
        if(pubConfig.calibrationFile.empty()) {
            info = converter->generateCameraInfo(daiImg);
        } else {
            info = infoManager->getCameraInfo();
        }
        if(pubConfig.publishCompressed) {
            if(encConfig.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
                std::deque<sensor_msgs::msg::CompressedImage> deq;
                converter->toRosCompressedMsg(daiImg, deq);
                img->compressedImg = std::make_unique<sensor_msgs::msg::CompressedImage>(std::move(deq.front()));
                info.header = img->compressedImg->header;
            } else {
                std::deque<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> deq;
                converter->toRosFFMPEGPacket(daiImg, deq);
                img->ffmpegPacket = std::make_unique<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(std::move(deq.front()));
                img->ffmpegPacket->width = info.width;
                img->ffmpegPacket->height = info.height;
                info.header = img->ffmpegPacket->header;
            }
        } else {
            auto rawMsg = converter->toRosMsgRawPtr(daiImg, info);
            info.header = rawMsg.header;
            sensor_msgs::msg::Image::UniquePtr msg = std::make_unique<sensor_msgs::msg::Image>(std::move(rawMsg));
            img->image = std::move(msg);
        }
    } else {
        auto daiImg = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        if(!daiImg) {
            RCLCPP_ERROR(node->get_logger(), "Expected an ImgFrame on %s", qName.c_str());
            return nullptr;
        }
        if(pubConfig.calibrationFile.empty()) {
            info = converter->generateCameraInfo(daiImg);
        } else {
            info = infoManager->getCameraInfo();
        }
        auto rawMsg = converter->toRosMsgRawPtr(daiImg, info);
        info.header = rawMsg.header;
        sensor_msgs::msg::Image::UniquePtr msg = std::make_unique<sensor_msgs::msg::Image>(std::move(rawMsg));
        img->image = std::move(msg);
    }
    if(pubConfig.rectified) {
        info.r[0] = info.r[4] = info.r[8] = 1.0;
    }
    if(pubConfig.undistorted) {
        std::fill(info.d.begin(), info.d.end(), 0.0);
    }
    sensor_msgs::msg::CameraInfo::UniquePtr infoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>(info);
    img->info = std::move(infoMsg);
    return img;
}
void ImagePublisher::publish(std::shared_ptr<Image> img) {
    if(!img) {
        return;
    }
    if(pubConfig.publishCompressed) {
        if(encConfig.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
            compressedImgPub->publish(std::move(img->compressedImg));
        } else {
            ffmpegPub->publish(std::move(img->ffmpegPacket));
        }
        infoPub->publish(std::move(img->info));
    } else {
        if(!pubConfig.lazyPub || imgPubIT.getNumSubscribers() > 0) {
            if(ipcEnabled) {
                imgPubIT.publish(std::move(img->image), std::move(img->info));
            } else {
                imgPubIT.publish(*img->image, *img->info);
            }
        }
    }
}
void ImagePublisher::publish(std::shared_ptr<Image> img, rclcpp::Time timestamp) {
    if(!img) {
        return;
    }
    img->info->header.stamp = timestamp;
    if(pubConfig.publishCompressed) {
        if(encConfig.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
            img->compressedImg->header.stamp = timestamp;
        } else {
            img->ffmpegPacket->header.stamp = timestamp;
            img->ffmpegPacket->pts = timestamp.nanoseconds();
        }
    } else {
        img->image->header.stamp = timestamp;
    }
    publish(img);
}

void ImagePublisher::publish(const std::shared_ptr<dai::ADatatype>& data) {
    if(rclcpp::ok() && shouldPublish()) {
        auto img = convertData(data);
        publish(img);
    }
}

bool ImagePublisher::shouldPublish() const {
    if(!pubConfig.lazyPub) {
        return true;
    }
    if(pubConfig.publishCompressed) {
        return (infoPub && infoPub->get_subscription_count() > 0) || (compressedImgPub && compressedImgPub->get_subscription_count() > 0)
               || (ffmpegPub && ffmpegPub->get_subscription_count() > 0);
    }
    // CameraPublisher counts both image and camera-info subscriptions.
    return imgPubIT.getNumSubscribers() > 0;
}

rclcpp::Time ImagePublisher::getTimestamp(const std::shared_ptr<dai::ADatatype>& data) {
    auto buffer = std::dynamic_pointer_cast<dai::Buffer>(data);
    if(!buffer) {
        throw std::invalid_argument("Expected a timestamped image buffer on " + qName);
    }
    return rclcpp::Time(converter->getRosHeader(buffer, convConfig.addExposureOffset, convConfig.expOffset).stamp);
}
}  // namespace sensor_helpers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
