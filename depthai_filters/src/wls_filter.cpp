#include "depthai_filters/wls_filter.hpp"

#include <cmath>
#include <limits>
#include <memory>

#if __has_include("cv_bridge/cv_bridge.hpp")
    #include "cv_bridge/cv_bridge.hpp"
#else
    #include "cv_bridge/cv_bridge.h"
#endif
#include "depthai_filters/utils.hpp"

namespace depthai_filters {

WLSFilter::WLSFilter(const rclcpp::NodeOptions& options) : rclcpp::Node("wls_filter", options) {
    onInit();
}
void WLSFilter::onInit() {
    const auto qos = utils::inputQoS(*this);
    disparityImgSub.subscribe(this, "disparity/image_raw", qos);
    leftImgSub.subscribe(this, "left/image_raw", qos);
    disparityInfoSub.subscribe(this, "disparity/camera_info", qos);
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), disparityImgSub, disparityInfoSub, leftImgSub);
    sync->registerCallback(std::bind(&WLSFilter::wlsCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
    filter->setLambda(this->declare_parameter<double>("lambda", 8000.0));
    filter->setSigmaColor(this->declare_parameter<double>("sigma_color", 1.5));
    maxDisparity = this->declare_parameter<double>("max_disparity", 760.0);
    declare_parameter<int>("disparity_fractional_bits", 5);
    const auto initial =
        parameterCB({get_parameter("lambda"), get_parameter("sigma_color"), get_parameter("max_disparity"), get_parameter("disparity_fractional_bits")});
    if(!initial.successful) throw std::invalid_argument(initial.reason);
    paramCBHandle = this->add_on_set_parameters_callback(std::bind(&WLSFilter::parameterCB, this, std::placeholders::_1));
    depthPub = image_transport::create_camera_publisher(*this, "wls_filtered", rclcpp::QoS(1));
}

rcl_interfaces::msg::SetParametersResult WLSFilter::parameterCB(const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    for(const auto& p : params) {
        if(p.get_name() == "lambda" || p.get_name() == "sigma_color" || p.get_name() == "max_disparity") {
            if(p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || !std::isfinite(p.as_double()) || p.as_double() <= 0) {
                res.successful = false;
                res.reason = p.get_name() + " must be a finite positive double";
            }
        } else if(p.get_name() == "disparity_fractional_bits") {
            if(p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER || p.as_int() < 0 || p.as_int() > 5) {
                res.successful = false;
                res.reason = "disparity_fractional_bits must be between 0 and 5";
            }
        }
    }
    return res;
}

void WLSFilter::wlsCB(const sensor_msgs::msg::Image::ConstSharedPtr& disp,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& disp_info,
                      const sensor_msgs::msg::Image::ConstSharedPtr& leftImg) {
    if(!disp || !disp_info || !leftImg) return;
    auto left = utils::msgToMat(get_logger(), leftImg, sensor_msgs::image_encodings::MONO8);
    auto input = utils::msgToMat(get_logger(), disp, disp->encoding);
    const double focalBaseline = std::abs(disp_info->p[3]);
    if(left.empty() || input.empty() || left.size() != input.size() || !std::isfinite(focalBaseline) || focalBaseline <= 0
       || (disp->encoding != "mono8" && disp->encoding != "16UC1" && disp->encoding != "32FC1")) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "WLS requires matching disparity/left images and calibrated right-camera projection");
        return;
    }
    try {
        cv::Mat pixels;
        const double scale = disp->encoding == "16UC1" ? 1.0 / (1u << get_parameter("disparity_fractional_bits").as_int()) : 1.0;
        input.convertTo(pixels, CV_32FC1, scale);
        const double maximum = get_parameter("max_disparity").as_double();
        for(int y = 0; y < pixels.rows; ++y)
            for(int x = 0; x < pixels.cols; ++x) {
                auto& value = pixels.at<float>(y, x);
                if(!std::isfinite(value) || value <= 0 || value > maximum || value > 2047) value = 0;
            }
        cv::Mat fixed, filtered;
        pixels.convertTo(fixed, CV_16SC1, 16.0);  // OpenCV WLS uses four fractional bits.
        filter->setLambda(get_parameter("lambda").as_double());
        filter->setSigmaColor(get_parameter("sigma_color").as_double());
        filter->filter(fixed, left, filtered);
        cv::Mat metres(filtered.size(), CV_32FC1);
        for(int y = 0; y < filtered.rows; ++y)
            for(int x = 0; x < filtered.cols; ++x) {
                const double disparity = filtered.at<int16_t>(y, x) / 16.0;
                metres.at<float>(y, x) = pixels.at<float>(y, x) > 0 && disparity > 0 ? focalBaseline / disparity : std::numeric_limits<float>::quiet_NaN();
            }
        sensor_msgs::msg::Image depth;
        cv_bridge::CvImage(disp->header, "32FC1", metres).toImageMsg(depth);
        auto info = *disp_info;
        info.header = depth.header;
        depthPub.publish(depth, info);
    } catch(const cv::Exception& error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping WLS input: %s", error.what());
    }
}
}  // namespace depthai_filters

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::WLSFilter);
