#include "depthai_bridge/PointCloudConverter.hpp"

#include <cstring>
#include <limits>
#include <stdexcept>

#include "sensor_msgs/msg/point_field.hpp"

namespace depthai_bridge {

PointCloudConverter::PointCloudConverter(std::string frameName, bool getBaseDeviceTimestamp)
    : BaseConverter(std::move(frameName), getBaseDeviceTimestamp), scaleFactor(1.0) {}

PointCloudConverter::~PointCloudConverter() = default;

void PointCloudConverter::setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit depthUnit) {
    // Default is millimeter
    switch(depthUnit) {
        case dai::StereoDepthConfig::AlgorithmControl::DepthUnit::MILLIMETER:
            scaleFactor = 1.0f;
            break;
        case dai::StereoDepthConfig::AlgorithmControl::DepthUnit::METER:
            scaleFactor = 0.001f;
            break;
        case dai::StereoDepthConfig::AlgorithmControl::DepthUnit::CENTIMETER:
            scaleFactor = 0.01f;
            break;
        case dai::StereoDepthConfig::AlgorithmControl::DepthUnit::FOOT:
            scaleFactor = 0.3048f;
            break;
        case dai::StereoDepthConfig::AlgorithmControl::DepthUnit::INCH:
            scaleFactor = 0.0254f;
            break;
        case dai::StereoDepthConfig::AlgorithmControl::DepthUnit::CUSTOM:
            scaleFactor = 1.0f;
            break;
    }
}
double PointCloudConverter::getScaleFactor() const {
    return scaleFactor;
}
void PointCloudConverter::toRosMsg(std::shared_ptr<dai::PointCloudData> inPcl, std::deque<sensor_msgs::msg::PointCloud2>& pclMsgs) {
    sensor_msgs::msg::PointCloud2 msg;
    bool isColored = inPcl->isColor();
    unsigned int width = inPcl->getWidth();
    unsigned int height = inPcl->getHeight();
    msg.header = getRosHeader(inPcl);

    msg.fields.clear();
    sensor_msgs::msg::PointField field_x;
    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_x.count = 1;
    msg.fields.push_back(field_x);

    sensor_msgs::msg::PointField field_y;
    field_y.name = "y";
    field_y.offset = 4;
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count = 1;
    msg.fields.push_back(field_y);

    sensor_msgs::msg::PointField field_z;
    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count = 1;
    msg.fields.push_back(field_z);

    uint32_t pointStep = 12;
    if(isColored) {
        sensor_msgs::msg::PointField field_rgb;
        field_rgb.name = "rgb";
        field_rgb.offset = 12;
        field_rgb.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_rgb.count = 1;
        msg.fields.push_back(field_rgb);
        pointStep += 4;
    }
    msg.point_step = pointStep;

    // Read the SDK buffer in place; getPoints()/getPointsRGB() return full copies.
    auto data = inPcl->getData();
    const size_t numPoints = data.size() / (isColored ? sizeof(dai::Point3fRGBA) : sizeof(dai::Point3f));
    const bool organized = inPcl->isOrganized();
    if(organized) {
        if(numPoints != static_cast<size_t>(width) * height) {
            throw std::runtime_error("Organized point cloud size does not match its width and height");
        }
        msg.width = width;
        msg.height = height;
    } else {
        // Sparse clouds contain only valid points; their width/height describe the source frame.
        msg.width = static_cast<uint32_t>(numPoints);
        msg.height = 1;
    }
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(static_cast<size_t>(msg.row_step) * msg.height);

    // REP-117: organized clouds keep invalid (zero-depth) points as NaN and are not dense.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    bool dense = true;
    auto writeXYZ = [&](float* ptr, float x, float y, float z) {
        if(organized && z == 0.0f) {
            ptr[0] = ptr[1] = ptr[2] = nan;
            dense = false;
        } else {
            ptr[0] = x * scaleFactor;
            ptr[1] = y * scaleFactor;
            ptr[2] = z * scaleFactor;
        }
    };

    if(isColored) {
        const auto* points = reinterpret_cast<const dai::Point3fRGBA*>(data.data());
        for(size_t i = 0; i < numPoints; ++i) {
            float* ptr = reinterpret_cast<float*>(&msg.data[i * pointStep]);
            const auto& pt = points[i];
            writeXYZ(ptr, pt.x, pt.y, pt.z);
            uint32_t rgb = (pt.r << 16) | (pt.g << 8) | pt.b;
            std::memcpy(&ptr[3], &rgb, sizeof(float));
        }
    } else {
        const auto* points = reinterpret_cast<const dai::Point3f*>(data.data());
        for(size_t i = 0; i < numPoints; ++i) {
            const auto& pt = points[i];
            writeXYZ(reinterpret_cast<float*>(&msg.data[i * pointStep]), pt.x, pt.y, pt.z);
        }
    }
    msg.is_dense = dense;

    pclMsgs.push_back(std::move(msg));
}

}  // namespace depthai_bridge
