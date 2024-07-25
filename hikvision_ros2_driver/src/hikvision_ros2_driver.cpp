#include "hikvision_ros2_driver/hikvision_ros2_driver.hpp"

// ros
#include <hikvision_interface/msg/hik_image_info.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>

// hikvision sdk
#include <MvCameraControl.h>

#define MV_CHECK(logger, func, ...)                                            \
    do {                                                                       \
        int nRet = func(__VA_ARGS__);                                          \
        if (MV_OK != nRet) {                                                   \
            RCLCPP_ERROR(logger, "hikvision sdk error: " #func " = %d", nRet); \
        }                                                                      \
    } while (0)

using hikvision_interface::msg::HikImageInfo;

namespace hikvision_ros2_driver {

struct HikvisionDriver::Impl {
    std::unique_ptr<rclcpp::Logger> logger;

    void *handle;
    std::string camera_name;
    image_transport::Publisher img_pub;
    std::shared_ptr<rclcpp::Publisher<HikImageInfo>> p_info_pub;
    static void image_callback_ex(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser);
};

void HikvisionDriver::Impl::image_callback_ex(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser) {
    auto node = reinterpret_cast<HikvisionDriver *>(pUser);

    uint64_t dev_stamp = (uint64_t)pFrameInfo->nDevTimeStampHigh << 32ull | (uint64_t)pFrameInfo->nDevTimeStampLow;
    uint64_t host_stamp = pFrameInfo->nHostTimeStamp;

    auto p_img_msg = std::make_unique<sensor_msgs::msg::Image>();
    if (pFrameInfo->nFrameLen > p_img_msg->data.max_size()) {
        RCLCPP_ERROR_ONCE(node->get_logger(), "image bytes exceed max available size");
        return;
    }
    p_img_msg->header.frame_id = node->pImpl->camera_name;
    p_img_msg->header.stamp.nanosec = host_stamp % 1000ull * 1000000ull;
    p_img_msg->header.stamp.sec = host_stamp / 1000ull;
    p_img_msg->is_bigendian = false;
    p_img_msg->width = pFrameInfo->nWidth;
    p_img_msg->height = pFrameInfo->nHeight;
    if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerRG8) {
        p_img_msg->step = pFrameInfo->nWidth * 1;
        p_img_msg->encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerBG8) {
        p_img_msg->step = pFrameInfo->nWidth * 1;
        p_img_msg->encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerGR8) {
        p_img_msg->step = pFrameInfo->nWidth * 1;
        p_img_msg->encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerGB8) {
        p_img_msg->step = pFrameInfo->nWidth * 1;
        p_img_msg->encoding = sensor_msgs::image_encodings::BAYER_GBRG8;
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_Mono8) {
        p_img_msg->step = pFrameInfo->nWidth * 1;
        p_img_msg->encoding = sensor_msgs::image_encodings::MONO8;
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_RGB8_Packed) {
        p_img_msg->step = pFrameInfo->nWidth * 3;
        p_img_msg->encoding = sensor_msgs::image_encodings::RGB8;
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_BGR8_Packed) {
        p_img_msg->step = pFrameInfo->nWidth * 3;
        p_img_msg->encoding = sensor_msgs::image_encodings::BGR8;
    } else {
        RCLCPP_ERROR_ONCE(node->get_logger(), "unsupport pixel format: %d", (int)pFrameInfo->enPixelType);
        return;
    }
    p_img_msg->data.resize(p_img_msg->height * p_img_msg->step);
    if (pFrameInfo->nFrameLen < p_img_msg->data.size()) {
        RCLCPP_ERROR(node->get_logger(), "nFrameLen < data.size(), len=%d", pFrameInfo->nFrameLen);
        return;
    }
    std::copy_n(pData, p_img_msg->data.size(), p_img_msg->data.data());
    node->pImpl->img_pub.publish(std::move(p_img_msg));

    auto p_info_msg = std::make_unique<hikvision_interface::msg::HikImageInfo>();
    p_info_msg->header.frame_id = node->pImpl->camera_name;
    p_info_msg->header.stamp.nanosec = host_stamp % 1000ull * 1000000ull;
    p_info_msg->header.stamp.sec = host_stamp / 1000ull;
    p_info_msg->dev_stamp.nanosec = dev_stamp % 1000000000ull;
    p_info_msg->dev_stamp.sec = dev_stamp / 1000000000ull;
    p_info_msg->frame_num = pFrameInfo->nFrameNum;
    p_info_msg->gain = pFrameInfo->fGain;
    p_info_msg->exposure = pFrameInfo->fExposureTime;
    p_info_msg->red = pFrameInfo->nRed;
    p_info_msg->green = pFrameInfo->nGreen;
    p_info_msg->blue = pFrameInfo->nBlue;
    node->pImpl->p_info_pub->publish(std::move(p_info_msg));
}

HikvisionDriver::HikvisionDriver(const rclcpp::NodeOptions &options)
    : rclcpp::Node("hikvision_ros2_driver_node", options), pImpl(std::make_unique<Impl>()) {
    auto logger = get_logger();
    pImpl->logger = std::make_unique<rclcpp::Logger>(logger);

    declare_parameter<std::string>("camera_name");
    pImpl->camera_name = get_parameter("camera_name").as_string();
    RCLCPP_INFO(logger, "trying to open camera: '%s'", pImpl->camera_name.c_str());

    rclcpp::QoS qos(1);
    pImpl->img_pub = image_transport::create_publisher(this, "raw/image", qos.get_rmw_qos_profile());
    pImpl->p_info_pub = create_publisher<HikImageInfo>("info", qos);

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    MV_CHECK(logger, MV_CC_EnumDevices, MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);

    for (uint32_t nDeviceId = 0; nDeviceId < stDeviceList.nDeviceNum; nDeviceId++) {
        auto *pDeviceInfo = stDeviceList.pDeviceInfo[nDeviceId];
        const char *pUserDefinedName = nullptr;
        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            pUserDefinedName = (const char *)pDeviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName;
            if (pUserDefinedName == pImpl->camera_name) {
                int nIp1 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
                int nIp2 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
                int nIp3 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
                int nIp4 = (pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
                RCLCPP_INFO(logger, "[%s]: GIGE, %s, %d.%d.%d.%d", pUserDefinedName,
                            pDeviceInfo->SpecialInfo.stGigEInfo.chModelName, nIp1, nIp2, nIp3, nIp4);
            }
        } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
            pUserDefinedName = (const char *)pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName;
            if (pUserDefinedName == pImpl->camera_name) {
                RCLCPP_INFO(logger, "[%s]: USB, %s", pUserDefinedName,
                            pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName);
            }
        } else {
            RCLCPP_WARN(logger, "type(%d) not support", pDeviceInfo->nTLayerType);
        }
        if (pUserDefinedName == pImpl->camera_name) {
            MV_CHECK(logger, MV_CC_CreateHandle, &pImpl->handle, pDeviceInfo);
            MV_CHECK(logger, MV_CC_OpenDevice, pImpl->handle);
            MV_CHECK(logger, MV_CC_RegisterImageCallBackEx, pImpl->handle, &HikvisionDriver::Impl::image_callback_ex,
                     this);
            MV_CHECK(logger, MV_CC_StartGrabbing, pImpl->handle);
            break;
        }
    }
    if (pImpl->handle == nullptr) {
        RCLCPP_ERROR(logger, "camera '%s' not found", pImpl->camera_name.c_str());
    }
}

HikvisionDriver::~HikvisionDriver() {
    if (pImpl->handle == nullptr) return;
    auto logger = get_logger();

    MV_CHECK(logger, MV_CC_StopGrabbing, pImpl->handle);
    MV_CHECK(logger, MV_CC_CloseDevice, pImpl->handle);
    MV_CHECK(logger, MV_CC_DestroyHandle, pImpl->handle);
    pImpl->handle = nullptr;
}

}  // namespace hikvision_ros2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hikvision_ros2_driver::HikvisionDriver);