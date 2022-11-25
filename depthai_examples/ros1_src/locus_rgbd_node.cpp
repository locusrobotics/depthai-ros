/*
 * locus_rgbd_node.cpp
 *
 *  Created on: 23/11/2022
 *      Author: cribeiromendes
 */

// #include "utility.hpp"
#include <camera_info_manager/camera_info_manager.h>

#include "sensor_msgs/Image.h"

// Inludes common necessary includes for development using depthai library
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

#include "depthai/depthai.hpp"

struct OakDProWConfig {
    enum EImuMode : int { COPY, LINEAR_INTER_GYRO, LINEAR_INTER_ACCEL, MAX_MODES };
    std::string mx_id{""};
    bool usb_2_mode{false};
    bool poe_mode{false};

    // Frame wise
    std::string resource_folder{""};
    std::string tf_prefix{"oak"};
    /**
     * @brief Computes and combines disparities in both L-R and R-L directions, and combine them.
     *
     * For better occlusion handling, discarding invalid disparity values
     */
    bool lr_check{true};
    /**
     * @brief Threshold for left-right, right-left disparity map combine, 0..255
     */
    int lr_check_threshold{5};

    /**
     * @brief Computes disparity with sub-pixel interpolation (3 fractional bits by default).
     *
     * Suitable for long range. Currently incompatible with extended disparity
     * Improves the precision and is especially useful for long range measurements.
     * It also helps for better estimating surface normals.
     */
    bool subpixel_interpolation{true};
    /**
     * @brief Disparity range increased from 0-95 to 0-190, combined from full resolution and downscaled images.
     *
     * Suitable for short range objects. Currently incompatible with sub-pixel disparity
     * Extended disparity mode allows detecting closer distance objects for the given baseline.
     * This increases the maximum disparity search from 96 to 191, meaning the range is now: [0..190].
     * So this cuts the minimum perceivable distance in half, given that the minimum distance is now
     * focal_length * base_line_dist / 190 instead of focal_length * base_line_dist / 95
     */
    bool extedend_disparity{false};
    /**
     * @brief
     */
    bool aligned_depth{true};
    /**
     * @brief Rectify image
     */
    bool rectify{true};
    /**
     *
     */
    int stereo_fps{15};
    int rgb_fps{15};
    /**
     * @brief Confidence threshold for disparity calculation [0..255]
     */
    int confidence{200};

    std::string stereo_resolution_str{""};
    dai::node::MonoCamera::Properties::SensorResolution stereo_resolution;
    std::string rgb_resolution_str{""};
    dai::node::ColorCamera::Properties::SensorResolution rgb_resolution;

    bool manual_exposure{false};
    /**
     * @brief  Exposure time in microseconds
     */
    int exposure_time{20000};
    /**
     * @brief sensitivityIso Sensitivity as ISO value, usual range 100..1600
     */
    int sens_iso{800};
    /**
     * @brief rgb scale num
     *
     * rgbWidth = rgbWidth * rgbScaleNumerator / rgbScaleDinominator;
     * rgbHeight = rgbHeight * rgbScaleNumerator / rgbScaleDinominator;
     * camRgb->setIspScale(rgbScaleNumerator, rgbScaleDinominator);
     *
     */
    int rgb_scale_num{2};
    int rgb_scale_den{3};

    int preview_width{416};
    int preview_height{416};

    bool enable_dot_projector{false};
    /**
     * @brief The brightness of the IR Laser Dot Projector. Limits: up to 765mA at 30% duty cycle, up to 1200mA at 6% duty cycle.
     * The duty cycle is controlled by `left` camera STROBE, aligned to start of exposure.
     * The emitter is turned off by default
     */
    float dot_projector_ma{200.0f};
    /**
     * @brief
     */
    bool enable_flood_light{false};
    /**
     * @brief The brightness of the IR Flood Light. Limits: up to 1500mA at 30% duty cycle.
     * The duty cycle is controlled by the `left` camera STROBE, aligned to start of exposure.
     * If the dot projector is also enabled, its lower duty cycle limits take precedence.
     * The emitter is turned off by default
     * Current in mA that will determine brightness, 0 or negative to turn off
     **/
    float flood_light_ma{200.0f};
    /**
     * @brief
     */
    EImuMode imu_mode{EImuMode::COPY};
    int imu_frequency{0};

    int stereo_width{0};
    int stereo_height{0};
    int rgb_width{0};
    int rgb_height{0};
};

class OakDTest {
   public:
    OakDTest(ros::NodeHandle nh) {
        std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();
        this->readConfig(nh);
        this->createPipeline();

        device_ = std::make_shared<dai::Device>(*this->pipeline_);

        auto calibrationHandler = device_->readCalibration();

        auto stereoQueue = device_->getOutputQueue("depth", 30, false);
        auto rgbQueue = device_->getOutputQueue("rgb", 30, false);
        // auto previewQueue = device_->getOutputQueue("rgb", 30, false);
        dai::rosBridge::ImageConverter rgbConverter(cfg_.tf_prefix + "_rgb_camera_optical_frame", false);
        dai::rosBridge::ImageConverter depthConverter(cfg_.tf_prefix + "_right_camera_optical_frame", true);
        auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, cfg_.rgb_width, cfg_.rgb_height);
        auto imgQueue = device_->getOutputQueue("rgb", 30, false);
        dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(
            imgQueue,
            nh,
            std::string("color/image"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
            30,
            rgbCameraInfo,
            "color");
        rgbPublish.addPublisherCallback();

        dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(
            stereoQueue,
            nh,
            std::string("stereo/depth"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                      &depthConverter,  // since the converter has the same frame name
                                        // and image type is also same we can reuse it
                      std::placeholders::_1,
                      std::placeholders::_2),
            30,
            rgbCameraInfo,
            "stereo");
        depthPublish.addPublisherCallback();
    }

    void createPipeline() {
        pipeline_ = std::make_shared<dai::Pipeline>();
        // Let us configure first stereo
        // IMU

        auto monoLeft = pipeline_->create<dai::node::MonoCamera>();
        auto monoRight = pipeline_->create<dai::node::MonoCamera>();
        auto stereo = pipeline_->create<dai::node::StereoDepth>();
        auto xoutDepth = pipeline_->create<dai::node::XLinkOut>();
        xoutDepth->setStreamName("depth");
        // MonoCamera
        monoLeft->setResolution(cfg_.stereo_resolution);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        monoRight->setResolution(cfg_.stereo_resolution);
        monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
        // StereoDepth
        stereo->initialConfig.setConfidenceThreshold(cfg_.confidence);
        stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
        stereo->initialConfig.setLeftRightCheckThreshold(cfg_.lr_check_threshold);

        stereo->setLeftRightCheck(cfg_.lr_check);
        stereo->setExtendedDisparity(cfg_.extedend_disparity);
        stereo->setSubpixel(cfg_.subpixel_interpolation);
        if(cfg_.aligned_depth) {
            stereo->setDepthAlign(dai::CameraBoardSocket::RGB);
        }

        // Color camers steream setup -------->
        auto colorCam = pipeline_->create<dai::node::ColorCamera>();
        auto xlinkOut = pipeline_->create<dai::node::XLinkOut>();

        xlinkOut->setStreamName("rgb");
        colorCam->setBoardSocket(dai::CameraBoardSocket::RGB);
        colorCam->setFps(cfg_.rgb_fps);

        colorCam->setResolution(cfg_.rgb_resolution);
        colorCam->setInterleaved(true);
        colorCam->video.link(xlinkOut->input);

        if(cfg_.aligned_depth) {
            colorCam->setIspScale(cfg_.rgb_scale_num, cfg_.rgb_scale_den);
            // Link plugins CAM -> XLINK
            colorCam->isp.link(xlinkOut->input);
        }
        // Stereo imges // Stereo imges
        auto xoutLeft = pipeline_->create<dai::node::XLinkOut>();
        auto xoutRight = pipeline_->create<dai::node::XLinkOut>();
        // XLinkOut
        xoutLeft->setStreamName("left");
        xoutRight->setStreamName("right");
        if (cfg_.rectify)
        {
            stereo->rectifiedLeft.link(xoutLeft->input);
            stereo->rectifiedRight.link(xoutRight->input);
        }
        else
        {
            stereo->syncedLeft.link(xoutLeft->input);
            stereo->syncedRight.link(xoutRight->input);
        }
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);
        //if(cfg_.depth) {
            stereo->depth.link(xoutDepth->input);
        //} else {
           // stereo->disparity.link(xoutDepth->input);
        //}
    }

    void readConfig(ros::NodeHandle nh) {
        nh.param("resource_folder", cfg_.resource_folder, cfg_.resource_folder);
        nh.param("tf_prefix", cfg_.tf_prefix, cfg_.tf_prefix);
        nh.param("lr_check", cfg_.lr_check, cfg_.lr_check);
        nh.param("lr_check_threshold", cfg_.lr_check_threshold, cfg_.lr_check_threshold);
        nh.param("usb_2_mode", cfg_.usb_2_mode, cfg_.usb_2_mode);
        nh.param("poe_mode", cfg_.poe_mode, cfg_.poe_mode);
        nh.param("subpixel_interpolation", cfg_.subpixel_interpolation, cfg_.subpixel_interpolation);
        nh.param("extedend_disparity", cfg_.extedend_disparity, cfg_.extedend_disparity);
        nh.param("confidence", cfg_.confidence, cfg_.confidence);
        nh.param("rectify", cfg_.rectify, cfg_.rectify);
        nh.param("manual_exposure", cfg_.manual_exposure, cfg_.manual_exposure);
        nh.param("exposure_time", cfg_.exposure_time, cfg_.exposure_time);
        nh.param("sens_iso", cfg_.sens_iso, cfg_.sens_iso);
        nh.param("rgb_scale_num", cfg_.rgb_scale_num, cfg_.rgb_scale_num);
        nh.param("rgb_scale_den", cfg_.rgb_scale_den, cfg_.rgb_scale_den);
        nh.param("preview_width", cfg_.preview_width, cfg_.preview_width);
        nh.param("preview_height", cfg_.preview_height, cfg_.preview_height);
        nh.param("aligned_depth", cfg_.aligned_depth, cfg_.aligned_depth);
        nh.param("stereo_fps", cfg_.stereo_fps, cfg_.stereo_fps);
        nh.param("enable_dot_projector", cfg_.enable_dot_projector, cfg_.enable_dot_projector);
        nh.param("dot_projector_ma", cfg_.dot_projector_ma, cfg_.dot_projector_ma);
        nh.param("enable_flood_light", cfg_.enable_flood_light, cfg_.enable_flood_light);
        nh.param("flood_light_ma", cfg_.flood_light_ma, cfg_.flood_light_ma);
        int mode = static_cast<int>(cfg_.imu_mode);
        nh.param("imu_mode", mode, mode);
        cfg_.imu_mode = static_cast<OakDProWConfig::EImuMode>(mode);
        nh.param("imu_frequency", cfg_.imu_frequency, cfg_.imu_frequency);

        // REsolutions

        nh.param("stereo_resolution", cfg_.stereo_resolution_str, cfg_.stereo_resolution_str);
        nh.param("rgb_resolution", cfg_.rgb_resolution_str, cfg_.rgb_resolution_str);
        //

        if(cfg_.stereo_resolution_str == "720p") {
            cfg_.stereo_resolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
            cfg_.stereo_width = 1280;
            cfg_.stereo_height = 720;
        } else if(cfg_.stereo_resolution_str == "400p") {
            cfg_.stereo_resolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
            cfg_.stereo_width = 640;
            cfg_.stereo_height = 400;
        } else if(cfg_.stereo_resolution_str == "800p") {
            cfg_.stereo_resolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
            cfg_.stereo_width = 1280;
            cfg_.stereo_height = 800;
        } else if(cfg_.stereo_resolution_str == "480p") {
            cfg_.stereo_resolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
            cfg_.stereo_width = 640;
            cfg_.stereo_height = 480;
        } else {
            ROS_ERROR("Invalid parameter. -> monoResolution: %s", cfg_.stereo_resolution_str.c_str());
            throw std::runtime_error("Invalid mono camera resolution.");
        }
        if(cfg_.rgb_resolution_str == "1080p") {
            cfg_.rgb_resolution = dai::node::ColorCamera::Properties::SensorResolution::THE_1080_P;
            cfg_.rgb_width = 1920;
            cfg_.rgb_height = 1080;
        } else if(cfg_.rgb_resolution_str == "4K") {
            cfg_.rgb_resolution = dai::node::ColorCamera::Properties::SensorResolution::THE_4_K;
            cfg_.rgb_width = 3840;
            cfg_.rgb_height = 2160;
        } else if(cfg_.rgb_resolution_str == "12MP") {
            cfg_.rgb_resolution = dai::node::ColorCamera::Properties::SensorResolution::THE_12_MP;
            cfg_.rgb_width = 4056;
            cfg_.rgb_height = 3040;
        } else if(cfg_.rgb_resolution_str == "13MP") {
            cfg_.rgb_resolution = dai::node::ColorCamera::Properties::SensorResolution::THE_13_MP;
            cfg_.rgb_width = 4208;
            cfg_.rgb_height = 3120;
        } else {
            ROS_ERROR("Invalid parameter. -> rgbResolution: %s", cfg_.rgb_resolution_str.c_str());
            throw std::runtime_error("Invalid color camera resolution.");
        }
        if(cfg_.aligned_depth) {
            cfg_.rgb_width = cfg_.rgb_width * cfg_.rgb_scale_num / cfg_.rgb_scale_den;
            cfg_.rgb_height = cfg_.rgb_height * cfg_.rgb_scale_num / cfg_.rgb_scale_den;
            if(cfg_.rgb_width > cfg_.rgb_width || cfg_.rgb_height > cfg_.stereo_height) {
                ROS_WARN_STREAM(
                    "RGB Camera resolution is higher than the configured stereo resolution. Upscaling the stereo depth/disparity to match RGB camera "
                    "resolution.");
            } else if(cfg_.rgb_width > cfg_.rgb_width || cfg_.rgb_height > cfg_.stereo_height) {
                ROS_WARN_STREAM(
                    "RGB Camera resolution is higher than the configured stereo resolution. Downscaling the stereo depth/disparity to match RGB camera "
                    "resolution.");
            }

            if(cfg_.rgb_width % 16 != 0) {
                if(cfg_.rgb_resolution == dai::node::ColorCamera::Properties::SensorResolution::THE_12_MP) {
                    ROS_ERROR_STREAM("RGB Camera width should be multiple of 16. Please choose a different scaling factor."
                                     << std::endl
                                     << "Here are the scaling options that works for 12MP with depth aligned" << std::endl
                                     << "4056 x 3040 *  2/13 -->  624 x  468" << std::endl
                                     << "4056 x 3040 *  2/39 -->  208 x  156" << std::endl
                                     << "4056 x 3040 *  2/51 -->  160 x  120" << std::endl
                                     << "4056 x 3040 *  4/13 --> 1248 x  936" << std::endl
                                     << "4056 x 3040 *  4/26 -->  624 x  468" << std::endl
                                     << "4056 x 3040 *  4/29 -->  560 x  420" << std::endl
                                     << "4056 x 3040 *  4/35 -->  464 x  348" << std::endl
                                     << "4056 x 3040 *  4/39 -->  416 x  312" << std::endl
                                     << "4056 x 3040 *  6/13 --> 1872 x 1404" << std::endl
                                     << "4056 x 3040 *  6/39 -->  624 x  468" << std::endl
                                     << "4056 x 3040 *  7/25 --> 1136 x  852" << std::endl
                                     << "4056 x 3040 *  8/26 --> 1248 x  936" << std::endl
                                     << "4056 x 3040 *  8/39 -->  832 x  624" << std::endl
                                     << "4056 x 3040 *  8/52 -->  624 x  468" << std::endl
                                     << "4056 x 3040 *  8/58 -->  560 x  420" << std::endl
                                     << "4056 x 3040 * 10/39 --> 1040 x  780" << std::endl
                                     << "4056 x 3040 * 10/59 -->  688 x  516" << std::endl
                                     << "4056 x 3040 * 12/17 --> 2864 x 2146" << std::endl
                                     << "4056 x 3040 * 12/26 --> 1872 x 1404" << std::endl
                                     << "4056 x 3040 * 12/39 --> 1248 x  936" << std::endl
                                     << "4056 x 3040 * 13/16 --> 3296 x 2470" << std::endl
                                     << "4056 x 3040 * 14/39 --> 1456 x 1092" << std::endl
                                     << "4056 x 3040 * 14/50 --> 1136 x  852" << std::endl
                                     << "4056 x 3040 * 14/53 --> 1072 x  804" << std::endl
                                     << "4056 x 3040 * 16/39 --> 1664 x 1248" << std::endl
                                     << "4056 x 3040 * 16/52 --> 1248 x  936" << std::endl);

                } else {
                    ROS_ERROR_STREAM("RGB Camera width should be multiple of 16. Please choose a different scaling factor.");
                }
                throw std::runtime_error("Adjust RGB Camera scaling.");
            }
        }
    }
    OakDProWConfig cfg_;
    std::shared_ptr<dai::Device> device_;
    std::shared_ptr<dai::Pipeline> pipeline_;
    std::shared_ptr<dai::DataOutputQueue> pipe_out_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "locus_rgb_node");
    ros::NodeHandle pnh("~");
    OakDTest node(pnh);
    ros::spin();

    return 0;
}
