#ifndef RGB_IMAGE_HDL_H
#define RGB_IMAGE_HDL_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#if CV_VERSION_MAJOR < 4
#pragma message( "OpenCV version < 4" )
#endif

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include "SSDModel.h"
#include "Graphic.h"

using namespace rs2;
using namespace cv;

// Window size detection
#define CFG_DETECTION_WIDTH 1280 //1280 //640
#define CFG_DETECTION_HEIGHT 720 //720 //480
#define CFG_DETECTION_CAMERA_FPS 15
#define WINDOW_DETECTION_SIZE Size(CFG_DETECTION_WIDTH, CFG_DETECTION_HEIGHT)

// Detection parameter
#define CFG_CONF_THRESHOLD  0.3f
#define CFG_NMS_THRESHOLD  0.5f
#define CFG_DETECTION_FPS  10

#define CFG_DETECTION_Y_THRES  720/2
#define CFG_DETECTION_X1_THRES  300
#define CFG_DETECTION_X2_THRES  1000
#define CFG_DETECTION_WIDTH_THRES  30
#define CFG_DETECTION_HEIGHT_THRES  30

const std::string window_detection_name = "Object Detection";

class rgb_image_handler
{
public:
	rgb_image_handler(void);
	// explicit
	rgb_image_handler(rgb_image_handler &&) {}
	// implicit
	rgb_image_handler(const rgb_image_handler&) = default;
	rgb_image_handler& operator=(const rgb_image_handler&) = default;
	~rgb_image_handler();

	void Camera_RGBframe_Handler(Mat image1);

private:
	// Create SSD MobileNet model
	SSDModel ssd_model = SSDModel(CFG_CONF_THRESHOLD, CFG_NMS_THRESHOLD);
	// Create Graphic model which handles images
	Graphic graphic_input = Graphic(ssd_model.getClassNumber(), WINDOW_DETECTION_SIZE);

	// Declare depth colorizer for pretty visualization of data
	rs2::colorizer color_map;
	rs2::frame color_data;

	std::vector<int> classIds;
	std::vector<std::string> classNames;
	std::vector<float> confidences;
	std::vector<cv::Rect> boxes;

	cv::Point po1 = cv::Point(CFG_DETECTION_X1_THRES, CFG_DETECTION_Y_THRES);
    cv::Point po2 = cv::Point(CFG_DETECTION_X2_THRES, CFG_DETECTION_HEIGHT);
    cv::Scalar po_color = cv::Scalar(255, 255, 255);


	uint32_t count_detection_frame = 0;
	bool is_obj_in_safe_area = false;
	uint32_t count_obj_in_safe_area = 0;
	void obj_in_safe_area_hdl(void);

};

#endif
