#ifndef DEPTH_HDL_H
#define DEPTH_HDL_H

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

using namespace rs2;
using namespace cv;

#define CFG_DEPTH_WIDTH 640
#define CFG_DEPTH_HEIGHT 480
#define CFG_DEPTH_CAMERA_FPS 15
#define WINDOW_DEPTH_SIZE  Size(CFG_DEPTH_WIDTH, CFG_DEPTH_HEIGHT)

const std::string window_depth_name = "Display Depth data";

class depth_handler
{
public:
	depth_handler(void);
	// explicit
	depth_handler(depth_handler &&) {}
	// implicit
	depth_handler(const depth_handler&) = default;
	depth_handler& operator=(const depth_handler&) = default;
	~depth_handler();

	void Camera_Depth_Handler(rs2::frameset _frameset);

private:

	rs2::frame depth_data;
	rs2::colorizer color_map;
	float dist_to_center;
};

#endif
