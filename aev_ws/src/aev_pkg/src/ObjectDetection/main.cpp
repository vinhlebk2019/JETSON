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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "aev_pkg/object_detection_msg.h"
 
#include "main.h" 
#include "SSDModel.h"
#include "Graphic.h"
#include "imu_handler.h"
#include "depth_handler.h"
#include "rgb_image_handler.h"

using namespace rs2;
using namespace cv;
using namespace std::chrono;

/************************** MACRO *******************************/
//#define EXPORT_REPORT_DATA_CSV 
#define FPS_TEST_ENABLE

#define COLOR_DATA_ENABLE
//#define DEPTH_DATA_ENABLE
//#define IMU_DATA_ENABLE
/*********************************************************/

using namespace std;
#ifdef EXPORT_REPORT_DATA_CSV
#define FILE_NAME_PREFIX "Report_ObjDetc_"
std::string GetCurrentTimeForFileName()
{
    auto time = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%F_%T"); // ISO 8601 without timezone information.
    auto s = ss.str();
    std::replace(s.begin(), s.end(), ':', '-');
    return s;
}
#endif

ros::Publisher objectDetection_pub;
object_detection_data objectDetection_data;

unsigned int msg_cnt = 0; 
aev_pkg::object_detection_msg Objdetc_Output_msg;
void timer_main_Callback(const ros::TimerEvent& e)
{
	//msg_cnt++;
	//Objdetc_Output_msg.msg_counter = msg_cnt;
	//std::cout << "Object detected pub. \n";
    //objectDetection_pub.publish(Objdetc_Output_msg); 
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ssd");
	ros::NodeHandle ObjectDetection_Node;
	objectDetection_pub = ObjectDetection_Node.advertise<aev_pkg::object_detection_msg>("ObjDetc_Data", 1000);
	//ros::Timer timer1 = ObjectDetection_Node.createTimer(ros::Duration(0.5), timer_main_Callback);

	// Check that a device supporting IMU is connected
	if (!check_imu_is_supported())
	{
		std::cerr << "Device supporting REALSENSE CAMERA not found \n";
		return EXIT_FAILURE;
	}
	else
	{
		std::cout << "Device found" << endl;
	}

	#ifdef COLOR_DATA_ENABLE
	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe_rgb;
	// Config to size of rgb image from camera
	config cfg_rgb;
	cfg_rgb.enable_stream(RS2_STREAM_COLOR, CFG_DETECTION_WIDTH, CFG_DETECTION_HEIGHT, RS2_FORMAT_BGR8, CFG_DETECTION_CAMERA_FPS);
	pipe_rgb.start(cfg_rgb);
	rgb_image_handler rgb_image_handler1;
	rs2::frameset data_rgb;
	#endif

	#ifdef DEPTH_DATA_ENABLE
	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe_depth;;
	// Config to size of depth data from camera
	config cfg_depth;
	cfg_depth.enable_stream(RS2_STREAM_DEPTH, CFG_DEPTH_WIDTH, CFG_DEPTH_HEIGHT, RS2_FORMAT_Z16, CFG_DEPTH_CAMERA_FPS);
	pipe_depth.start(cfg_depth);
	depth_handler depth_handler1;
	#endif

	#ifdef IMU_DATA_ENABLE
	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe_imu;
	// Config to get IMU from camera
	config cfg_imu;
	cfg_imu.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
	cfg_imu.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
	pipe_imu.start(cfg_imu, imu_hdl_callback);
	#endif

	#ifdef FPS_TEST_ENABLE
	uint32_t count_data = 0;
	auto start = high_resolution_clock::now();
	auto stop = high_resolution_clock::now();
	milliseconds duration = duration_cast<milliseconds>(stop - start);
	#endif

	#ifdef EXPORT_REPORT_DATA_CSV
	ofstream MyExcelFile;
	std::string fileName = "/home/ubuntu/aev/reports/object_detection/";
	fileName = fileName + FILE_NAME_PREFIX + GetCurrentTimeForFileName() + ".csv";
	MyExcelFile.open(fileName);
	MyExcelFile << "Frame Num. ,Time(ms), FPS, IsObjInFrame, IsObjInSafeArea, YawRate" << endl;
	#endif

    while((cv::waitKey(1) < 0) && (ros::ok()))
    {
		
		/* RGB frame handler */
		#ifdef COLOR_DATA_ENABLE
		data_rgb = pipe_rgb.wait_for_frames(); // Wait for next set of frames from the camera
		rgb_image_handler1.Camera_RGBframe_Handler(data_rgb);
		#endif

		/* Depth frame handler */
		#ifdef DEPTH_DATA_ENABLE
		//depth_handler1.Camera_Depth_Handler(data_rgb);
		#endif

		if (objectDetection_data.isObjInSafeArea == true)
		{
			//std::cout << "Object detected in safe area. " << msg_cnt << "\n";
			Objdetc_Output_msg.isObject = true;
		}
		else
		{
			Objdetc_Output_msg.isObject = false;
		}
		msg_cnt++;
		Objdetc_Output_msg.msg_counter = msg_cnt;
		Objdetc_Output_msg.yaw_rate = objectDetection_data.yaw_rate;
    	objectDetection_pub.publish(Objdetc_Output_msg);

		#ifdef FPS_TEST_ENABLE
		count_data++;
		if (count_data == 0)
		{
			start = high_resolution_clock::now();
		}
		stop = high_resolution_clock::now();
		duration = duration_cast<milliseconds>(stop - start);
		std::cout << "Num of data_rgb = " << count_data << " in " << duration.count() << " ms. " << "FPS = " << count_data/(duration.count()/1000.0f) << "\r\n";
		std::cout << "Yaw rate = " << objectDetection_data.yaw_rate << "\r\n";
		#endif

		#ifdef EXPORT_REPORT_DATA_CSV
		std::string report_data_str;
		report_data_str = std::to_string(int(msg_cnt)) + "," 
		#ifdef FPS_TEST_ENABLE
		+ std::to_string(int(duration.count())) + "," + std::to_string(float(count_data/(duration.count()/1000.0f))) + ","
		#else
		+ "0" + "," + "0" + ","
		#endif
		+ std::to_string(int(objectDetection_data.isObjInFrame)) + "," 
		+ std::to_string(int(objectDetection_data.isObjInSafeArea)) + "," 
		+ std::to_string(float(objectDetection_data.yaw_rate));
		MyExcelFile << report_data_str << endl;
    	#endif

		//Timer_Data_Callback();
    }

    std::terminate();
	// Stop the pipeline
	#ifdef COLOR_DATA_ENABLE
	pipe_rgb.stop();
	#endif
	#ifdef DEPTH_DATA_ENABLE 
	pipe_depth.stop();
	#endif
	#ifdef IMU_DATA_ENABLE
	pipe_imu.stop();
	#endif

	ros::shutdown();

#ifdef FPS_TEST_ENABLE
	std::cout << "\n";
	stop = high_resolution_clock::now();
	duration = duration_cast<milliseconds>(stop - start);
	std::cout << "Num of data_rgb " << count_data << " in " << duration.count() << " ms \r\n";
	std::cout << "Actual FPS: " << count_data/(duration.count()/1000.0f) << " \r\n";
#endif

    std::cout << " --- App finished. Quit.---\n";

    return 0;
}
