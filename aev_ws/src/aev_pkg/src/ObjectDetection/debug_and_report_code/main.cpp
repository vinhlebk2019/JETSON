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

#define EXPORT_REPORT_DATA_CSV
#define FPS_TEST_ENABLE

#define COLOR_DATA_ENABLE
//#define DEPTH_DATA_ENABLE
#define IMU_DATA_ENABLE

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

//cv::String path("/home/jetson/aev/aev_ws/src/aev_pkg/src/ObjectDetection/images/data/person/images/*.jpg");
cv::String path("/home/jetson/aev/aev_ws/src/aev_pkg/src/ObjectDetection/images/data/perdestrian/*.jpg"); 
vector<cv::String> fn;
vector<cv::Mat> data;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ssd");
	ros::NodeHandle ObjectDetection_Node;
	objectDetection_pub = ObjectDetection_Node.advertise<aev_pkg::object_detection_msg>("ObjDetc_Data", 1000);
	//ros::Timer timer1 = ObjectDetection_Node.createTimer(ros::Duration(0.5), timer_main_Callback);

	cv::glob(path,fn,true); // recurse

	rgb_image_handler rgb_image_handler1;

	#ifdef FPS_TEST_ENABLE
	uint32_t count_data = 0;
	auto start = high_resolution_clock::now();
	auto stop = high_resolution_clock::now();
	milliseconds duration = duration_cast<milliseconds>(stop - start);
	#endif

	#ifdef EXPORT_REPORT_DATA_CSV
	ofstream MyExcelFile;
	std::string fileName = "/home/jetson/aev/reports/object_detection/";
	fileName = fileName + FILE_NAME_PREFIX + GetCurrentTimeForFileName() + ".csv";
	MyExcelFile.open(fileName);
	MyExcelFile << "Frame Num. ,Time(ms), IsObjInFrame, IsObjInSafeArea" << endl;
	#endif

	uint16_t k=0;

    while((cv::waitKey(1) < 0) && (ros::ok()) && k<fn.size())
    {
    	for (k=0; k<fn.size(); ++k)
		{
		    cv::Mat im = cv::imread(fn[k]);
		    if (im.empty()) continue; //only proceed if sucsessful
		    
		    // do some preprocessing

			/* RGB Show */
			#ifdef COLOR_DATA_ENABLE
			rgb_image_handler1.Camera_RGBframe_Handler(im);
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
			stop = high_resolution_clock::now();
			duration = duration_cast<milliseconds>(stop - start);
			std::cout << "Num of data_rgb " << count_data << " in " << duration.count() << " ms \r";
			#endif

			#ifdef EXPORT_REPORT_DATA_CSV
	    	std::string report_data_str;
	    	report_data_str = std::to_string(int(msg_cnt)) + "," 
	    		#ifdef FPS_TEST_ENABLE
	    		+ std::to_string(int(duration.count())) + "," 
	    		#else
	    		+ "0" + "," 
	    		#endif
	    		+ std::to_string(int(objectDetection_data.isObjInFrame)) + "," 
	    		+ std::to_string(int(objectDetection_data.isObjInSafeArea));
	    	MyExcelFile << report_data_str << endl;
	    	#endif

	    	std::this_thread::sleep_for(std::chrono::milliseconds(50));
			data.push_back(im);
		}
    }

	ros::shutdown();

#ifdef FPS_TEST_ENABLE
	std::cout << "\n";
	stop = high_resolution_clock::now();
	duration = duration_cast<milliseconds>(stop - start);
	std::cout << "Num of data_rgb " << count_data << " in " << duration.count() << " ms \r\n";
	std::cout << "Actual FPS: " << count_data/(duration.count()/1000) << " \r\n";
#endif

    std::cout << " --- App finished. Quit.---\n";

    return 0;
}
