//// License: Apache 2.0. See LICENSE file in root directory.
//// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
//#include <librealsense2/rs.hpp>
//#include <mutex>
//#include <cstring>
//#include <iostream>
//#include <fstream>
//#include <sstream>
//#include <thread>
//#include <chrono>
//#include "imu_handler.h"
//
//int main(int argc, char * argv[]) try
//{
//	// Before running the example, check that a device supporting IMU is connected
//	if (!check_imu_is_supported())
//	{
//		std::cerr << "Device supporting IMU not found";
//		return EXIT_FAILURE;
//	}
//
//	// Declare RealSense pipeline, encapsulating the actual device and sensors
//	rs2::pipeline pipe;
//	// Create a configuration for configuring the pipeline with a non default profile
//	rs2::config cfg;
//
//	// Add streams of gyro and accelerometer to configuration
//	cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
//	cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
//
//	// Start streaming with the given configuration;
//	// Note that since we only allow IMU streams, only single frames are produced
//	pipe.start(cfg, imu_hdl_callback);
//
//	// Main loop
//	while (1)
//	{
//	}
//	// Stop the pipeline
//	pipe.stop();
//
//	return EXIT_SUCCESS;
//}
//catch (const rs2::error & e)
//{
//	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
//	return EXIT_FAILURE;
//}
//catch (const std::exception& e)
//{
//	std::cerr << e.what() << std::endl;
//	return EXIT_FAILURE;
//}
