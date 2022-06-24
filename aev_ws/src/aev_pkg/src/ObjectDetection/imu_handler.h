#ifndef IMU_HDL_H
#define IMU_HDL_H

#include <librealsense2/rs.hpp>
#include <mutex>
#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>
#include "rotation_estimator.h"
#include "main.h"

//#define IUM_DATA_PRINT_ENABLE

bool check_imu_is_supported()
{
	bool found_gyro = false;
	bool found_accel = false;
	rs2::context ctx;
	for (auto dev : ctx.query_devices())
	{
		// The same device should support gyro and accel
		found_gyro = false;
		found_accel = false;
		for (auto sensor : dev.query_sensors())
		{
			for (auto profile : sensor.get_stream_profiles())
			{
				if (profile.stream_type() == RS2_STREAM_GYRO)
					found_gyro = true;

				if (profile.stream_type() == RS2_STREAM_ACCEL)
					found_accel = true;
			}
		}
		if (found_gyro && found_accel)
			break;
	}
	return found_gyro && found_accel;
}

// Declare object that handles camera pose calculations
rotation_estimator algo;
rs2_vector gyro_data;
rs2_vector accel_data;
float3 theta;
uint32_t count_data = 0;
auto imu_hdl_callback = [&](rs2::frame frame)
{
	// Cast the frame that arrived to motion frame
	auto motion = frame.as<rs2::motion_frame>();
	// If casting succeeded and the arrived frame is from gyro stream
	if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
	{
		// Get the timestamp of the current frame
		double ts = motion.get_timestamp();
		// Get gyro measures
		gyro_data = motion.get_motion_data();
		// Call function that computes the angle of motion based on the retrieved measures
		algo.process_gyro(gyro_data, ts);
	}
	// If casting succeeded and the arrived frame is from accelerometer stream
	if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
	{
		// Get accelerometer measures
		accel_data = motion.get_motion_data();
		// Call function that computes the angle of motion based on the retrieved measures
		algo.process_accel(accel_data);
	}

	count_data++;

	theta = algo.get_theta();

	#ifdef IUM_DATA_PRINT_ENABLE
	char buffer[256];
	sprintf(buffer, "Data num %5d | Accel: %10.6f  %10.6f %10.6f | Gyro: %10.6f  %10.6f %10.6f | Theta %10.6f %10.6f %10.6f\r\n",
		count_data, accel_data.x, accel_data.y, accel_data.z, gyro_data.x, gyro_data.y, gyro_data.z, theta.x, theta.y, theta.z);
	std::cout << buffer;
	#endif

	objectDetection_data.yaw_rate = gyro_data.y;
	objectDetection_data.newYawRate_flag = true;
};

#endif
