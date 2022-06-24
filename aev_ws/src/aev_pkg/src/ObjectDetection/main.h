#ifndef MAIN_H
#define MAIN_H

#include <iostream>

typedef struct
{
	uint64_t msg_counter;

	bool isObjInSafeArea;
	bool isObjInFrame;

	double yaw_rate;
	bool newYawRate_flag;

} object_detection_data;

extern object_detection_data objectDetection_data;

#endif