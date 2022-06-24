#include "depth_handler.h"

depth_handler::depth_handler(void)
{
	
}

depth_handler::~depth_handler()
{

}

void depth_handler::Camera_Depth_Handler(rs2::frameset _frameset)
{
	// Try to get a frame of a depth image
	rs2::depth_frame depthFrame = _frameset.get_depth_frame();
	// Get the depth frame's dimensions
	auto w = depthFrame.get_width();
	auto h = depthFrame.get_height();
	// Query the distance from the camera to the object in the center of the image
	dist_to_center = depthFrame.get_distance(w / 2, h / 2);
	//std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";
	// Create OpenCV matrix of size (w,h) from the colorized depth data
	depth_data = _frameset.get_depth_frame().apply_filter(color_map);
	// Plot the image
	Mat image(WINDOW_DEPTH_SIZE, CV_8UC3, (void*)depth_data.get_data(), Mat::AUTO_STEP);
	imshow(window_depth_name, image);
}
