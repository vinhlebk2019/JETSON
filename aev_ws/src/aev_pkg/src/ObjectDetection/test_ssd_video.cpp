////
//// SDD object detection using OpenCV
////   - Using SSD MobileNet v2 COCO data with TensorFlow
////
//// configration file (.pbtxt) downloaded from below:
//// https://github.com/opencv/opencv_extra/tree/master/testdata/dnn
////
//// SDD MobileNet model file (.pb) downloaded from below:
//// https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md
////
//// Sample source:
//// https://github.com/opencv/opencv/blob/master/samples/dnn/object_detection.cpp
////
//
//#include <iostream>
//#include <fstream>
//#include <sstream>
//#include <thread>
//#include <chrono>
//
//#include <opencv2/dnn.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/core/utils/filesystem.hpp>
//
//#if CV_VERSION_MAJOR < 4
//#pragma message( "OpenCV version < 4" )
//#endif
//
//#include "SSDModel.h"
//#include "Graphic.h"
//
//using namespace cv;
//
///* Check if the input file has been specified properly. */
//// std::string img_file = "/images/street_video1.mp4";
//// std::string img_file = "/images/street_video2.mp4";
//std::string img_file = "C:/Users/Admin/Desktop/CameraIntel/VisualStudioPrj/librealsense-2.49.0/examples/test/images/people_video1.mp4";
//// std::string img_file = "/images/people_video2.mp4";
//
//// Detection parameter
//float CFG_CONF_THRESHOLD = 0.3f;
//float CFG_NMS_THRESHOLD = 0.5f;
//
//#define CFG_DETECTION_FPS  2
//#define detection_frame_Thres (CFG_DETECTION_CAMERA_FPS / CFG_DETECTION_FPS)
//
//std::vector<int> classIds;
//std::vector<std::string> classNames;
//std::vector<float> confidences;
//std::vector<cv::Rect> boxes;
//cv::Mat current_image;
//
//int main(int argc, char** argv)
//{
//	int count_detection_frame = 0;
//
//	// Create queues for sending image to display and to detection
//	std::shared_ptr<MessageQueue<cv::Mat>> image_queue(new MessageQueue<cv::Mat>);
//	std::shared_ptr<MessageQueue<cv::Mat>> detection_queue(new MessageQueue<cv::Mat>);
//	// Create SSD MobileNet model
//	SSDModel ssd_model = SSDModel(CFG_CONF_THRESHOLD, CFG_NMS_THRESHOLD);
//	// Create Graphic model which handles images 
//
//	Graphic graphic_input = Graphic(img_file, ssd_model.getClassNumber());
//	// Set setDetectFramePerSec
//	graphic_input.setDetectFramePerSec(CFG_DETECTION_FPS);
//	// Set shared pointers of queues into objects
//	graphic_input.setImageQueue(image_queue);
//	graphic_input.setDetectionQueue(detection_queue);
//	ssd_model.setDetectionQueue(detection_queue);
//
//	// Launch the readinig thread and the detecting thread
//	graphic_input.thread_for_read();
//	ssd_model.thread_for_detection();
//
//	const int duration = (int)(1000 / graphic_input.getFps());
//	while (cv::waitKey(duration) < 0)
//	{
//		if (image_queue->getTotal() > 0 && count_detection_frame >= image_queue->getTotal())
//		{
//			break;
//		}
//		current_image = image_queue->receive();
//		// Execute the detection once per counts specified by getDetectFreq()
//		if (count_detection_frame % (graphic_input.getDetectFreq()) == 0)
//		{
//			ssd_model.getNextDetection(classIds, classNames, confidences, boxes);
//		}
//		// Plot the result and show the image on window
//		graphic_input.drawResult(current_image, classIds, classNames, confidences, boxes);
//		cv::imshow("Object detection", current_image);
//		++count_detection_frame;
//	}
//
//	std::cout << " --- Object detection finished. Press Enter key to quit.---\n";
//
//	cv::waitKey(0);
//
//	return 0;
//}
