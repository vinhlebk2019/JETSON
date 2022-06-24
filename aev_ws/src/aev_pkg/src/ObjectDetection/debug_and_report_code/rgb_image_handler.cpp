#include "rgb_image_handler.h"
#include "main.h"

#define CFG_DETECTION_ENABLE  (CFG_DETECTION_CAMERA_FPS / CFG_DETECTION_FPS)

// Create queues for sending image to display and to detection
//std::shared_ptr<MessageQueue<cv::Mat>> image_queue(new MessageQueue<cv::Mat>);
std::shared_ptr<MessageQueue<cv::Mat>> detection_queue(new MessageQueue<cv::Mat>);

rgb_image_handler::rgb_image_handler(void)
{
	ssd_model.setDetectionQueue(detection_queue);
	// Launch the readinig thread and the detecting thread
	ssd_model.threadDetectionStart();
}

rgb_image_handler::~rgb_image_handler()
{

}

void rgb_image_handler::obj_in_safe_area_hdl(void)
{
	count_obj_in_safe_area++;

	if (count_obj_in_safe_area < 2)
	{
		// Not actual an object
	}
	else
	{
		//std::cout << "Object detected in safe area. " << count_obj_in_safe_area << "\n";
		objectDetection_data.isObjInSafeArea = true;
	}
}

void rgb_image_handler::Camera_RGBframe_Handler(Mat image1)
{
	// color_data = _frameset.get_color_frame().apply_filter(color_map);
	// Create OpenCV matrix of size (w,h) from the colorized depth data
	// Mat image1(WINDOW_DETECTION_SIZE, CV_8UC3, (void*)color_data.get_data(), Mat::AUTO_STEP);

	detection_queue->send(std::move(cv::Mat(image1)));

	if (ssd_model.isNewResult())
	{
		// Get object detection result
		ssd_model.getNextDetection(classIds, classNames, confidences, boxes);

		is_obj_in_safe_area = false;
		for(size_t i = 0; i < classIds.size(); i++)
	    {
	    	if((classNames[i] == "person"		)) // person
	    	//|| (classNames[i] == "car"	)
	    	//|| (classNames[i] == "truck"	)
	    	//|| (classNames[i] == "bus"		))
	    	{
	    		objectDetection_data.isObjInFrame = true;
	    		if (confidences[i]*100.0 > 60.0)
	    		{
	    			// Box
		        	cv::Point p1 = cv::Point(boxes[i].x, boxes[i].y);
		        	cv::Point p2 = cv::Point(boxes[i].x + boxes[i].width, boxes[i].y + boxes[i].height);
	    			
	    			if ((boxes[i].height > CFG_DETECTION_HEIGHT_THRES) && (boxes[i].width > CFG_DETECTION_WIDTH_THRES))
	    			{
		    			if((boxes[i].y + boxes[i].height) > CFG_DETECTION_Y_THRES)
		    			{
		    				if ((boxes[i].x > CFG_DETECTION_X2_THRES) || ((boxes[i].x + boxes[i].width) < CFG_DETECTION_X1_THRES))
		    				{
		    					// There is no object detected in safe area.
		    				}
		    				else
		    				{

		    					is_obj_in_safe_area = true;
		    				}
		    			}
	    			}

	    		}
	    	} 
	    	else
	    	{
	    		objectDetection_data.isObjInFrame = false;
	    	}
	    }	
	}

	if (is_obj_in_safe_area)
	{
		obj_in_safe_area_hdl();
	}
	else
	{
		count_obj_in_safe_area = 0;
		objectDetection_data.isObjInSafeArea = false;
	}
	
	// Update the window with new data
	count_detection_frame++;

	//graphic_input.drawResult(image1, classIds, classNames, confidences, boxes);
	// cv::rectangle(image1, po1, po2, po_color, 2);
	//resize(image1, image1, Size(400, 400));
	//imshow("Display frame", image1);
}
