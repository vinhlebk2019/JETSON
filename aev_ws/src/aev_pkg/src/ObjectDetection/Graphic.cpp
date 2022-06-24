#include <iostream>
#include <random>
#include <iomanip>
#include <thread>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "Graphic.h"


// Constructor.
// Parameter: path to the image file, stored to the private attribute
//            number of classes, used to generate colors for classes

Graphic::Graphic(int class_num, cv::Size image_size)
{
	setClassColor(class_num);
	window_size = image_size;
	_fps = 15;
}

Graphic::Graphic(int class_num)
{
	setClassColor(class_num);
	_fps = 15;
	window_size = cv::Size(640, 480);
}


Graphic::~Graphic()
{
    
}

cv::Size Graphic::getWindowSize()
{
    return window_size;
}


float Graphic::getFps()
{
    return _fps;
}

// Draw the result of detection on image(reference parameter)
void Graphic::drawResult(cv::Mat &image, 
                         const std::vector<int> &classIds, 
                         const std::vector<std::string> &classNames,
                         const std::vector<float> &confidences,
                         const std::vector<cv::Rect> &boxes)
{
    for(size_t i = 0; i < classIds.size(); i++)
    {
        if((classNames[i] == "person"       ) 
            || (classNames[i] == "car"      )
            || (classNames[i] == "truck"    )
            || (classNames[i] == "bicycle"  )
            || (classNames[i] == "bus"      ))
        {
            if (confidences[i]*100.0 > 60.0)
            {
                // Box
                cv::Point p1 = cv::Point(boxes[i].x, boxes[i].y);
                cv::Point p2 = cv::Point(boxes[i].x + boxes[i].width, boxes[i].y + boxes[i].height);
                CV_Assert(classIds[i] < class_color.size());
                cv::rectangle(image, p1, p2, class_color[classIds[i]], 2);

                // Label
                std::ostringstream streamObj;
                streamObj << std::fixed << std::setprecision(2) << confidences[i]*100.0;
                std::string label = classNames[i]  + " : " + streamObj.str();

                int baseLine;
                cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);

                int top = std::max(boxes[i].y, labelSize.height);
                cv::Point lp1 = cv::Point(boxes[i].x, top - labelSize.height-2);
                cv::Point lp2 = cv::Point(boxes[i].x + labelSize.width, top);
                cv::rectangle(image, lp1, lp2, class_color[classIds[i]], cv::FILLED);
                cv::putText(image, label, cv::Point(boxes[i].x, top-1), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(), 1);
            }
        }
    }
}

// Assign a color to each class
void Graphic::setClassColor(int class_num)
{
    std::mt19937 random_engine(2019); // Use a fixed seed to get same colors always
    std::uniform_int_distribution<int> distribution(0, 255);

    for(int i = 0; i < class_num; ++i)
    {
        cv::Scalar color = cv::Scalar(distribution(random_engine),
                                      distribution(random_engine),
                                      distribution(random_engine));
        class_color.push_back(color);
    }
}
