#ifndef _GRAPHIC_H
#define _GRAPHIC_H

#include "SSDModel.h"
#include "MessageQueue.h"

class Graphic
{
  public:
    Graphic(int class_num, cv::Size image_size);
	Graphic(int class_num);
	// explicit
	Graphic(Graphic &&) {}
	// implicit
	Graphic(const Graphic&) = default;
	Graphic& operator=(const Graphic&) = default;
    ~Graphic();

    void drawResult(cv::Mat &image, 
                    const std::vector<int> &classIds,
                    const std::vector<std::string> &classNames,
                    const std::vector<float> &confidences,
                    const std::vector<cv::Rect> &boxes);

    float getFps();
    cv::Size getWindowSize();

  private:
    // Information about the input video
    float _fps;
    int image_width;
    int image_height;
    cv::Size window_size;

    // colors being assigned to classes randomly
    std::vector<cv::Scalar> class_color;

    void setClassColor(int class_num);
};

#endif
