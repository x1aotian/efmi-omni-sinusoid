//
// Created by xuqw on 12/4/19.
//

#ifndef OMNI_SINUSOID_FRAME_H
#define OMNI_SINUSOID_FRAME_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class Frame {
public:
    Frame(){}
    Frame(const Frame & frame);
    Frame(cv::Mat & img);
    ~Frame(){}

protected:
    cv::Mat img_;
};


#endif //OMNI_SINUSOID_FRAME_H
