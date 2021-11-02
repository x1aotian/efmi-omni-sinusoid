//
// Created by xuqw on 12/5/19.
//

#ifndef OMNI_SINUSOID_TRACKING_H
#define OMNI_SINUSOID_TRACKING_H

#include "CameraModel.h"
#include "Frame.h"

class Tracking {
public:
    Tracking(){}
    Tracking(CameraModel & cam_model, cv::Mat & img1, cv::Mat & img2);
    ~Tracking(){}

    virtual void ImgRegister() = 0;

protected:
    CameraModel * cam_model_;
    Frame * f1_;
    Frame * f2_;
    cv::Mat rel_pose_;
};


#endif //OMNI_SINUSOID_TRACKING_H
