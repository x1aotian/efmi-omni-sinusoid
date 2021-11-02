//
// Created by xuqw on 12/5/19.
//

#include "Tracking.h"

Tracking::Tracking(CameraModel &cam_model,
                   cv::Mat &img1,
                   cv::Mat &img2) {
    cam_model_ = & cam_model;
    f1_ = new Frame(img1);
    f2_ = new Frame(img2);
}

