//
// Created by xuqw on 12/4/19.
//

#ifndef OMNI_SINUSOID_PCAMMODEL_H
#define OMNI_SINUSOID_PCAMMODEL_H


#include <opencv2/core/core.hpp>
#include "CameraModel.h"

class PCamModel : public CameraModel{
public:
    PCamModel();
    PCamModel(double fx, double fy, double cx, double cy);
    ~PCamModel(){}
    cv::Mat K();
    cv::Point3d ImgToWorld(cv::Point2d & pixel);
    cv::Point2d WorldToImg(cv::Point3d & X);

private:
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    cv::Mat K_;
};


#endif //OMNI_SINUSOID_PCAMMODEL_H
