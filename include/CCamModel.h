//
// Created by xuqw on 12/4/19.
//

#ifndef OMNI_SINUSOID_CCAMMODEL_H
#define OMNI_SINUSOID_CCAMMODEL_H

#include <opencv2/core/core.hpp>
#include "CameraModel.h"

#define PI 3.1416

//Cylinder model

class CCamModel : public CameraModel{
public:
    CCamModel(){}
    CCamModel(const CCamModel & ccam_model);
    CCamModel(double width, double height);
    CCamModel(double cx, double cy, double lr, double sr);
    ~CCamModel(){}

    cv::Point3d ImgToWorld(cv::Point2d & pixel);
    cv::Point2d WorldToImg(cv::Point3d & X);

    cv::Point2d OmniToPano(cv::Point2d & opixel);
    cv::Point2d PanoToOmni(cv::Point2d & ppixel);

    double GetWidth();
    double GetHeight();
    double GetR();
    double GetH();
    double GetCx();
    double GetCy();
    double GetLr();
    double GetSr();

protected:
    double width_; // panorama image width = 2 * pi * R
    double height_; // panorama image height = 2 * h, assume the origin of the coordinate is in the center of the cylinder

    double r_;
    double h_;

    double cx_; // omni image center x
    double cy_; // omni image center y
    double lr_; // omni image larger radius
    double sr_; // omni image smaller radius
};


#endif //OMNI_SINUSOID_CCAMMODEL_H
