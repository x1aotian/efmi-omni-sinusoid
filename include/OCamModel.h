//
// Created by xuqw on 12/4/19.
//

#ifndef OMNI_SINUSOID_OCAMMODEL_H
#define OMNI_SINUSOID_OCAMMODEL_H

#include <opencv2/core/core.hpp>
#include "CameraModel.h"

class OCamModel : public CameraModel {
public:
    OCamModel();
    OCamModel(double cdeu0v0[],
              cv::Mat_<double> p_,
              cv::Mat_<double> invP_);
    OCamModel(double cdeu0v0[],
              cv::Mat_<double> p_,
              cv::Mat_<double> invP_,
              double Iw_,
              double Ih_);
    ~OCamModel(){}

    cv::Point3d ImgToWorld(cv::Point2d & pixel);
    cv::Point2d WorldToImg(cv::Point3d & X);

private:
    // affin
    double c;
    double d;
    double e;
    double invAffine;
    cv::Mat_<double> cde1;
    // principal
    double u0;
    double v0;
    // polynomial
    double p1;
    cv::Mat_<double> p;
    int p_deg;
    // inverse polynomial
    cv::Mat_<double> invP;

    int invP_deg;
    // image width and height
    double Iwidth;
    double Iheight;
    // mirror mask on pyramid levels
    std::vector<cv::Mat> mirrorMasks;
};

#endif //OMNI_SINUSOID_OCAMMODEL_H
