//
// Created by xuqw on 12/4/19.
//

#ifndef OMNI_SINUSOID_CAMERAMODEL_H
#define OMNI_SINUSOID_CAMERAMODEL_H

#include <opencv2/core/core.hpp>

class CameraModel {
public:
    CameraModel(){}
    virtual ~CameraModel(){}

    virtual cv::Point3d ImgToWorld(cv::Point2d & pixel) = 0;
    virtual cv::Point2d WorldToImg(cv::Point3d & X) = 0;
};


#endif //OMNI_SINUSOID_CAMERAMODEL_H
