//
// Created by xuqw on 12/4/19.
//

#include "PCamModel.h"

PCamModel::PCamModel() {
    K_ = cv::Mat::eye(3, 3, CV_64FC1);
}

PCamModel::PCamModel(double fx,
                     double fy,
                     double cx,
                     double cy) :
        fx_(fx),
        fy_(fy),
        cx_(cx),
        cy_(cy) {
    K_ = cv::Mat::eye(3, 3, CV_64FC1);
    K_.at<double>(0, 0) = fx_;
    K_.at<double>(1, 1) = fy_;
    K_.at<double>(0, 2) = cx_;
    K_.at<double>(1, 2) = cy_;
}

cv::Mat PCamModel::K() {
    return K_;
}

cv::Point3d PCamModel::ImgToWorld(cv::Point2d &pixel) {
    cv::Mat pt(3, 1, CV_64FC1);
    pt.at<double>(0, 0) = pixel.x;
    pt.at<double>(1, 0) = pixel.y;
    pt.at<double>(2, 0) = 1;
    cv::Mat P = K_.inv() * pt;
    P = P / norm(P);
    return cv::Point3d(P.at<double>(0, 0),
                       P.at<double>(1, 0),
                       P.at<double>(2, 0));
}

cv::Point2d PCamModel::WorldToImg(cv::Point3d &X) {
    cv::Mat P(3, 1, CV_64FC1);
    P.at<double>(0, 0) = X.x;
    P.at<double>(1, 0) = X.y;
    P.at<double>(2, 0) = X.z;
    cv::Mat px = K_ * P;
    return cv::Point2d(px.at<double>(0, 0) / px.at<double>(2, 0),
                       px.at<double>(1, 0) / px.at<double>(2, 0));
}