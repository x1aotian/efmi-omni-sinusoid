//
// Created by xuqw on 12/4/19.
//

#include "CCamModel.h"


CCamModel::CCamModel(const CCamModel &ccam_model) {
    width_ = ccam_model.width_;
    height_ = ccam_model.height_;
    r_ = ccam_model.r_;
    h_ = ccam_model.h_;
    cx_ = ccam_model.cx_;
    cy_ = ccam_model.cy_;
    lr_ = ccam_model.lr_;
    sr_ = ccam_model.sr_;
}

CCamModel::CCamModel(double width,
                     double height) : width_(width),
                                      height_(height) {
    r_ = width / (2.0 * PI);
    h_ = height / 2.0;
}

CCamModel::CCamModel(double cx,
                     double cy,
                     double lr,
                     double sr) : cx_(cx),
                                  cy_(cy),
                                  lr_(lr),
                                  sr_(sr) {
    height_ = lr_ - sr_;
    width_ = PI * (lr_ + sr_) + 1;
    r_ = width_ / (2.0 * PI);
    h_ = height_ / 2.0;
}

double CCamModel::GetCx() {
    return cx_;
}

double CCamModel::GetCy() {
    return cy_;
}

double CCamModel::GetWidth() {
    return width_;
}

double CCamModel::GetHeight() {
    return height_;
}

double CCamModel::GetH() {
    return h_;
}

double CCamModel::GetR() {
    return r_;
}

double CCamModel::GetLr() {
    return lr_;
}

double CCamModel::GetSr() {
    return sr_;
}

cv::Point3d CCamModel::ImgToWorld(cv::Point2d &pixel) {
    double u = pixel.x;
    double v = pixel.y;

    double theta = u / r_;
    double x = r_ * cos(theta);
    double y = r_ * sin(theta);
    double z = h_ - v;

    return cv::Point3d(x, y, z);
}

cv::Point2d CCamModel::WorldToImg(cv::Point3d &X) {
    double u = r_ * atan2(X.y, X.x);
    double v = h_ - X.z;

    return cv::Point2d(u, v);
}

cv::Point2d CCamModel::OmniToPano(cv::Point2d &opixel) {
    double x = opixel.x - cx_;
    double y = opixel.y - cy_;

    double r = sqrt(pow(x, 2) + pow(y, 2));
    double theta = atan2(x, y);

    double v = r - sr_;
    double u = theta / (2 * PI) * width_;
    return cv::Point2d(u,v);
}

cv::Point2d CCamModel::PanoToOmni(cv::Point2d &ppixel) {
    double r = 0.0, theta = 0.0;
    r = ppixel.y / height_ * (lr_ - sr_) + sr_;
    theta = ppixel.x / width_ * 2 * PI;
    cv::Point2d opixel;
    opixel.x = cx_ + r * sin(theta); //col
    opixel.y = cy_ + r * cos(theta); //row
    return opixel;
}
