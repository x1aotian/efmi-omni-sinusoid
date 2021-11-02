//
// Created by xuqw on 12/8/19.
//

#ifndef OMNI_SINUSOID_MISC_1_H
#define OMNI_SINUSOID_MISC_1_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void WriteVector(std::string filename, std::vector<double> vals);

template<typename T>
std::vector<T> MergeVector(std::vector<T> v1, std::vector<T> v2) {
    size_t s1 = v1.size();
    size_t s2 = v2.size();
    std::vector<T> merge(s1 + s2);
    for (int i = 0; i < s1; ++i) {
        merge[i] = v1[i];
    }
    for (int j = 0; j < s2; ++j) {
        merge[s1 + j] = v2[j];
    }
    return merge;
}

template<typename T>
std::vector<T> MixVector(std::vector<T> v1, std::vector<T> v2, double w) {
    std::vector<T> mix(v1.size());
    for (int i = 0; i < v1.size(); ++i) {
        mix[i] = v1[i] + w * v2[i];
    }
    return mix;
}

template<typename T>
T AverVector(std::vector<T> v1) {
    T sum = 0;
    if (v1.empty()) {
        return sum;
    }
    for (int i = 0; i < v1.size(); ++i) {
        sum += v1[i];
    }
    return sum / v1.size();
}

template<typename T>
T MedianVector(std::vector<T> v1) {
    size_t len = v1.size();
    if (len % 2) {
        auto idx = (len - 1) / 2;
        return v1[idx];
    } else {
        auto idx_1 = (len - 2) / 2;
        auto idx_2 = len / 2;
        return (v1[idx_1] + v1[idx_2]) / 2;
    }
}

template<typename T>
T AverDistance(std::vector<T> vx, std::vector<T> vy) {
    T sum = 0;
    if (vx.empty()) {
        return sum;
    }
    for (int i = 0; i < vx.size(); ++i) {
        sum += sqrt(pow(vx[i], 2) + pow(vy[i], 2));
    }
    return sum / vx.size();
}

void LoadImages(
        const int startFrame,
        const int endFrame,
        const std::string path2imgs,
        std::vector<std::string> &imageFilenames);

void ReadGts(const std::string path2gts,
             std::vector<cv::Vec3d> &gts);

cv::Mat eulerAnglesToRotationMatrix(cv::Vec3d &theta);

bool isRotationMatrix(cv::Mat &R);

cv::Vec3d rotationMatrixToEulerAngles(cv::Mat &R);

cv::Mat display_wtrans_worot(int height, int width,
                             std::vector<double> idxs,
                             std::vector<double> delta_u,
                             double param_u[3],
                             std::vector<double> delta_v, double param_v[3]);
cv::Mat display_wtrans_worot(int height, int width,
                             std::vector<double> idxs,
                             std::vector<double> delta_u,
                             std::vector<double> param_u,
                             std::vector<double> delta_v,
                             std::vector<double> param_v);

cv::Mat display_wotrans_worot(int height, int width,
                              std::vector<double> idxs,
                              std::vector<double> delta_u,
                              double param_u[1],
                              std::vector<double> delta_v, double param_v[2]);
cv::Mat display_wotrans_worot(int height, int width,
                              std::vector<double> idxs,
                              std::vector<double> delta_u,
                              double param_u[1],
                              std::vector<double> delta_v,
                              std::vector<double> &param_v);

cv::Mat display_wtrans_wrot(int height, int width,
                            std::vector<double> idxs,
                            std::vector<double> delta_u,
                            std::vector<double> delta_theta,
                            double param_u[3],
                            std::vector<double> delta_v, double param_v[3]);
cv::Mat display_wtrans_wrot(int height, int width,
                            std::vector<double> idxs,
                            std::vector<double> delta_u,
                            std::vector<double> delta_theta,
                            std::vector<double> &param_u,
                            std::vector<double> delta_v, std::vector<double> &param_v);

cv::Mat display_wotrans_wrot(int height, int width,
                             std::vector<double> idxs,
                             std::vector<double> delta_u,
                             std::vector<double> delta_theta,
                             double param_u[1],
                             std::vector<double> delta_v, double param_v[2]);
cv::Mat display_wotrans_wrot(int height, int width,
                             std::vector<double> &idxs,
                             std::vector<double> &delta_u,
                             std::vector<double> &delta_theta,
                             double param_u[1],
                             std::vector<double> &delta_v, std::vector<double> &param_v);


#endif //OMNI_SINUSOID_MISC_H
