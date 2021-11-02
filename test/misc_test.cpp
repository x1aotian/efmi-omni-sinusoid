//
// Created by xuqw on 1/8/20.
//

#include "misc.h"
#include <iostream>
#include <opencv2/core/core.hpp>

using namespace std;

int main(int argc, char ** argv){
    double theta_x = M_PI / 10;
    double theta_y = -M_PI / 4;
    double theta_z = M_PI / 10;

    cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(theta_x), -sin(theta_x), 0, sin(theta_x), cos(theta_x));
    cv::Mat Ry = (cv::Mat_<double>(3, 3) << cos(theta_y), 0, sin(theta_y), 0, 1, 0, -sin(theta_y), 0, cos(theta_y));
    cv::Mat Rz = (cv::Mat_<double>(3, 3) << cos(theta_z), -sin(theta_z), 0, sin(theta_z), cos(theta_z), 0, 0, 0, 1);

    cv::Mat R = Rz * Ry * Rx;

    cv::Vec3d euler_angles = rotationMatrixToEulerAngles(R); // yaw pitch roll
    cout << " " << euler_angles[0] * 180 / M_PI << " "
         << euler_angles[1] * 180 / M_PI << " "
         << euler_angles[2] * 180 / M_PI << std::endl;

    return 0;
}

