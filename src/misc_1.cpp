//
// Created by xuqw on 12/8/19.
//

#include <opencv2/opencv.hpp>
#include "misc_1.h"

void WriteVector(std::string filename, std::vector<double> vals) {
    std::ofstream save_vec;
    save_vec.open(filename.c_str());

    for (int i = 0; i < vals.size(); ++i) {
        save_vec << vals[i] << std::endl;
    }
}

void LoadImages(
        const int startFrame,
        const int endFrame,
        const std::string path2imgs,
        std::vector<std::string> &imageFilenames) {
    std::ifstream fTimes;
    std::string strPathTimeFile = path2imgs + "/imageLists.txt";
    fTimes.open(strPathTimeFile.c_str());
    std::string line;
    int cnt = 1;
    while (std::getline(fTimes, line)) {
        if (cnt >= startFrame && cnt < endFrame) // skip until startframe
        {
            std::istringstream iss(line);
            std::string pathimg1;
            if (!(iss >> pathimg1))
                break;
            imageFilenames.push_back(path2imgs + '/' + pathimg1);
        }
        ++cnt;
    }
}

void ReadGts(const std::string path2gts,
             std::vector<cv::Vec3d> &gts) {
    gts.clear();
    std::ifstream fgts;
    std::string strPathFile = path2gts;
    fgts.open(strPathFile.c_str());
    std::string line;
    while (std::getline(fgts, line)) {

        std::istringstream iss(line);
        double yaw, pitch, roll;
        if (!(iss >> yaw >> pitch >> roll))
            break;
        gts.push_back(cv::Vec3d(yaw, pitch, roll));
    }
}

cv::Mat eulerAnglesToRotationMatrix(cv::Vec3d &theta)
{
    // Calculate rotation about x axis
    cv::Mat R_x;
    R_x = (cv::Mat_<double>(3,3) <<
                                 1,       0,              0,
            0,       cos(theta[2]),   -sin(theta[2]),
            0,       sin(theta[2]),   cos(theta[2])
    );

    // Calculate rotation about y axis
    cv::Mat R_y;
    R_y = (cv::Mat_<double>(3,3) <<
                                 cos(theta[1]),    0,      sin(theta[1]),
            0,               1,      0,
            -sin(theta[1]),   0,      cos(theta[1])
    );

    // Calculate rotation about z axis
    cv::Mat R_z;
    R_z = (cv::Mat_<double>(3,3) <<
                                 cos(theta[0]),    -sin(theta[0]),      0,
            sin(theta[0]),    cos(theta[0]),       0,
            0,               0,                  1);

    // Combined rotation matrix
    cv::Mat R = R_z * R_y * R_x;
    return R;
}

bool isRotationMatrix(cv::Mat &R) {
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

    return cv::norm(I, shouldBeIdentity) < 1e-6;

}

cv::Vec3d rotationMatrixToEulerAngles(cv::Mat &R) {
    assert(isRotationMatrix(R));

    double sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6;

    double x = 0., y = 0., z = 0.;
    if (!singular) {
        x = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        x = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return cv::Vec3d(z, y, x);
}

cv::Mat display_wtrans_worot(int height, int width,
                             std::vector<double> idxs,
                             std::vector<double> delta_u,
                             double param_u[3],
                             std::vector<double> delta_v,
                             double param_v[3]) {
    double scale = 6;
    cv::Mat show_img = cv::Mat(height * scale, width, CV_8UC3, cv::Scalar(255, 255, 255));
    //plot coordinates
    for (int i = -10; i <= 10; ++i) {
        cv::line(show_img, cv::Point(0, (-i * 10 + height / 2) * scale),
                 cv::Point(width, (-i * 10 + height / 2) * scale),
                 cv::Scalar(144, 144, 144), 2);
        cv::putText(show_img, std::to_string(i * 10), cv::Point(0, (-i * 10 + height / 2) * scale),
                    cv::FONT_HERSHEY_SIMPLEX, 1.,
                    cv::Scalar(144, 144, 144), 2);
    }

    // plot legend
//    cv::circle(show_img, cv::Point(width - 300, 2.5 * scale), 5, cv::Scalar(255, 0, 0), 3);
//    cv::putText(show_img, std::string("delta u"), cv::Point(width - 260, 3.5 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::circle(show_img, cv::Point(width - 300, 7. * scale), 5, cv::Scalar(0, 255, 0), 3);
//    cv::putText(show_img, std::string("delta v"), cv::Point(width - 260, 8. * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::line(show_img, cv::Point(width - 350, 11.5 * scale), cv::Point(width - 300, 11.5 * scale),
//             cv::Scalar(0, 165, 255));
//    cv::putText(show_img, std::string("Fitting delta u"), cv::Point(width - 260, 12.5 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::line(show_img, cv::Point(width - 350, 16. * scale), cv::Point(width - 300, 16. * scale),
//             cv::Scalar(255, 0, 255));
//    cv::putText(show_img, std::string("Fitting delta v"), cv::Point(width - 260, 17. * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);


//    cv::putText(show_img,
//                std::string("u"),
//                cv::Point(50, 3 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("A(pixel):")+ std::to_string(param_u[0]),
//                cv::Point(50, 8 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("P(pixel.):")+ std::to_string(param_u[1]),
//                cv::Point(50, 15 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("O(pixel):") + std::to_string(param_u[2]),
//                cv::Point(50, 22 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::putText(show_img,
//                std::string("u"),
//                cv::Point(400, 3 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("A(deg.):"),
//                cv::Point(400, 8 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("P(deg.):"),
//                cv::Point(400, 15 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("O(deg.):")+ std::to_string(param_u[2]/width*360.),
//                cv::Point(400, 22 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::putText(show_img,
//                std::string("v"),
//                cv::Point(400, 75 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("A(pixel):") + std::to_string(param_v[0]* width / (2 * M_PI)),
//                cv::Point(400, 81 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("P(pixel):") + std::to_string(param_v[1]* width / (2 * M_PI)),
//                cv::Point(400, 87 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("O(pixel):")+ std::to_string(param_v[2]),
//                cv::Point(400, 95 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::putText(show_img,
//                std::string("v"),
//                cv::Point(800, 75 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("A(deg.):") + std::to_string(param_v[0]/M_PI*180.),
//                cv::Point(800, 81 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("P(deg.):") + std::to_string(param_v[1]/M_PI*180.),
//                cv::Point(800, 87 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("O(deg):"),
//                cv::Point(800, 95 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);

    for (int i = 0; i < idxs.size(); ++i) {
        cv::circle(show_img, cv::Point(idxs[i], (-delta_u[i] + height / 2) * scale), 5, cv::Scalar(255, 0, 0), 3);
        cv::circle(show_img, cv::Point(idxs[i], (-delta_v[i] + height / 2) * scale), 5, cv::Scalar(0, 255, 0), 3);
    }
    for (int j = 0; j < width; ++j) {
        double delta_u = width / (2 * M_PI) * param_u[0] *
                         std::sin((2 * M_PI / width) * j + param_u[1])
                         + param_u[2];
        cv::circle(show_img, cv::Point(j, (-delta_u + height / 2) * scale), 1, cv::Scalar(0, 165, 255), 1);
        double delta_v = width / (2 * M_PI) * param_v[0] *
                         std::sin((2 * M_PI / width) * j + param_v[1])
                         + param_v[2];
        cv::circle(show_img, cv::Point(j, (-delta_v + height / 2) * scale), 1, cv::Scalar(255, 0, 255), 1);
    }
    return show_img;
}

cv::Mat display_wtrans_worot(int height, int width,
                             std::vector<double> idxs,
                             std::vector<double> delta_u,
                             std::vector<double> param_u,
                             std::vector<double> delta_v,
                             std::vector<double> param_v) {
    double u_copy[param_u.size()];
    for (int i = 0; i < param_u.size(); ++i) {
        u_copy[i] = param_u[i];
    }
    double v_copy[param_v.size()];
    for (int i = 0; i < param_v.size(); ++i) {
        v_copy[i] = param_v[i];
    }
    cv::Mat show_img = display_wtrans_worot(height, width, idxs, delta_u, u_copy, delta_v, v_copy);
    return show_img;
}

cv::Mat display_wotrans_worot(int height, int width,
                              std::vector<double> idxs,
                              std::vector<double> delta_u,
                              double param_u[1],
                              std::vector<double> delta_v,
                              double param_v[2]) {
    double scale = 6;
    cv::Mat show_img = cv::Mat(height * scale, width, CV_8UC3, cv::Scalar(255, 255, 255));
    //plot coordinates
    for (int i = -10; i <= 10; ++i) {
        cv::line(show_img, cv::Point(0, (-i * 10 + height / 2) * scale),
                 cv::Point(width, (-i * 10 + height / 2) * scale),
                 cv::Scalar(144, 144, 144), 2);
        cv::putText(show_img, std::to_string(i * 10), cv::Point(0, (-i * 10 + height / 2) * scale),
                    cv::FONT_HERSHEY_SIMPLEX, 1.,
                    cv::Scalar(144, 144, 144), 2);
    }

    // plot legend
//    cv::circle(show_img, cv::Point(width - 300, 2.5 * scale), 5, cv::Scalar(255, 0, 0), 3);
//    cv::putText(show_img, std::string("delta u"), cv::Point(width - 260, 3.5 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::circle(show_img, cv::Point(width - 300, 7. * scale), 5, cv::Scalar(0, 255, 0), 3);
//    cv::putText(show_img, std::string("delta v"), cv::Point(width - 260, 8. * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::line(show_img, cv::Point(width - 350, 11.5 * scale), cv::Point(width - 300, 11.5 * scale),
//             cv::Scalar(0, 165, 255));
//    cv::putText(show_img, std::string("Fitting delta u"), cv::Point(width - 260, 12.5 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::line(show_img, cv::Point(width - 350, 16. * scale), cv::Point(width - 300, 16. * scale),
//             cv::Scalar(255, 0, 255));
//    cv::putText(show_img, std::string("Fitting delta v"), cv::Point(width - 260, 17. * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);

//    cv::putText(show_img,
//                std::string("u"),
//                cv::Point(50, 3 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("A(pixel):"),
//                cv::Point(50, 8 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("P(pixel.):"),
//                cv::Point(50, 15 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("O(pixel):") + std::to_string(param_u[0]),
//                cv::Point(50, 22 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::putText(show_img,
//                std::string("u"),
//                cv::Point(400, 3 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("A(deg.):"),
//                cv::Point(400, 8 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("P(deg.):"),
//                cv::Point(400, 15 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("O(deg.):")+ std::to_string(param_u[0]/width*360.),
//                cv::Point(400, 22 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::putText(show_img,
//                std::string("v"),
//                cv::Point(400, 75 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("A(pixel):") + std::to_string(param_v[0]* width / (2 * M_PI)),
//                cv::Point(400, 81 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("P(pixel):") + std::to_string(param_v[1]* width / (2 * M_PI)),
//                cv::Point(400, 87 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("O(pixel):"),
//                cv::Point(400, 95 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::putText(show_img,
//                std::string("v"),
//                cv::Point(800, 75 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("A(deg.):") + std::to_string(param_v[0] * 180. /M_PI),
//                cv::Point(800, 81 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("P(deg.):") + std::to_string(param_v[1] * 180. /M_PI),
//                cv::Point(800, 87 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("O(deg):"),
//                cv::Point(800, 95 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);

    for (int i = 0; i < idxs.size(); ++i) {
        cv::circle(show_img, cv::Point(idxs[i], (-delta_u[i] + height / 2) * scale), 5, cv::Scalar(255, 0, 0), 3);
        cv::circle(show_img, cv::Point(idxs[i], (-delta_v[i] + height / 2) * scale), 5, cv::Scalar(0, 255, 0), 3);
    }
    for (int j = 0; j < width; ++j) {
        double delta_u = param_u[0];
        cv::circle(show_img, cv::Point(j, (-delta_u + height / 2) * scale), 1, cv::Scalar(0, 165, 255), 1);
        double delta_v = width / (2 * M_PI) * param_v[0] *
                         std::sin((2 * M_PI / width) * j + param_v[1]);
        cv::circle(show_img, cv::Point(j, (-delta_v + height / 2) * scale), 1, cv::Scalar(255, 0, 255), 1);
    }
    return show_img;
}

cv::Mat display_wotrans_worot(int height, int width,
                              std::vector<double> idxs,
                              std::vector<double> delta_u,
                              double param_u[1],
                              std::vector<double> delta_v,
                              std::vector<double> &param_v) {
    double v_copy[param_v.size()];
    for (int i = 0; i < param_v.size(); ++i) {
        v_copy[i] = param_v[i];
    }
    cv::Mat show_img = display_wotrans_worot(height, width, idxs, delta_u, param_u, delta_v, v_copy);
    return show_img;
}

cv::Mat display_wtrans_wrot(int height, int width,
                            std::vector<double> idxs,
                            std::vector<double> delta_u,
                            std::vector<double> delta_theta,
                            double param_u[3],
                            std::vector<double> delta_v, double param_v[3]) {
    double scale = 6;
    cv::Mat show_img = cv::Mat(height * scale, width, CV_8UC3, cv::Scalar(255, 255, 255));
    // plot coordinates
    for (int i = -10; i <= 10; ++i) {
        cv::line(show_img, cv::Point(0, (-i * 10 + height / 2) * scale),
                 cv::Point(width, (-i * 10 + height / 2) * scale),
                 cv::Scalar(144, 144, 144), 2);
        cv::putText(show_img, std::to_string(i * 10), cv::Point(0, (-i * 10 + height / 2) * scale),
                    cv::FONT_HERSHEY_SIMPLEX, 1.,
                    cv::Scalar(144, 144, 144), 2);
    }

    // plot legend
//    cv::circle(show_img, cv::Point(width - 300, 2.5 * scale), 5, cv::Scalar(255, 0, 0), 3);
//    cv::putText(show_img, std::string("delta u"), cv::Point(width - 260, 3.5 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::circle(show_img, cv::Point(width - 300, 7. * scale), 5, cv::Scalar(0, 255, 0), 3);
//    cv::putText(show_img, std::string("delta v"), cv::Point(width - 260, 8. * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::line(show_img, cv::Point(width - 350, 11.5 * scale), cv::Point(width - 300, 11.5 * scale),
//             cv::Scalar(0, 165, 255), 2);
//    cv::putText(show_img, std::string("Fitting delta u"), cv::Point(width - 260, 12.5 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::line(show_img, cv::Point(width - 350, 16. * scale), cv::Point(width - 300, 16. * scale),
//             cv::Scalar(255, 0, 255), 2);
//    cv::putText(show_img, std::string("Fitting delta v"), cv::Point(width - 260, 17. * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);

//    cv::putText(show_img,
//                std::string("u"),
//                cv::Point(50, 3 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("A(pixel):")+ std::to_string(param_u[0]),
//                cv::Point(50, 8 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("P(pixel.):")+ std::to_string(param_u[1]),
//                cv::Point(50, 15 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("O(pixel):") + std::to_string(param_u[2]),
//                cv::Point(50, 22 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::putText(show_img,
//                std::string("u"),
//                cv::Point(400, 3 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("A(deg.):"),
//                cv::Point(400, 8 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("P(deg.):"),
//                cv::Point(400, 15 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("O(deg.):")+ std::to_string(param_u[2]/width*360.),
//                cv::Point(400, 22 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::putText(show_img,
//                std::string("v"),
//                cv::Point(400, 75 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("A(pixel):") + std::to_string(param_v[0]* width / (2 * M_PI)),
//                cv::Point(400, 81 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("P(pixel):") + std::to_string(param_v[1]* width / (2 * M_PI)),
//                cv::Point(400, 87 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("O(pixel):")+ std::to_string(param_v[2]),
//                cv::Point(400, 95 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::putText(show_img,
//                std::string("v"),
//                cv::Point(800, 75 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("A(deg.):") + std::to_string(param_v[0]/M_PI*180.),
//                cv::Point(800, 81 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("P(deg.):") + std::to_string(param_v[1]/M_PI*180.),
//                cv::Point(800, 87 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("O(deg):"),
//                cv::Point(800, 95 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);


    for (int i = 0; i < idxs.size(); ++i) {
        cv::circle(show_img, cv::Point(idxs[i], (-delta_u[i] + height / 2) * scale), 5, cv::Scalar(255, 0, 0), 3);
        cv::circle(show_img, cv::Point(idxs[i],
                                       (-(delta_v[i] - width / (2 * M_PI) * 0.1 * delta_theta[i]) + height / 2) *
                                       scale),
                   5, cv::Scalar(0, 255, 0), 3);
    }
    for (int j = 0; j < width; ++j) {
        double delta_u = width / (2 * M_PI) * param_u[0] *
                         std::sin((2 * M_PI / width) * j + param_u[1])
                         + param_u[2];
        cv::circle(show_img, cv::Point(j, (-delta_u + height / 2) * scale), 1, cv::Scalar(0, 165, 255), 1);
        double delta_v = width / (2 * M_PI) * param_v[0] *
                         std::sin((2 * M_PI / width) * j + param_v[1])
                         + param_v[2] - width / (2 * M_PI) * 0.1 * param_v[0] *
                                        std::cos((2 * M_PI / width) * j + param_v[1]);
        cv::circle(show_img, cv::Point(j, (-delta_v + height / 2) * scale), 1, cv::Scalar(255, 0, 255), 1);
    }
    return show_img;
}

cv::Mat display_wtrans_wrot(int height, int width,
                            std::vector<double> idxs,
                            std::vector<double> delta_u,
                            std::vector<double> delta_theta,
                            std::vector<double> &param_u,
                            std::vector<double> delta_v, std::vector<double> &param_v) {
    double u_copy[param_u.size()];
    for (int i = 0; i < param_u.size(); ++i) {
        u_copy[i] = param_u[i];
    }
    double v_copy[param_v.size()];
    for (int i = 0; i < param_v.size(); ++i) {
        v_copy[i] = param_v[i];
    }
    cv::Mat show_img = display_wtrans_wrot(height, width, idxs, delta_u, delta_theta, u_copy, delta_v, v_copy);
    return show_img;
}

cv::Mat display_wotrans_wrot(int height, int width,
                             std::vector<double> idxs,
                             std::vector<double> delta_u,
                             std::vector<double> delta_theta,
                             double param_u[1],
                             std::vector<double> delta_v, double param_v[2]) {
    double scale = 6;
    cv::Mat show_img = cv::Mat(height * scale, width, CV_8UC3, cv::Scalar(255, 255, 255));
    //plot coordinates
    for (int i = -10; i <= 10; ++i) {
        cv::line(show_img, cv::Point(0, (-i * 10 + height / 2) * scale),
                 cv::Point(width, (-i * 10 + height / 2) * scale),
                 cv::Scalar(144, 144, 144), 2);
        cv::putText(show_img, std::to_string(i * 10), cv::Point(0, (-i * 10 + height / 2) * scale),
                    cv::FONT_HERSHEY_SIMPLEX, 1.,
                    cv::Scalar(144, 144, 144), 2);
    }

    // plot legend
//    cv::circle(show_img, cv::Point(width - 300, 2.5 * scale), 5, cv::Scalar(255, 0, 0), 3);
//    cv::putText(show_img, std::string("delta u"), cv::Point(width - 260, 3.5 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::circle(show_img, cv::Point(width - 300, 7. * scale), 5, cv::Scalar(0, 255, 0), 3);
//    cv::putText(show_img, std::string("delta v"), cv::Point(width - 260, 8. * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::line(show_img, cv::Point(width - 350, 11.5 * scale), cv::Point(width - 300, 11.5 * scale),
//             cv::Scalar(0, 165, 255));
//    cv::putText(show_img, std::string("Fitting delta u"), cv::Point(width - 260, 12.5 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::line(show_img, cv::Point(width - 350, 16. * scale), cv::Point(width - 300, 16. * scale),
//             cv::Scalar(255, 0, 255));
//    cv::putText(show_img, std::string("Fitting delta v"), cv::Point(width - 260, 17. * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);

//    cv::putText(show_img,
//                std::string("u"),
//                cv::Point(50, 3 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("A(pixel):"),
//                cv::Point(50, 8 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("P(pixel.):"),
//                cv::Point(50, 15 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("O(pixel):") + std::to_string(param_u[0]),
//                cv::Point(50, 22 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::putText(show_img,
//                std::string("u"),
//                cv::Point(400, 3 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("A(deg.):"),
//                cv::Point(400, 8 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("P(deg.):"),
//                cv::Point(400, 15 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("O(deg.):")+ std::to_string(param_u[0]/width*360.),
//                cv::Point(400, 22 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::putText(show_img,
//                std::string("v"),
//                cv::Point(400, 75 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("A(pixel):") + std::to_string(param_v[0]* width / (2 * M_PI)),
//                cv::Point(400, 81 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("P(pixel):") + std::to_string(param_v[1]* width / (2 * M_PI)),
//                cv::Point(400, 87 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("O(pixel):"),
//                cv::Point(400, 95 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//
//    cv::putText(show_img,
//                std::string("v"),
//                cv::Point(800, 75 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("A(deg.):") + std::to_string(param_v[0] * 180. /M_PI),
//                cv::Point(800, 81 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("P(deg.):") + std::to_string(param_v[1] * 180. /M_PI),
//                cv::Point(800, 87 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);
//    cv::putText(show_img,
//                std::string("O(deg):"),
//                cv::Point(800, 95 * scale),
//                cv::FONT_HERSHEY_SIMPLEX, 1.,
//                cv::Scalar(0, 0, 0), 2);

    for (int i = 0; i < idxs.size(); ++i) {
        cv::circle(show_img, cv::Point(idxs[i], (-delta_u[i] + height / 2) * scale), 5, cv::Scalar(255, 0, 0), 3);
        cv::circle(show_img, cv::Point(idxs[i],
                                       (-(delta_v[i] - width / (2 * M_PI) * 0.1 * delta_theta[i]) + height / 2) *
                                       scale),
                   5, cv::Scalar(0, 255, 0), 3);
    }
    for (int j = 0; j < width; ++j) {
        double delta_u = param_u[0];
        cv::circle(show_img, cv::Point(j, (-delta_u + height / 2) * scale), 1, cv::Scalar(0, 165, 255), 1);
        double delta_v = width / (2 * M_PI) * param_v[0] *
                         std::sin((2 * M_PI / width) * j + param_v[1])
                         - width / (2 * M_PI) * 0.1 * param_v[0] *
                           std::cos((2 * M_PI / width) * j + param_v[1]);
        cv::circle(show_img, cv::Point(j, (-delta_v + height / 2) * scale), 1, cv::Scalar(255, 0, 255), 1);
    }
    return show_img;
}

cv::Mat display_wotrans_wrot(int height, int width,
                             std::vector<double> &idxs,
                             std::vector<double> &delta_u,
                             std::vector<double> &delta_theta,
                             double param_u[1],
                             std::vector<double> &delta_v, std::vector<double> &param_v) {
//    double u_copy[param_u.size()];
//    for (int i = 0; i < param_u.size(); ++i) {
//        u_copy[i] = param_u[i];
//    }
    double v_copy[param_v.size()];
    for (int i = 0; i < param_v.size(); ++i) {
        v_copy[i] = param_v[i];
    }
    cv::Mat show_img = display_wotrans_wrot(height, width, idxs, delta_u, delta_theta, param_u, delta_v, v_copy);
    return show_img;
}
