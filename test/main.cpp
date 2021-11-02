//
// Created by xuqw on 12/6/19.
//

#include "CTracking.h"
#include <ctime>
#include <opencv2/imgproc.hpp>
//#include "misc_0.h"
#include "misc_1.h"

int main(int argc, char ** argv){
    // read two images
    cv::Mat oimg1 = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat oimg2 = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);

    // camera model parameters
    double cx = 675.0;
    double cy = 350.0;
    double lr = 230.0;
    double sr = 120.0;
    CCamModel ccam_model = CCamModel(cx, cy, lr, sr);

    // create tracker
    int edge = 110;
    int step = 20;
    double square_coeff = 1.8;
    CTracking ctracker = CTracking(ccam_model, oimg1, oimg2, edge, step, square_coeff, WITH_TRANS_WITH_ROT);

    // register these two frames
    double sum1_t = 0.0, sum2_t =0.0;
    cv::Mat show_img;
    for (int i = 0; i < 1; ++i) {
        clock_t start_t, end1_t, end2_t;
        start_t = clock();
        ctracker.GetMotionVec();
        end1_t = clock();
        show_img = ctracker.ImgRegisterVideo();
        end2_t = clock();
        sum1_t += (double) (end1_t - start_t);
        sum2_t += (double) (end2_t - end1_t);
    }

    std::cout << "run time 1: " << sum1_t / (100 * CLOCKS_PER_SEC) << ", run time 2: " << sum2_t / (100 * CLOCKS_PER_SEC) << std::endl;

    cv::Mat show_img_pano = cv::Mat(show_img.rows + 200, show_img.cols + ctracker.cf1_->pano_img_.cols,
                                    show_img.type(), cv::Scalar(255, 255, 255));

    cv::Mat pano_color_1, pano_color_2;
    cv::cvtColor(ctracker.cf1_->pano_img_, pano_color_1, cv::COLOR_GRAY2RGB);
    cv::cvtColor(ctracker.cf2_->pano_img_, pano_color_2, cv::COLOR_GRAY2RGB);
    pano_color_1.copyTo(
            show_img_pano.rowRange(200, ctracker.cf1_->pano_img_.rows + 200).colRange(0, ctracker.cf1_->pano_img_.cols));
    cv::putText(show_img_pano, std::string("Frame 1"), cv::Point(100, 140),
                cv::FONT_HERSHEY_SIMPLEX, 1.,
                cv::Scalar(41, 36, 33), 2);

    pano_color_2.copyTo(
            show_img_pano.rowRange(ctracker.cf1_->pano_img_.rows + 400,
                                   ctracker.cf1_->pano_img_.rows + ctracker.cf2_->pano_img_.rows + 400).colRange(
                    0, ctracker.cf2_->pano_img_.cols));
    cv::putText(show_img_pano, std::string("Frame 2"), cv::Point(100, 450),
                cv::FONT_HERSHEY_SIMPLEX, 1.,
                cv::Scalar(41, 36, 33), 2);

    show_img.copyTo(show_img_pano.rowRange(0, show_img.rows).colRange(show_img_pano.cols - show_img.cols,
                                                                      show_img_pano.cols));
    std::string method;
    if(ctracker.method_ == WITH_TRANS_WITH_ROT){
        method = std::string("ours_fmt_wtrans_wrot");
    } else if(ctracker.method_ == WITH_TRANS_WITHOUT_ROT){
        method = std::string("ours_fmt_wtrans_worot");
    } else if(ctracker.method_ == WITHOUT_TRANS_WITH_ROT){
        method = std::string("ours_fmt_wotrans_wrot");
    } else if(ctracker.method_ == WITHOUT_TRANS_WITHOUT_ROT){
        method = std::string("ours_fmt_wotrans_worot");
    }

    cv::putText(show_img_pano, method,
                cv::Point(show_img_pano.cols - show_img.cols + 100, show_img.rows + 100),
                cv::FONT_HERSHEY_SIMPLEX, 2.,
                cv::Scalar(41, 36, 33), 2);

    std::string label[4] = {"x", "y", "-x", "-y"};
    for (int i = 0; i < 4; ++i) {
        cv::arrowedLine(show_img_pano, cv::Point(10 + ctracker.cf1_->pano_img_.cols / 4 * i,
                                                 ctracker.cf1_->pano_img_.rows + ctracker.cf2_->pano_img_.rows + 550),
                        cv::Point(10 + ctracker.cf1_->pano_img_.cols / 4 * i,
                                  ctracker.cf1_->pano_img_.rows + ctracker.cf2_->pano_img_.rows + 500),
                        cv::Scalar(41, 36, 33), 2);
        cv::putText(show_img_pano, label[i],
                    cv::Point(10 + ctracker.cf1_->pano_img_.cols / 4 * i,
                              ctracker.cf1_->pano_img_.rows + ctracker.cf2_->pano_img_.rows + 570),
                    cv::FONT_HERSHEY_SIMPLEX, 1.,
                    cv::Scalar(41, 36, 33), 2);
    }

    cv::Mat pose_R = ctracker.R_.inv();
    cv::Vec3d euler_angles = rotationMatrixToEulerAngles(pose_R); // yaw pitch roll
    std::string estimate = std::string("estimated (degrees): ")
                           + std::to_string(euler_angles[0] * 180.0 / M_PI) + std::string(", ")
                           + std::to_string(euler_angles[1] * 180.0 / M_PI) + std::string(", ")
                           + std::to_string(euler_angles[2] * 180.0 / M_PI);
    std::string gt = std::string("ground truth (degrees): ")
                           + std::to_string(euler_angles[0] * 180.0 / M_PI) + std::string(", ")
                           + std::to_string(euler_angles[1] * 180.0 / M_PI) + std::string(", ")
                           + std::to_string(euler_angles[2] * 180.0 / M_PI);

    cv::putText(show_img_pano, estimate,
                cv::Point(100,
                          ctracker.cf1_->pano_img_.rows + ctracker.cf2_->pano_img_.rows + 700),
                cv::FONT_HERSHEY_SIMPLEX, 1.,
                cv::Scalar(41, 36, 33), 2);
    cv::putText(show_img_pano, gt,
                cv::Point(60,
                          ctracker.cf1_->pano_img_.rows + ctracker.cf2_->pano_img_.rows + 800),
                cv::FONT_HERSHEY_SIMPLEX, 1.,
                cv::Scalar(41, 36, 33), 2);

    cv::namedWindow("show_img", cv::WINDOW_NORMAL);
    cv::imshow("show_img", show_img_pano);
    cv::imwrite("show_img.jpg", show_img_pano);

    cv::imwrite("pano_1.jpg", ctracker.cf1_->pano_img_);
    cv::imwrite("pano_2.jpg", ctracker.cf2_->pano_img_);
    cv::waitKey(0);

    return 0;
}
