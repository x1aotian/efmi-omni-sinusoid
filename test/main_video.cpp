//
// Created by xuqw on 8/17/20.
//

#include "CSystem.h"
#include <time.h>
#include <opencv2/imgproc.hpp>
#include <misc.h>

#include <sstream>

int main(int argc, char **argv) {
    std::string path2params = std::string(argv[1]);
    cv::FileStorage fSettings(path2params, cv::FileStorage::READ);
    // read images
    auto path2imgs = (std::string) fSettings["path_imgs"];
    int startFrame = 1;
    int endFrame = 200;
    std::vector<std::string> imgNames;
    LoadImages(startFrame, endFrame, path2imgs, imgNames);
    std::cout << "imgNames size " << imgNames.size() << std::endl;

    auto dataset_name = (std::string) fSettings["name"];
    auto path2gt = (std::string) fSettings["path_gt"];
    std::vector<cv::Vec3d> gts;
    ReadGts(path2gt, gts);

    auto cx = (double) fSettings["cx"];
    auto cy = (double) fSettings["cy"];
    auto lr = (double) fSettings["lr"];
    auto sr = (double) fSettings["sr"];
    CCamModel ccam_model = CCamModel(cx, cy, lr, sr);
    auto edge = (int) fSettings["edge"];
    auto step = (int) fSettings["step"];
    auto square_coeff = (double) fSettings["square_coeff"];

    CTracking ctracker1 = CTracking(ccam_model, edge, step, square_coeff, WITH_TRANS_WITH_ROT);
    CTracking ctracker2 = CTracking(ccam_model, edge, step, square_coeff, WITHOUT_TRANS_WITH_ROT);
    CTracking ctracker3 = CTracking(ccam_model, edge, step, square_coeff, WITH_TRANS_WITHOUT_ROT);
    CTracking ctracker4 = CTracking(ccam_model, edge, step, square_coeff, WITHOUT_TRANS_WITHOUT_ROT);

    CSystem csystem1 = CSystem(ccam_model, ctracker1, FMI);
    CSystem csystem2 = CSystem(ccam_model, ctracker2, FMI);
    CSystem csystem3 = CSystem(ccam_model, ctracker3, OpticalFlow);
    CSystem csystem4 = CSystem(ccam_model, ctracker4, OpticalFlow);
    // tracking
//    auto result_path = (std::string) fSettings["result_path"];
//    std::cout << "result path: " << result_path << std::endl;
//    std::ofstream out(result_path);
    csystem1.ctracker_->cf1_->idx_ = 0;
    csystem1.ctracker_->cf2_->idx_ = 0;
    double sum_time = 0.0;
    for (int i = 0; i < imgNames.size(); i += 1) {
        std::cout << "image " << imgNames[i] << std::endl;
        cv::Mat oimg = cv::imread(imgNames[i], cv::IMREAD_GRAYSCALE);

        int ref_idx = csystem1.ctracker_->cf1_->idx_;
        int curr_idx = i;

        cv::Mat fmt_ww_img = csystem1.RegisterVideo(i, oimg);
        cv::Mat fmt_wow_img = csystem2.RegisterVideo(i, oimg);
        cv::Mat opt_wwo_img = csystem3.RegisterVideo(i, oimg);
        cv::Mat opt_wowo_img = csystem4.RegisterVideo(i, oimg);



//        cv::imwrite("pano_0.jpg", csystem1.ctracker_->cf1_->pano_img_);
//        std::stringstream pano_name;
//        pano_name << "pano" << i << ".jpg";
//        cv::imwrite(pano_name.str(), csystem1.ctracker_->cf2_->pano_img_);

        cv::Vec3d euler_angles1, euler_angles2, euler_angles3, euler_angles4;
        cv::Vec3d err_angles1, err_angles2, err_angles3, err_angles4;


        std::cout << "ref idx: " << ref_idx << std::endl;
        std::cout << "curr idx: " << curr_idx << std::endl;
        cv::Vec3d gt_ref = gts[ref_idx];
        cv::Vec3d gt_curr = gts[curr_idx];

//        cv::Vec3d gt_ref = cv::Vec3d(-0.425206, -0.00296248, -0.00495558);
//        cv::Vec3d gt_curr = cv::Vec3d(-0.4474, 0.00787755, 0.0421489);
        std::cout << "gt_ref: " << gt_ref << std::endl;
        std::cout << "gt_curr: " << gt_curr << std::endl;
        cv::Mat rotm_ref = eulerAnglesToRotationMatrix(gt_ref);
        cv::Mat rotm_curr = eulerAnglesToRotationMatrix(gt_curr);
        std::cout << "rotm ref: " << rotm_ref << std::endl;
        std::cout << "rotm curr: " << rotm_curr << std::endl;
        cv::Mat rotm_rel = rotm_ref.t() * rotm_curr;
        std::cout << "rotm rel: " << rotm_rel << std::endl;
        cv::Vec3d eul_gt = rotationMatrixToEulerAngles(rotm_rel);
//        Eigen::Matrix3d eigen_rel;
//        cv::cv2eigen(rotm_rel, eigen_rel);
//        Eigen::Vector3d eul_gt = eigen_rel.eulerAngles(2, 1, 0);
        std::cout << "eul_gt: " << eul_gt << std::endl;
        std::stringstream gt1_1, gt1_2, gt1_3;
        gt1_1.precision(2);
        gt1_1.setf(std::ios::fixed);
        gt1_1 << eul_gt[0] * 180.0 / M_PI;
        gt1_2.precision(2);
        gt1_2.setf(std::ios::fixed);
        gt1_2 << -eul_gt[1] * 180.0 / M_PI;
        gt1_3.precision(2);
        gt1_3.setf(std::ios::fixed);
        gt1_3 << eul_gt[2] * 180.0 / M_PI;

        cv::Vec3d eul_gt_t = cv::Vec3d(eul_gt[0], -eul_gt[1], eul_gt[2]);
        cv::Mat trans_rel = eulerAnglesToRotationMatrix(eul_gt_t);

        if (csystem1.kf_flag_) {
            cv::Mat pose_R = csystem1.T_;
            euler_angles1 = rotationMatrixToEulerAngles(pose_R); // yaw pitch roll
            cv::Mat err_R = pose_R.t() * trans_rel;
            err_angles1 = rotationMatrixToEulerAngles(err_R);
//            out << i-1 << " " << euler_angles[0] << " " << euler_angles[1] << " " << -euler_angles[2] << std::endl;
        }

        if (csystem2.kf_flag_) {
            cv::Mat pose_R = csystem2.T_;
            euler_angles2 = rotationMatrixToEulerAngles(pose_R); // yaw pitch roll
            cv::Mat err_R = pose_R.t() * trans_rel;
            err_angles2 = rotationMatrixToEulerAngles(err_R);
//            out << i-1 << " " << euler_angles[0] << " " << euler_angles[1] << " " << -euler_angles[2] << std::endl;
        }

        if (csystem3.kf_flag_) {
            cv::Mat pose_R = csystem3.T_;
            euler_angles3 = rotationMatrixToEulerAngles(pose_R); // yaw pitch roll
            cv::Mat err_R = pose_R.t() * trans_rel;
            err_angles3 = rotationMatrixToEulerAngles(err_R);
//            out << i-1 << " " << euler_angles[0] << " " << euler_angles[1] << " " << -euler_angles[2] << std::endl;
        }

        if (csystem4.kf_flag_) {
            cv::Mat pose_R = csystem4.T_;
            euler_angles4 = rotationMatrixToEulerAngles(pose_R); // yaw pitch roll
            cv::Mat err_R = pose_R.t() * trans_rel;
            err_angles4 = rotationMatrixToEulerAngles(err_R);
//            out << i-1 << " " << euler_angles[0] << " " << euler_angles[1] << " " << -euler_angles[2] << std::endl;
        }

        if (i > 0) {


            cv::Mat show_img_pano = cv::Mat(2250, 3000, fmt_ww_img.type(), cv::Scalar(255, 255, 255));

            cv::Mat pano_color_1, pano_color_2;
            cv::Mat resized_pano_1 = cv::Mat(2 * ctracker1.cf1_->pano_img_.rows, 2 * ctracker1.cf1_->pano_img_.cols,
                                             ctracker1.cf1_->pano_img_.type());
            cv::Mat resized_pano_2 = cv::Mat(2 * ctracker1.cf1_->pano_img_.rows, 2 * ctracker1.cf1_->pano_img_.cols,
                                             ctracker1.cf1_->pano_img_.type());

            cv::resize(ctracker1.cf1_->pano_img_, resized_pano_1, resized_pano_1.size());
            cv::resize(ctracker1.cf2_->pano_img_, resized_pano_2, resized_pano_2.size());


            cv::cvtColor(resized_pano_1, pano_color_1, CV_GRAY2RGB);
            cv::cvtColor(resized_pano_2, pano_color_2, CV_GRAY2RGB);
            pano_color_1.copyTo(
                    show_img_pano.rowRange(50, pano_color_1.rows + 50).colRange(225, pano_color_1.cols + 225));
            cv::putText(show_img_pano, std::string("Frame ") + std::to_string(ref_idx),
                        cv::Point(15, 150), cv::FONT_HERSHEY_SIMPLEX, 1., cv::Scalar(41, 36, 33), 2);
            pano_color_2.copyTo(
                    show_img_pano.rowRange(300, pano_color_2.rows + 300).colRange(
                            225, pano_color_2.cols + 225));
            cv::putText(show_img_pano, std::string("Frame ") + std::to_string(curr_idx),
                        cv::Point(15, 420), cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);

            std::string label[4] = {"x", "y", "-x", "-y"};
            for (int i = 0; i < 4; ++i) {
                cv::arrowedLine(show_img_pano, cv::Point(235 + pano_color_1.cols / 4 * i,
                                                         pano_color_1.rows + 340),
                                cv::Point(235 + pano_color_1.cols / 4 * i,
                                          pano_color_1.rows + 300),
                                cv::Scalar(41, 36, 33), 2);
                cv::putText(show_img_pano, label[i],
                            cv::Point(235 + pano_color_1.cols / 4 * i,
                                      pano_color_1.rows + 350),
                            cv::FONT_HERSHEY_SIMPLEX, 1.,
                            cv::Scalar(41, 36, 33), 2);
            }

            cv::putText(show_img_pano, "Rotation Estimation for Omni-directional Cameras using Sinusoid Fitting",
                        cv::Point(1400, 30),
                        cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5,
                        cv::Scalar(41, 36, 33), 2);

            cv::putText(show_img_pano, dataset_name,
                        cv::Point(2440, 300),
                        cv::FONT_HERSHEY_SIMPLEX, 1.8,
                        cv::Scalar(41, 36, 33), 2);

            cv::Mat pose_R = ctracker1.R_.inv();

            std::stringstream es1_1, es1_2, es1_3;
            es1_1.precision(2);
            es1_1.setf(std::ios::fixed);
            es1_1 << euler_angles1[0] * 180.0 / M_PI;
            es1_2.precision(2);
            es1_2.setf(std::ios::fixed);
            es1_2 << euler_angles1[1] * 180.0 / M_PI;
            es1_3.precision(2);
            es1_3.setf(std::ios::fixed);
            es1_3 << euler_angles1[2] * 180.0 / M_PI;
            std::string estimate1 = std::string("estimated (deg.):   ") + es1_1.str()
                                    + std::string("        ")
                                    + es1_2.str()
                                    + std::string("       ") + es1_3.str();


            std::string gt = std::string("ground truth (deg.):   ")
                             + gt1_1.str()
                             + std::string("        ")
                             + gt1_2.str()
                             + std::string("       ") + gt1_3.str();

            fmt_ww_img.copyTo(show_img_pano.rowRange(585, fmt_ww_img.rows + 585).colRange(225, fmt_ww_img.cols + 225));
            cv::putText(show_img_pano, "ours_fmt_wtrans_wrot",
                        cv::Point(400, fmt_ww_img.rows + 630),
                        cv::FONT_HERSHEY_SIMPLEX, 1.8,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, "yaw(z)",
                        cv::Point(550, fmt_ww_img.rows + 665),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, "pitch(y)",
                        cv::Point(780, fmt_ww_img.rows + 665),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, "roll(x)",
                        cv::Point(990, fmt_ww_img.rows + 665),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, estimate1,
                        cv::Point(225, fmt_ww_img.rows + 715),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, gt,
                        cv::Point(179, fmt_ww_img.rows + 765),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);

            cv::putText(show_img_pano,
                        std::string("err(deg.): ") + std::to_string(cv::norm(err_angles1)),
                        cv::Point(1200, fmt_ww_img.rows + 765),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);

            std::stringstream es3_1, es3_2, es3_3;
            es3_1.precision(2);
            es3_1.setf(std::ios::fixed);
            es3_1 << euler_angles3[0] * 180.0 / M_PI;
            es3_2.precision(2);
            es3_2.setf(std::ios::fixed);
            es3_2 << euler_angles3[1] * 180.0 / M_PI;
            es3_3.precision(2);
            es3_3.setf(std::ios::fixed);
            es3_3 << euler_angles3[2] * 180.0 / M_PI;
            std::string estimate3 = std::string("estimated (deg.):   ") + es3_1.str()
                                    + std::string("        ")
                                    + es3_2.str()
                                    + std::string("       ") + es3_3.str();
            opt_wwo_img.copyTo(
                    show_img_pano.rowRange(585, fmt_ww_img.rows + 585).colRange(1675, fmt_ww_img.cols + 1675));
            cv::putText(show_img_pano, "ours_opt_wtrans_worot",
                        cv::Point(1750, fmt_ww_img.rows + 630),
                        cv::FONT_HERSHEY_SIMPLEX, 1.8,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, "yaw(z)",
                        cv::Point(1990, fmt_ww_img.rows + 665),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, "pitch(y)",
                        cv::Point(2250, fmt_ww_img.rows + 665),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, "roll(x)",
                        cv::Point(2450, fmt_ww_img.rows + 665),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, estimate3,
                        cv::Point(1675, fmt_ww_img.rows + 715),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, gt,
                        cv::Point(1629, fmt_ww_img.rows + 765),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);

            cv::putText(show_img_pano, std::string("err(deg.): ") + std::to_string(cv::norm(err_angles3)),
                        cv::Point(2600, fmt_ww_img.rows + 765),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);

            std::stringstream es2_1, es2_2, es2_3;
            es2_1.precision(2);
            es2_1.setf(std::ios::fixed);
            es2_1 << euler_angles2[0] * 180.0 / M_PI;
            es2_2.precision(2);
            es2_2.setf(std::ios::fixed);
            es2_2 << euler_angles2[1] * 180.0 / M_PI;
            es2_3.precision(2);
            es2_3.setf(std::ios::fixed);
            es2_3 << euler_angles2[2] * 180.0 / M_PI;
            std::string estimate2 = std::string("estimated (deg.):   ") + es2_1.str()
                                    + std::string("        ")
                                    + es2_2.str()
                                    + std::string("       ") + es2_3.str();
            fmt_wow_img.copyTo(
                    show_img_pano.rowRange(fmt_ww_img.rows + 800, 2 * fmt_ww_img.rows + 800).colRange(225,
                                                                                                      fmt_ww_img.cols +
                                                                                                      225));
            cv::putText(show_img_pano, "ours_fmt_wotrans_wrot",
                        cv::Point(325, 2 * fmt_ww_img.rows + 850),
                        cv::FONT_HERSHEY_SIMPLEX, 1.8,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, "yaw(z)",
                        cv::Point(550, 2 * fmt_ww_img.rows + 900),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, "pitch(y)",
                        cv::Point(770, 2 * fmt_ww_img.rows + 900),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, "roll(x)",
                        cv::Point(990, 2 * fmt_ww_img.rows + 900),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, estimate2,
                        cv::Point(225, 2 * fmt_ww_img.rows + 945),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, gt,
                        cv::Point(179, 2 * fmt_ww_img.rows + 990),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);

            cv::putText(show_img_pano, std::string("err(deg.): ") + std::to_string(cv::norm(err_angles2)),
                        cv::Point(1200, 2 * fmt_ww_img.rows + 990),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);


            std::stringstream es4_1, es4_2, es4_3;
            es4_1.precision(2);
            es4_1.setf(std::ios::fixed);
            es4_1 << euler_angles4[0] * 180.0 / M_PI;
            es4_2.precision(2);
            es4_2.setf(std::ios::fixed);
            es4_2 << euler_angles4[1] * 180.0 / M_PI;
            es4_3.precision(2);
            es4_3.setf(std::ios::fixed);
            es4_3 << euler_angles4[2] * 180.0 / M_PI;
            std::string estimate4 = std::string("estimated (deg.):   ") + es4_1.str()
                                    + std::string("        ")
                                    + es4_2.str()
                                    + std::string("       ") + es4_3.str();
            opt_wowo_img.copyTo(show_img_pano.rowRange(fmt_ww_img.rows + 800,
                                                       2 * fmt_ww_img.rows + 800).colRange(1675,
                                                                                           fmt_ww_img.cols + 1675));
            cv::putText(show_img_pano, "ours_opt_wotrans_worot",
                        cv::Point(1750, 2 * fmt_ww_img.rows + 850),
                        cv::FONT_HERSHEY_SIMPLEX, 1.8,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, "yaw(z)",
                        cv::Point(1990, 2 * fmt_ww_img.rows + 900),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, "pitch(y)",
                        cv::Point(2250, 2 * fmt_ww_img.rows + 900),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, "roll(x)",
                        cv::Point(2450, 2 * fmt_ww_img.rows + 900),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, estimate4,
                        cv::Point(1675, 2 * fmt_ww_img.rows + 945),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano, gt,
                        cv::Point(1629, 2 * fmt_ww_img.rows + 990),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);

            cv::putText(show_img_pano, std::string("err(deg.): ")+std::to_string(cv::norm(err_angles4)),
                        cv::Point(2600, 2 * fmt_ww_img.rows + 990),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);

            cv::putText(show_img_pano,
                        std::string("gt"),
                        cv::Point(1350, 750),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano,
                        std::string("A(deg.): ")
                        +std::to_string(sqrt(pow(eul_gt_t[1],2)+pow(eul_gt_t[2], 2))*180./M_PI),
                        cv::Point(1350, 800),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano,
                        std::string("A(pixel): ")
                        +std::to_string(sqrt(pow(eul_gt_t[1],2)+pow(eul_gt_t[2], 2))*1100/(2*M_PI)),
                        cv::Point(1350, 850),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano,
                        std::string("P(deg.): ")
                        +std::to_string((atan2(eul_gt_t[1], eul_gt_t[2]) + M_PI) * 180. / M_PI),
                        cv::Point(1350, 900),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);
            cv::putText(show_img_pano,
                        std::string("P(pixel): ")
                        +std::to_string((atan2(eul_gt_t[1], eul_gt_t[2]) + M_PI)*1100/(2*M_PI)),
                        cv::Point(1350, 950),
                        cv::FONT_HERSHEY_SIMPLEX, 1.,
                        cv::Scalar(41, 36, 33), 2);

            cv::namedWindow("show_img", CV_WINDOW_NORMAL);
            cv::imshow("show_img", show_img_pano);

            std::stringstream ss_name;
            ss_name << "show_img_pano" << i << ".jpg";
            cv::imwrite(ss_name.str(), show_img_pano);
            cv::waitKey(10);
        }
    }
//    out.close();

    return 0;
}
