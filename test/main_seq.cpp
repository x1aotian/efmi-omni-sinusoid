//
// Created by xuqw on 12/25/19.
//

#include "CSystem.h"
#include "misc.h"
#include <time.h>



int main(int argc, char ** argv){
    std::string path2params = std::string(argv[1]);
    cv::FileStorage fSettings(path2params, cv::FileStorage::READ);
    // read images
    auto path2imgs = (std::string) fSettings["path_imgs"];
    int startFrame = 1;
    int endFrame = 200;
    std::vector<std::string> imgNames;
    LoadImages(startFrame, endFrame, path2imgs, imgNames);
    std::cout << "imgNames size " << imgNames.size() << std::endl;

    // create system
//    // phone param
//    double cx = 670.0;
//    double cy = 350.0;
//    double lr = 230.0;
//    double sr = 120.0;
//    CCamModel ccam_model = CCamModel(cx, cy, lr, sr);
//
//    int edge = 110;
//    int step = 20;
//    double square_coeff = 1.8;
//    CTracking ctracker = CTracking(ccam_model, edge, step, square_coeff, WITHOUT_ROTATION_SCALING);

    // CVLIBS
//    double cx = 680.0;
//    double cy = 730.0;
//    double lr = 650.0;
//    double sr = 200.0;
//    CCamModel ccam_model = CCamModel(cx, cy, lr, sr);
//
//    int edge = 450;
//    int step = 50;
//    double square_coeff = 1.1;
//    CTracking ctracker = CTracking(ccam_model, edge, step, square_coeff, WITHOUT_ROTATION_SCALING);

    // OVMIS
//    double cx = 321.0;
//    double cy = 312.0;
//    double lr = 300.0;
//    double sr = 50.0;
//    CCamModel ccam_model = CCamModel(cx, cy, lr, sr);
//
//    int edge = 250;
//    int step = 25;
//    double square_coeff = 1.0;
//    CTracking ctracker = CTracking(ccam_model, edge, step, square_coeff, WITHOUT_TRANS_WITHOUT_ROT);

    // SVMIS
//    double width = 1920.0;
//    double height = 960.0;
//    CCamModel ccam_model = CCamModel(width, height);
//
//    int edge = 960;
//    int step = 200;
//    double square_coeff = 1.0;
//    CTracking ctracker = CTracking(ccam_model, edge, step, square_coeff, WITHOUT_ROTATION_SCALING, PANO);

    auto cx = (double) fSettings["cx"];
    auto cy = (double) fSettings["cy"];
    auto lr = (double) fSettings["lr"];
    auto sr = (double) fSettings["sr"];
    CCamModel ccam_model = CCamModel(cx, cy, lr, sr);
    auto edge = (int) fSettings["edge"];
    auto step = (int) fSettings["step"];
    auto square_coeff = (double) fSettings["square_coeff"];
    auto method = (int) fSettings["reg_method"];
    auto reg_method = Reg_Method(method);
    CTracking ctracker = CTracking(ccam_model, edge, step, square_coeff, reg_method);
    auto match_flag = (int) fSettings["match_method"];
    auto match_method = Match_Method(match_flag);
    CSystem csystem = CSystem(ccam_model, ctracker, match_method);

    // tracking
    auto result_path = (std::string) fSettings["result_path"];
    std::cout << "result path: " << result_path << std::endl;
    std::ofstream out(result_path);
    double sum_time = 0.0;
    for (int i = 0; i < imgNames.size() - 1;i+=1) {
        std::cout << "image" << i << std::endl;
        cv::Mat oimg = cv::imread(imgNames[i], cv::IMREAD_GRAYSCALE);

        clock_t start_time, end_time;
        start_time = clock();
        csystem.Tracking(i, oimg);
        end_time = clock();
        sum_time += (double) (end_time - start_time);
        if(csystem.kf_flag_){
//            Eigen::Matrix3d R_eigen;
//            cv::cv2eigen(csystem.T_, R_eigen);
//            Eigen::Vector3d euler_angles = R_eigen.eulerAngles(2, 1, 0); // yaw pitch roll
//            out << i << " " << euler_angles.transpose() << std::endl;
            cv::Mat pose_R = csystem.T_.inv();
            cv::Vec3d euler_angles = rotationMatrixToEulerAngles(pose_R); // yaw pitch roll
            out << i << " " << euler_angles[0] << " " << euler_angles[1] << " " << -euler_angles[2] << std::endl;
            std::cout << "sum angle: " << (euler_angles[0] * 180 / M_PI) << " "
                      << (euler_angles[1] * 180 / M_PI) << " "
                      << (euler_angles[2] * 180 / M_PI)<< std::endl;
        }
    }
    std::cout << "average processing time is " << sum_time/(double)(imgNames.size() - 1)/CLOCKS_PER_SEC << std::endl;
    out.close();

    return 0;
}
