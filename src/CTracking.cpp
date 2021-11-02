//
// Created by xuqw on 12/6/19.
//

#include "misc_1.h"
#include "misc_0.h"
#include "CTracking.h"

#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cmath>

using namespace jacobs_robotics;

CTracking::CTracking() {
    ccam_model_ = NULL;
    cf1_ = NULL;
    cf2_ = NULL;
    R_ = cv::Mat::eye(3, 3, CV_64F);
    method_ = WITH_TRANS_WITHOUT_ROT;
    input_type_ = OMNI;
    square_coeff_ = 1.0;
    idxs_.clear();
    delta_us_.clear();
    delta_vs_.clear();
    thetas_.clear();
    scalings_.clear();
    peak_u_ = 0;
    peak_v_ = 0;
    peak_theta_ = 0;
    peak_s_ = 0;
}

CTracking::CTracking(const CTracking &ctracker) :
        ccam_model_(ctracker.ccam_model_),
        cf1_(ctracker.cf1_),
        cf2_(ctracker.cf2_),
        R_(ctracker.R_),
        method_(ctracker.method_),
        idxs_(ctracker.idxs_),
        delta_us_(ctracker.delta_us_),
        delta_vs_(ctracker.delta_vs_),
        thetas_(ctracker.thetas_),
        scalings_(ctracker.scalings_),
        peak_u_(ctracker.peak_u_),
        peak_v_(ctracker.peak_v_),
        peak_theta_(ctracker.peak_theta_),
        peak_s_(ctracker.peak_s_),
        input_type_(ctracker.input_type_),
        square_coeff_(ctracker.square_coeff_) {
}

CTracking::CTracking(CCamModel &ccam_model,
                     int edge,
                     int step, Reg_Method method) {
    ccam_model_ = new CCamModel(ccam_model);
    cf1_ = new CFrame(edge, step, ccam_model);
    cf2_ = new CFrame(edge, step, ccam_model);
    R_ = cv::Mat::eye(3, 3, CV_64F);
    square_coeff_ = 1.0;
    method_ = method;
    input_type_ = OMNI;
    idxs_.clear();
    delta_us_.clear();
    delta_vs_.clear();
    thetas_.clear();
    scalings_.clear();
    peak_u_ = 0;
    peak_v_ = 0;
    peak_theta_ = 0;
    peak_s_ = 0;
}

CTracking::CTracking(CCamModel &ccam_model,
                     int edge,
                     int step,
                     double square_coeff, Reg_Method method) {
    ccam_model_ = new CCamModel(ccam_model);
    input_type_ = OMNI;
    cf1_ = new CFrame(edge, step, ccam_model, input_type_);
    cf2_ = new CFrame(edge, step, ccam_model, input_type_);
    square_coeff_ = square_coeff;
    R_ = cv::Mat::eye(3, 3, CV_64F);
    method_ = method;
    idxs_.clear();
    delta_us_.clear();
    delta_vs_.clear();
    thetas_.clear();
    scalings_.clear();
    peak_u_ = 0;
    peak_v_ = 0;
    peak_theta_ = 0;
    peak_s_ = 0;
}

CTracking::CTracking(CCamModel &ccam_model,
                     int edge,
                     int step,
                     double square_coeff,
                     Reg_Method method,
                     InputType input_type) {
    ccam_model_ = new CCamModel(ccam_model);
    input_type_ = input_type;
    cf1_ = new CFrame(edge, step, ccam_model, input_type);
    cf2_ = new CFrame(edge, step, ccam_model, input_type);
    square_coeff_ = square_coeff;
    R_ = cv::Mat::eye(3, 3, CV_64F);
    method_ = method;
    idxs_.clear();
    delta_us_.clear();
    delta_vs_.clear();
    thetas_.clear();
    scalings_.clear();
    peak_u_ = 0;
    peak_v_ = 0;
    peak_theta_ = 0;
    peak_s_ = 0;
}

CTracking::CTracking(CCamModel &ccam_model,
                     cv::Mat &oimg1,
                     cv::Mat &oimg2,
                     int edge,
                     int step, Reg_Method method) {
    ccam_model_ = new CCamModel(ccam_model);
    cf1_ = new CFrame(oimg1, edge, step, ccam_model);
    cf2_ = new CFrame(oimg2, edge, step, ccam_model);
    R_ = cv::Mat::eye(3, 3, CV_64F);
    square_coeff_ = 1.0;
    method_ = method;
    idxs_.clear();
    delta_us_.clear();
    delta_vs_.clear();
    thetas_.clear();
    scalings_.clear();
    peak_u_ = 0;
    peak_v_ = 0;
    peak_theta_ = 0;
    peak_s_ = 0;
}

CTracking::CTracking(CCamModel &ccam_model,
                     cv::Mat &oimg1,
                     cv::Mat &oimg2,
                     int edge,
                     int step,
                     double square_coeff,
                     Reg_Method method) {
    ccam_model_ = new CCamModel(ccam_model);
    cf1_ = new CFrame(oimg1, edge, step, ccam_model);
    cf2_ = new CFrame(oimg2, edge, step, ccam_model);
    R_ = cv::Mat::eye(3, 3, CV_64F);
    square_coeff_ = square_coeff;
    method_ = method;
    idxs_.clear();
    delta_us_.clear();
    delta_vs_.clear();
    thetas_.clear();
    scalings_.clear();
    peak_u_ = 0;
    peak_v_ = 0;
    peak_theta_ = 0;
    peak_s_ = 0;
}

void CTracking::SetImgs(cv::Mat &oimg1, cv::Mat &oimg2) {
    cf1_->SetImg(oimg1);
    cf2_->SetImg(oimg2);
}

void CTracking::SetFirstImg(cv::Mat &oimg1) {
    cf1_->SetImg(oimg1);
}

void CTracking::SetFirstImg(int idx, cv::Mat &oimg1) {
    cf1_->SetImg(idx, oimg1);
}

void CTracking::SetSecondImg(cv::Mat &oimg2) {
    cf2_->SetImg(oimg2);
}

void CTracking::SetSecondImg(int idx, cv::Mat &oimg2) {
    cf2_->SetImg(idx, oimg2);
}

void CTracking::SetNewImg(cv::Mat &oimg2) {
    cf1_ = new CFrame(*cf2_);
    cf2_->SetImg(oimg2);
}

void CTracking::RenewLastFrame() {
    cf1_ = new CFrame(*cf2_);
}

void CTracking::GetMotionVec() {
    idxs_.clear();
    delta_us_.clear();
    delta_vs_.clear();
    thetas_.clear();
    scalings_.clear();
    // calculate motion vector with iFMI
    std::vector<cv::Mat> subimg_set1 = cf1_->GetSubframes();
    std::vector<cv::Mat> subimg_set2 = cf2_->GetSubframes();
    std::vector<cv::Point2d> corners = cf1_->GetCorners();

    for (int i = 0; i < subimg_set1.size(); ++i) {
        //Convert to QImage
        QImage qImg1, qImg2;
        qImg1 = QImage((uchar *) subimg_set1[i].data,
                       subimg_set1[i].cols,
                       subimg_set1[i].rows,
                       subimg_set1[i].step,
                       QImage::Format_Grayscale8);
        qImg2 = QImage((uchar *) subimg_set2[i].data,
                       subimg_set2[i].cols,
                       subimg_set2[i].rows,
                       subimg_set2[i].step,
                       QImage::Format_Grayscale8);
        // Create fmiImage respectively
        FMIImage *img1 = fromQImage(qImg1);
        FMIImage *img2 = fromQImage(qImg2);
        // Register two images
        std::vector<FMIResult> results;
        // adjust threshold for SNR
//         FMIRegistration fmiRegister(qImg1.height(), qImg1.height(), 0.05, 1.0);
        FMIRegistration fmiRegister(qImg1.height(), qImg1.height(), 0.00, 0.0);
//        FMIRegistration fmiRegister(qImg1.height(), qImg1.height(), 0.00, 0.5);
        bool flag = fmiRegister.registerImages(img1, img2, results, false);
        // destructor
        // TODO: Whether destructor will influence the result
        delete img1;
        img1 = NULL;

        delete img2;
        img2 = NULL;

        if (flag) {
            idxs_.push_back(corners[i].x);
            delta_us_.push_back(results[0].transX);
            delta_vs_.push_back(results[0].transY * square_coeff_); // square pixel coeff
            thetas_.push_back(results[0].rotationRad);
            scalings_.push_back(results[0].scale);
        }
    }

    // write to file
    if (DEBUG) {
        std::stringstream idxs_name;
        idxs_name << "idxs.txt";
        WriteVector(idxs_name.str(), idxs_);
        std::stringstream delta_xs_name;
        delta_xs_name << "delta_xs.txt";
        WriteVector(delta_xs_name.str(), delta_us_);
        std::stringstream delta_ys_name;
        delta_ys_name << "delta_ys.txt";
        WriteVector(delta_ys_name.str(), delta_vs_);
        std::stringstream theta_name;
        theta_name << "thetas.txt";
        WriteVector(theta_name.str(), thetas_);
    }
}

void CTracking::GetMotionVecEfmi() {

    idxs_.clear();
    delta_us_.clear();
    delta_vs_.clear();
    thetas_.clear();
    scalings_.clear();

    // calculate motion vector with iFMI
    std::vector<cv::Mat> subimg_set1 = cf1_->GetSubframes();
    std::vector<cv::Mat> subimg_set2 = cf2_->GetSubframes();
    std::vector<cv::Point2d> corners = cf1_->GetCorners();

    // define some variables
    double du = 0.;
    double dv = 0.;

    double st_ab = 1.;
    double st_bc = 1.;
    double s_abs_a = 1.;
    double s_abs_b = 1.;
    double s_abs_c = 1.;
    double len_a = 1.;
    double len_b = 1.;
    double len_c = 1.;

    // Begin Loops
    for (int i = 0; i < subimg_set1.size()-1; ++i) {
        std::cout << "----------------" << i << "--------------------" << std::endl;

        std::vector<double> transline_a, scalesline_a;
        std::vector<double> transline_b, scalesline_b;
        std::vector<double> transline_c, scalesline_c;

        std::vector<FMIResult> results_a, results_b, results_c;

        //Convert to QImage
        QImage qImg1, qImg2, qImg3, qImg4;

        qImg1 = QImage((uchar *) subimg_set1[i].data,
                       subimg_set1[i].cols,
                       subimg_set1[i].rows,
                       subimg_set1[i].step,
                       QImage::Format_Grayscale8);
        qImg2 = QImage((uchar *) subimg_set2[i].data,
                       subimg_set2[i].cols,
                       subimg_set2[i].rows,
                       subimg_set2[i].step,
                       QImage::Format_Grayscale8);
        qImg3 = QImage((uchar *) subimg_set1[i+1].data,
                       subimg_set1[i+1].cols,
                       subimg_set1[i+1].rows,
                       subimg_set1[i+1].step,
                       QImage::Format_Grayscale8);
        qImg4 = QImage((uchar *) subimg_set2[i+1].data,
                       subimg_set2[i+1].cols,
                       subimg_set2[i+1].rows,
                       subimg_set2[i+1].step,
                       QImage::Format_Grayscale8);

        // Create fmiImage respectively
        FMIImage *img1 = fromQImage(qImg1);
        FMIImage *img2 = fromQImage(qImg2);
        FMIImage *img3 = fromQImage(qImg3);
        FMIImage *img4 = fromQImage(qImg4);

        // adjust threshold for SNR
        FMIRegistration fmiRegister(qImg1.height(), qImg1.height(), 2, 0.00, 0.0);

        // Run the efmi algorithm
        bool flag_a = fmiRegister.registerMultiTransMultiScale(img1, img2, results_a, scalesline_a, transline_a, false);
        bool flag_b = fmiRegister.registerMultiTransMultiScale(img1, img3, results_b, scalesline_b, transline_b, false);
        bool flag_c = fmiRegister.registerMultiTransMultiScale(img3, img4, results_c, scalesline_c, transline_c, false);

        len_a = sqrt(pow(results_a[0].transX, 2) + pow(results_a[0].transY, 2));
        len_b = sqrt(pow(results_b[0].transX, 2) + pow(results_b[0].transY, 2));
        len_c = sqrt(pow(results_c[0].transX, 2) + pow(results_c[0].transY, 2));

        if (i == 0 && flag_a){
            du = results_a[0].transX ;
            dv = results_a[0].transY ;
            delta_us_.push_back(du);
            delta_vs_.push_back(dv * square_coeff_);
            thetas_.push_back(results_a[0].rotationRad);
            idxs_.push_back(corners[i].x);
            scalings_.push_back(1);
            s_abs_a = len_a;
        }

        st_ab = search1DResize(transline_a, transline_b);
        st_bc = search1DResize(transline_b, transline_c);
        std::cout << "st_ab: "<< st_ab << " , " << "st_bc: " << st_bc << std::endl;

        s_abs_b = s_abs_a * st_ab;
        s_abs_c = s_abs_b * st_bc;

        du = s_abs_c * results_c[0].transX / len_c ;
        dv = s_abs_c * results_c[0].transY / len_c ;

        if (flag_a && flag_b && flag_c) {
            delta_us_.push_back(du);
            delta_vs_.push_back(dv * square_coeff_);
            thetas_.push_back(results_c[0].rotationRad);
            idxs_.push_back(corners[i + 1].x);
            scalings_.push_back(s_abs_c);
        }

        s_abs_a = s_abs_c;
        std::cout << "s_abs_a: " << s_abs_a << std::endl;
        // destructor
    }

    // write to file
    if (DEBUG) {
        std::stringstream idxs_name;
        idxs_name << "idxs.txt";
        WriteVector(idxs_name.str(), idxs_);
        std::stringstream delta_xs_name;
        delta_xs_name << "delta_xs.txt";
        WriteVector(delta_xs_name.str(), delta_us_);
        std::stringstream delta_ys_name;
        delta_ys_name << "delta_ys.txt";
        WriteVector(delta_ys_name.str(), delta_vs_);
        std::stringstream theta_name;
        theta_name << "thetas.txt";
        WriteVector(theta_name.str(), thetas_);
    }
}

void CTracking::GetMotionTransFlow( std::vector< std::vector<double> > & set_dusline, std::vector< std::vector<double> > & set_dvsline) {

    idxs_.clear();
    delta_us_.clear();
    delta_vs_.clear();
    thetas_.clear();
    scalings_.clear();

    std::vector<cv::Mat> subimg_set1 = cf1_->GetSubframes();
    std::vector<cv::Mat> subimg_set2 = cf2_->GetSubframes();
    std::vector<cv::Point2d> corners = cf1_->GetCorners();

    // save sub_images
    /*
    for (int i=0; i<subimg_set1.size(); ++i){
        std::string s = "./dataset/PanoraMIS_Sequence6/sub_images/sub";
        std::string name = s + std::to_string(i) + ".jpeg";
        cv::imwrite(name.c_str(), subimg_set1[i]);
    }
     */


    //cv::imshow("1", cf1->GetPanoImg());
    //cv::waitKey(0);
    //std::cout << subimg_set1.size() << std::endl;

    // subimg_set.size() = 117
    for (int i = 0; i < subimg_set1.size(); ++i) {
        std::cout << "-------------" << i << "-------------" << std::endl;

        QImage qImg1, qImg2;
        qImg1 = QImage((uchar *) subimg_set1[i].data,
                       subimg_set1[i].cols,
                       subimg_set1[i].rows,
                       subimg_set1[i].step,
                       QImage::Format_Grayscale8);
        qImg2 = QImage((uchar *) subimg_set2[i].data,
                       subimg_set2[i].cols,
                       subimg_set2[i].rows,
                       subimg_set2[i].step,
                       QImage::Format_Grayscale8);

        FMIImage *img1 = fromQImage(qImg1);
        FMIImage *img2 = fromQImage(qImg2);

        std::vector<FMIResult> results;
        auto resolution = static_cast<unsigned int>(qImg1.height());
        auto polarlogresolution = static_cast<unsigned int>(qImg1.height());
        std::vector<double> scalesline, transline;
        double maxAngle;

        FMIRegistration fmiRegister(qImg1.height(), qImg1.height(), 2,0.00, 0.0);

        //---------------modified function HERE----------------//
        // MaxAngle here to store the angle of sector with max energy in the results[0]'s phase shift diagram (after rerotate and rescale)
        bool flag = fmiRegister.registerMultiTransMultiScale_andMaxAngle(img1, img2, results, scalesline, transline, maxAngle, false);

        std::vector<double> tmp_dusline( resolution + 1, 0.); // +1
        std::vector<double> tmp_dvsline( resolution + 1, 0.);
        int tmp_cen = (resolution + 1) / 2;

        if (i == 20){
            std::cout << "NOW" << std::endl;
        }

        if (flag) {
            idxs_.push_back(corners[i].x);
            delta_us_.push_back(results[0].transX);
            delta_vs_.push_back(results[0].transY * square_coeff_); // square pixel coeff
            //std::cout << results[0].rotationRad << std::endl;
            std::cout << "atan(y,x): " << atan2(results[0].transY,results[0].transX) << std::endl;
            std::cout << "maxAngle: " << maxAngle << std::endl;
            thetas_.push_back(results[0].rotationRad);
            scalings_.push_back(results[0].scale);

            //double maxAngle = atan2(results[0].transY,results[0].transX);
            //maxAngle = results[0].rotationRad;

            int transline_cen = transline.size()/2;
            for (int j=0; j<transline_cen; j++){
                tmp_dusline[ tmp_cen - round(cos(maxAngle) * (transline_cen-j)) ] += transline[j];
                tmp_dvsline[ tmp_cen - round(sin(maxAngle) * (transline_cen-j)) ] += transline[j];
            }
            for (int j=transline_cen; j<transline.size(); j++) {

                tmp_dusline[tmp_cen + round(cos(maxAngle) * (-transline_cen + j))] += transline[j];
                tmp_dvsline[tmp_cen + round(sin(maxAngle) * (-transline_cen + j))] += transline[j];
            }

//            std::cout << tmp_cen + round(cos(maxAngle) * (-transline_cen + transline.size()-1)) << std::endl;
//            std::cout << tmp_cen + round(sin(maxAngle) * (-transline_cen + transline.size()-1)) << std::endl;

            std::cout << "transline.size(): " << transline.size() << std::endl;

            set_dusline.push_back(tmp_dusline);
            set_dvsline.push_back(tmp_dvsline);

            std::stringstream idxs_name;
            idxs_name << "./result/22_23/original_have/idxs.txt";
            WriteVector(idxs_name.str(), idxs_);
            std::stringstream delta_xs_name;
            delta_xs_name << "./result/22_23/original_have/delta_xs.txt";
            WriteVector(delta_xs_name.str(), delta_us_);
            std::stringstream delta_ys_name;
            delta_ys_name << "./result/22_23/original_have/delta_ys.txt";
            WriteVector(delta_ys_name.str(), delta_vs_);
            std::stringstream theta_name;
            theta_name << "./result/22_23/original_have/thetas.txt";
            WriteVector(theta_name.str(), thetas_);
        }

    }

    std::cout << "END" << std::endl;
}

void CTracking::GetMotionVecOptical() {
    idxs_.clear();
    delta_us_.clear();
    delta_vs_.clear();
    thetas_.clear();
    scalings_.clear();
    // calculate motion vector with optical flow
    // TODO: compare optical flow on pano image and omni/fish image
    cv::Mat pano_img_1 = cf1_->GetPanoImg();
    cv::Mat pano_img_2 = cf2_->GetPanoImg();

    std::vector<cv::Point2f> pts1, pts2;
    cv::goodFeaturesToTrack(pano_img_1, pts1, 400, 0.3, 7, cv::Mat(), 7, false, 0.04);
    //add center points
    double width = ccam_model_->GetWidth();
    double height = ccam_model_->GetHeight();

    int step = cf1_->GetStep();
    for (int i = step / 2; i < width; i += step) {
        pts1.push_back(cv::Point2d(i, height / 2));
    }

    std::vector<uchar> status;
    std::vector<float> err;
    cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
    cv::calcOpticalFlowPyrLK(pano_img_1,
                             pano_img_2,
                             pts1,
                             pts2,
                             status, err, cv::Size(15, 15), 2, criteria);

    for (size_t j = 0; j < status.size(); j++) {
        if (status[j]) {
            double idx = pts1[j].x;
            double delta_u = pts1[j].x - pts2[j].x;
            double delta_v = pts1[j].y - pts2[j].y;
            idxs_.push_back(idx);
            delta_us_.push_back(delta_u);
            delta_vs_.push_back(delta_v * square_coeff_);
        }
    }

    // write to file
    if (DEBUG) {
        std::stringstream idxs_name;
        idxs_name << "idxs.txt";
        WriteVector(idxs_name.str(), idxs_);
        std::stringstream delta_xs_name;
        delta_xs_name << "delta_xs.txt";
        WriteVector(delta_xs_name.str(), delta_us_);
        std::stringstream delta_ys_name;
        delta_ys_name << "delta_ys.txt";
        WriteVector(delta_ys_name.str(), delta_vs_);
    }
}

void CTracking::ImgRegister() {
    std::cout << "num of correspondences: " << idxs_.size() << std::endl;
    // Solver
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-8;

    if (method_ == WITH_TRANS_WITH_ROT) {
        // fitting delta_v and thetas to get roll and pitch
        double param_v[3] = {0.0, 0.0, 0.0};
        double T = ccam_model_->GetWidth();
        double coeff = T / (2 * M_PI) * 0.1;
        ceres::Problem problem_v;
        for (int i = 0; i < idxs_.size(); ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<WithRotationCostFunction, 1, 3>
                            (new WithRotationCostFunction(idxs_[i], delta_vs_[i] - coeff * thetas_[i], T, coeff));
            problem_v.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5), param_v);
        }
        ceres::Solver::Summary summary_v;
        ceres::Solve(options, &problem_v, &summary_v);

        std::cout << "delta_vs estimated Amplitude, phase, offset = ";
        for (auto a:param_v) {
            std::cout << a << " ";
        }
        std::cout << std::endl;

        // fitting delta_u to get yaw
        double param_u[3] = {0.0, 0.0, 0.0};
        ceres::Problem problem_u;
        for (int i = 0; i < idxs_.size(); ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<TranslationCostFunction, 1, 3>
                            (new TranslationCostFunction(idxs_[i], delta_us_[i], T));
            problem_u.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5), param_u);
        }

        ceres::Solver::Summary summary_u;
        ceres::Solve(options, &problem_u, &summary_u);

        std::cout << "delta_us estimated Amplitude, phase, offset = ";
        for (auto a:param_u) {
            std::cout << a << " ";
        }
        std::cout << std::endl;

        peak_u_ = fabs(param_u[0] * T / (2 * M_PI));
        peak_v_ = fabs(param_v[0] * T / (2 * M_PI));
        // convert to transformation
        double angle_Rxy = param_v[0];
        // TODO: double check the sign of param_v[1]
        double axis_Rxy = -param_v[1];
        double gamma = T / (2 * M_PI);
        double yaw = param_u[2] / gamma;
        double roll = angle_Rxy * cos(axis_Rxy);
        double pitch = angle_Rxy * sin(axis_Rxy);
//        double roll = angle_Rxy / (sqrt(1 + pow(tan(axis_Rxy), 2)));
//        double pitch = roll * tan(axis_Rxy);
        std::cout << "YPR = " << (yaw * 180.0 / M_PI) << ", "
                  << (pitch * 180.0 / M_PI) << ", "
                  << (roll * 180.0 / M_PI) << std::endl;

        Eigen::Matrix3d R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
        cv::eigen2cv(R, R_);
        if (DEBUG) {
            // display
            cv::Mat show_img = display_wtrans_wrot(100, T, idxs_, delta_us_, thetas_, param_u, delta_vs_, param_v);
            cv::namedWindow("debug", cv::WINDOW_NORMAL);
            cv::imshow("debug", show_img);
            cv::imwrite("wtrans_wrot.jpg", show_img);
            cv::waitKey(100);
        }

    } else if (method_ == WITH_TRANS_WITHOUT_ROT) {
        // fitting delta_v
        double param_v[3] = {0.0, 0.0, 0.0};
        double T = ccam_model_->GetWidth();
        ceres::Problem problem_v;
        for (int i = 0; i < idxs_.size(); ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<TranslationCostFunction, 1, 3>
                            (new TranslationCostFunction(idxs_[i], delta_vs_[i], T));
            problem_v.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5), param_v);
        }

        ceres::Solver::Summary summary_v;
        ceres::Solve(options, &problem_v, &summary_v);

        // display results
//        std::cout << summary_v.FullReport() << std::endl;
        std::cout << "delta_vs estimated Amplitude, phase, offset = ";
        for (auto a:param_v) {
            std::cout << a << " ";
        }
        std::cout << std::endl;

        // fitting delta_u
        double param_u[3] = {0.0, 0.0, 0.0};
        ceres::Problem problem_u;
        for (int i = 0; i < idxs_.size(); ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<TranslationCostFunction, 1, 3>
                            (new TranslationCostFunction(idxs_[i], delta_us_[i], T));
            problem_u.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5), param_u);
        }

        ceres::Solver::Summary summary_u;
        ceres::Solve(options, &problem_u, &summary_u);

        std::cout << "delta_us estimated Amplitude, phase, offset = ";
        for (auto a:param_u) {
            std::cout << a << " ";
        }
        std::cout << std::endl;

        peak_u_ = fabs(param_u[0] * T / (2 * M_PI));
        peak_v_ = fabs(param_v[0] * T / (2 * M_PI));
        // convert to transformation
        double angle_Rxy = param_v[0];
//        // TODO: double check the sign of param_v[1]
        double axis_Rxy = -param_v[1];
        double gamma = T / (2 * M_PI);
        double yaw = param_u[2] / gamma;

        double roll = angle_Rxy * cos(axis_Rxy);
        double pitch = angle_Rxy * sin(axis_Rxy);
//        double roll = angle_Rxy / (sqrt(1 + pow(tan(axis_Rxy), 2)));
//        double pitch = roll * tan(axis_Rxy);
        std::cout << "YPR = " << (yaw * 180.0 / M_PI) << ", "
                  << (pitch * 180.0 / M_PI) << ", "
                  << (roll * 180.0 / M_PI) << std::endl;

        Eigen::Matrix3d R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();

//        Eigen::Vector3d euler_angles = R.eulerAngles(2, 1, 0); // yaw pitch roll
//        std::cout << "yaw, pitch, roll = " << euler_angles.transpose() << std::endl;
        cv::eigen2cv(R, R_);

        if (DEBUG) {
            // display
            cv::Mat show_img = display_wtrans_worot(100, T, idxs_, delta_us_, param_u, delta_vs_, param_v);
            cv::namedWindow("debug", cv::WINDOW_NORMAL);
            cv::imshow("debug", show_img);
            cv::imwrite("wtrans_worot.jpg", show_img);
            cv::waitKey(100);
        }

    } else if (method_ == WITHOUT_TRANS_WITH_ROT) {
        // fitting delta_v
        double param_v[2] = {0.0, 0.0};
        double T = ccam_model_->GetWidth();
        double coeff = T / (2 * M_PI) * 0.1;
        ceres::Problem problem_v;
        for (int i = 0; i < idxs_.size(); ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<WithoutTransWithRotCostFunction, 1, 2>
                            (new WithoutTransWithRotCostFunction(idxs_[i], delta_vs_[i] - coeff * thetas_[i], T,
                                                                 coeff));
            problem_v.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5), param_v);
        }

        ceres::Solver::Summary summary_v;
        ceres::Solve(options, &problem_v, &summary_v);

        // display results
//        std::cout << summary_v.FullReport() << std::endl;
        std::cout << "delta_vs estimated Amplitude, phase, offset = ";
        for (auto a:param_v) {
            std::cout << a << " ";
        }
        std::cout << std::endl;

        // fitting delta_u
        double param_u[1] = {0.0};
        ceres::Problem problem_u;
        for (int i = 0; i < idxs_.size(); ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<OnlyYawCostFunction, 1, 1>
                            (new OnlyYawCostFunction(delta_us_[i]));
            problem_u.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5), param_u);
        }

        ceres::Solver::Summary summary_u;
        ceres::Solve(options, &problem_u, &summary_u);

        std::cout << "delta_us estimated Amplitude, phase, offset = ";
        for (auto a:param_u) {
            std::cout << a << " ";
        }
        std::cout << std::endl;

        peak_u_ = fabs(param_u[0] * T / (2 * M_PI));
        peak_v_ = fabs(param_v[0] * T / (2 * M_PI));
        // convert to transformation
        double angle_Rxy = param_v[0];
        // TODO: double check the sign of param_v[1]
        double axis_Rxy = -param_v[1];
        double gamma = T / (2 * M_PI);
        double yaw = param_u[0] / gamma;

        double roll = angle_Rxy * cos(axis_Rxy);
        double pitch = angle_Rxy * sin(axis_Rxy);
//        double roll = angle_Rxy / (sqrt(1 + pow(tan(axis_Rxy), 2)));
//        double pitch = roll * tan(axis_Rxy);
        std::cout << "YPR = " << (yaw * 180.0 / M_PI) << ", "
                  << (pitch * 180.0 / M_PI) << ", "
                  << (roll * 180.0 / M_PI) << std::endl;

        Eigen::Matrix3d R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
        cv::eigen2cv(R, R_);

        if (DEBUG) {
            // display
            cv::Mat show_img = display_wotrans_wrot(100, T, idxs_, delta_us_, thetas_, param_u, delta_vs_, param_v);
            cv::namedWindow("debug", cv::WINDOW_NORMAL);
            cv::imshow("debug", show_img);
            cv::imwrite("wotrans_wrot.jpg", show_img);
            cv::waitKey(100);
        }
    } else if (method_ == WITHOUT_TRANS_WITHOUT_ROT) {
        // fitting delta_v
        double param_v[2] = {0.0, 0.0};
        double T = ccam_model_->GetWidth();
        ceres::Problem problem_v;
        for (int i = 0; i < idxs_.size(); ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<WithoutTransCostFunction, 1, 2>
                            (new WithoutTransCostFunction(idxs_[i], delta_vs_[i], T));
            problem_v.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5), param_v);
        }

        ceres::Solver::Summary summary_v;
        ceres::Solve(options, &problem_v, &summary_v);

        // display results
//        std::cout << summary_v.FullReport() << std::endl;
        std::cout << "delta_vs estimated Amplitude, phase, offset = ";
        for (auto a:param_v) {
            std::cout << a << " ";
        }
        std::cout << std::endl;

        // fitting delta_u
        double param_u[1] = {0.0};
        ceres::Problem problem_u;
        for (int i = 0; i < idxs_.size(); ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<OnlyYawCostFunction, 1, 1>
                            (new OnlyYawCostFunction(delta_us_[i]));
            problem_u.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5), param_u);
        }

        ceres::Solver::Summary summary_u;
        ceres::Solve(options, &problem_u, &summary_u);

        std::cout << "delta_us estimated Amplitude, phase, offset = ";
        for (auto a:param_u) {
            std::cout << a << " ";
        }
        std::cout << std::endl;

        peak_u_ = fabs(param_u[0] * T / (2 * M_PI));
        peak_v_ = fabs(param_v[0] * T / (2 * M_PI));
        // convert to transformation
        double angle_Rxy = param_v[0];
        // TODO: double check the sign of param_v[1]
        double axis_Rxy = -param_v[1];
        double gamma = T / (2 * M_PI);
        double yaw = param_u[0] / gamma;

        double roll = angle_Rxy * cos(axis_Rxy);
        double pitch = angle_Rxy * sin(axis_Rxy);
//        double roll = angle_Rxy / (sqrt(1 + pow(tan(axis_Rxy), 2)));
//        double pitch = roll * tan(axis_Rxy);
        std::cout << "YPR = " << (yaw * 180.0 / M_PI) << ", "
                  << (pitch * 180.0 / M_PI) << ", "
                  << (roll * 180.0 / M_PI) << std::endl;

        Eigen::Matrix3d R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
        cv::eigen2cv(R, R_);
        if (DEBUG) {
            // display
            cv::Mat show_img = display_wotrans_worot(100, T, idxs_, delta_us_, param_u, delta_vs_, param_v);
            cv::namedWindow("debug", cv::WINDOW_NORMAL);
            cv::imshow("debug", show_img);
            cv::imwrite("wotrans_worot.jpg", show_img);
            cv::waitKey(100);
        }
    } else {
        R_ = cv::Mat::eye(3, 3, CV_64F);
    }
}

std::vector<double> CTracking::FittingWTransWRot(std::vector<double> &idx,
                                                 std::vector<double> &delta_v,
                                                 std::vector<double> &theta) {
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-8;

    double T = ccam_model_->GetWidth();
    double coeff = T / (2 * M_PI) * 0.1;

    int max_iter = 50;
    int cnt = 10;
    double last_min_cost = INFINITY;
    double last_cost = INFINITY;
    double param[3] = {0., 0., 0.};
    for (int k = 0; k < max_iter; ++k) {
        std::vector<double> sample_i(cnt);
        std::vector<double> sample_y(cnt);
        std::vector<double> sample_theta(cnt);
        for (int j = 0; j < cnt; ++j) {
            int ind = rand() % idx.size();
            sample_i[j] = idx[ind];
            sample_y[j] = delta_v[ind];
            sample_theta[j] = theta[ind];
        }
        double sub_param[3] = {0.0, 0.0, 0.0};
        ceres::Problem sub_problem;
        for (int i = 0; i < cnt; ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<WithRotationCostFunction, 1, 3>
                            (new WithRotationCostFunction(sample_i[i], sample_y[i] - coeff * sample_theta[i], T,
                                                          coeff));
            sub_problem.AddResidualBlock(cost_function, NULL, sub_param);
        }
        ceres::Solver::Summary sub_summary;
        ceres::Solve(options, &sub_problem, &sub_summary);

        //calculate distance
        double cost = 0.;
        for (int i = 0; i < cnt; ++i) {
            cost += fabs(delta_v[i] + coeff * theta[i] -
                         (sub_param[0] * std::sin((2 * M_PI / T) * idx[i] + sub_param[1]) + sub_param[2]));
        }

        if (cost < last_cost) {
            last_cost = cost;
            param[0] = sub_param[0];
            param[1] = sub_param[1];
            param[2] = sub_param[2];
            if (fabs(last_min_cost - last_cost) < 1e-5) {
                last_cost = cost;
                param[0] = sub_param[0];
                param[1] = sub_param[1];
                param[2] = sub_param[2];
                break;
            }
            last_min_cost = last_cost;
        }
    }

    std::vector<double> out_param(3);
    for (int i = 0; i < out_param.size(); ++i) {
        out_param[i] = param[i];
    }
    return out_param;
}

std::vector<double> CTracking::FittingWoTransWRot(std::vector<double> &idx,
                                                  std::vector<double> &delta_v,
                                                  std::vector<double> &theta) {
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-8;

    double T = ccam_model_->GetWidth();
    double coeff = T / (2 * M_PI) * 0.1;

    int max_iter = 50;
    int cnt = 10;
    double last_min_cost = INFINITY;
    double last_cost = INFINITY;
    double param[3] = {0., 0.};
    for (int k = 0; k < max_iter; ++k) {
        std::vector<double> sample_i(cnt);
        std::vector<double> sample_y(cnt);
        std::vector<double> sample_theta(cnt);
        for (int j = 0; j < cnt; ++j) {
            int ind = rand() % idx.size();
            sample_i[j] = idx[ind];
            sample_y[j] = delta_v[ind];
            sample_theta[j] = theta[ind];
        }
        double sub_param[2] = {0.0, 0.0};
        ceres::Problem sub_problem;
        for (int i = 0; i < cnt; ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<WithoutTransWithRotCostFunction, 1, 2>
                            (new WithoutTransWithRotCostFunction(sample_i[i], sample_y[i] - coeff * sample_theta[i], T,
                                                                 coeff));
            sub_problem.AddResidualBlock(cost_function, NULL, sub_param);
        }
        ceres::Solver::Summary sub_summary;
        ceres::Solve(options, &sub_problem, &sub_summary);

        //calculate distance
        double cost = 0.;
        for (int i = 0; i < cnt; ++i) {
            cost += fabs(delta_v[i] + coeff * theta[i] -
                         (sub_param[0] * std::sin((2 * M_PI / T) * idx[i] + sub_param[1]) + sub_param[2]));
        }

        if (cost < last_cost) {
            last_cost = cost;
            param[0] = sub_param[0];
            param[1] = sub_param[1];
            if (fabs(last_min_cost - last_cost) < 1e-5) {
                last_cost = cost;
                param[0] = sub_param[0];
                param[1] = sub_param[1];
                break;
            }
            last_min_cost = last_cost;
        }
    }

    std::vector<double> out_param(2);
    for (int i = 0; i < out_param.size(); ++i) {
        out_param[i] = param[i];
    }
    return out_param;
}

std::vector<double> CTracking::FittingWTransWoRot(std::vector<double> &idx, std::vector<double> &delta_v) {
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-8;

    double T = ccam_model_->GetWidth();

    int max_iter = 50;
    int cnt = 10;
    double last_min_cost = INFINITY;
    double last_cost = INFINITY;
    double param[3] = {0., 0., 0.};
    for (int k = 0; k < max_iter; ++k) {
        std::vector<double> sample_i(cnt);
        std::vector<double> sample_y(cnt);
        for (int j = 0; j < cnt; ++j) {
            int ind = rand() % idx.size();
            sample_i[j] = idx[ind];
            sample_y[j] = delta_v[ind];
        }
        double sub_param[3] = {0.0, 0.0, 0.0};
        ceres::Problem sub_problem;
        for (int i = 0; i < cnt; ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<TranslationCostFunction, 1, 3>
                            (new TranslationCostFunction(sample_i[i], sample_y[i], T));
            sub_problem.AddResidualBlock(cost_function, NULL, sub_param);
        }
        ceres::Solver::Summary sub_summary;
        ceres::Solve(options, &sub_problem, &sub_summary);

        //calculate distance
        double cost = 0.;
        for (int i = 0; i < idx.size(); ++i) {
            cost += fabs(delta_v[i] - (sub_param[0] * std::sin((2 * M_PI / T) * idx[i] + sub_param[1]) + sub_param[2]));
        }

        if (cost < last_cost) {
            last_cost = cost;
            param[0] = sub_param[0];
            param[1] = sub_param[1];
            param[2] = sub_param[2];
            if (fabs(last_min_cost - last_cost) < 1e-5) {
                last_cost = cost;
                param[0] = sub_param[0];
                param[1] = sub_param[1];
                param[2] = sub_param[2];
                break;
            }
            last_min_cost = last_cost;
        }
    }

    std::vector<double> out_param(3);
    for (int i = 0; i < out_param.size(); ++i) {
        out_param[i] = param[i];
    }
    return out_param;
}

std::vector<double> CTracking::FittingWoTransWoRot(std::vector<double> &idx, std::vector<double> &delta_v) {
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-8;

    double T = ccam_model_->GetWidth();

    int max_iter = 50;
    int cnt = 10;
    double last_min_cost = INFINITY;
    double last_cost = INFINITY;
    double param[2] = {0., 0.};
    for (int k = 0; k < max_iter; ++k) {
        std::vector<double> sample_i(cnt);
        std::vector<double> sample_y(cnt);
        for (int j = 0; j < cnt; ++j) {
            int ind = rand() % idx.size();
            sample_i[j] = idx[ind];
            sample_y[j] = delta_v[ind];
            std::cout << sample_i[j] << ", " << sample_y[j] << std::endl;
        }
        double sub_param[2] = {0.0, 0.0};
        ceres::Problem sub_problem;
        for (int i = 0; i < cnt; ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<WithoutTransCostFunction, 1, 3>
                            (new WithoutTransCostFunction(sample_i[i], sample_y[i], T));
            sub_problem.AddResidualBlock(cost_function, NULL, sub_param);
        }
        ceres::Solver::Summary sub_summary;
        ceres::Solve(options, &sub_problem, &sub_summary);

        std::cout << "sub_param = " << sub_param[0] << ", " << sub_param[1] << std::endl;

        //calculate distance
        double cost = 0.;
        for (int i = 0; i < idx.size(); ++i) {
            cost += fabs(delta_v[i] -
                         (T / (2 * M_PI) * sub_param[0] * std::sin((2 * M_PI / T) * idx[i] + sub_param[1])));
        }

        if (cost < last_cost) {
            last_cost = cost;
            param[0] = sub_param[0];
            param[1] = sub_param[1];
            if (fabs(last_min_cost - last_cost) < 1e-5) {
                last_cost = cost;
                param[0] = sub_param[0];
                param[1] = sub_param[1];
                break;
            }
            last_min_cost = last_cost;
        }
    }

    std::vector<double> out_param(2);
    for (int i = 0; i < out_param.size(); ++i) {
        out_param[i] = param[i];
    }
    return out_param;
}

void CTracking::ImgRegisterRANSAC() {
    std::cout << "num of correspondences: " << idxs_.size() << std::endl;
    // Solver
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-8;

    if (method_ == WITH_TRANS_WITH_ROT) {
        double T = ccam_model_->GetWidth();
        // fitting delta_v and thetas to get roll and pitch
        std::vector<double> param_v = FittingWTransWRot(idxs_, delta_vs_, thetas_);
        // fitting delta_u to get yaw
        std::vector<double> param_u = FittingWTransWoRot(idxs_, delta_us_);

        peak_u_ = fabs(param_u[0] * T / (2 * M_PI));
        peak_v_ = fabs(param_v[0] * T / (2 * M_PI));
        // convert to transformation
        double angle_Rxy = param_v[0];
        // TODO: double check the sign of param_v[1]
        double axis_Rxy = -param_v[1];
        double gamma = T / (2 * M_PI);
        double yaw = param_u[2] / gamma;
        double roll = angle_Rxy * cos(axis_Rxy);
        double pitch = angle_Rxy * sin(axis_Rxy);
//        double roll = angle_Rxy / (sqrt(1 + pow(tan(axis_Rxy), 2)));
//        double pitch = roll * tan(axis_Rxy);
        std::cout << "YPR = " << (yaw * 180.0 / M_PI) << ", "
                  << (pitch * 180.0 / M_PI) << ", "
                  << (roll * 180.0 / M_PI) << std::endl;

        Eigen::Matrix3d R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
        cv::eigen2cv(R, R_);
        if (DEBUG) {
            // display
            cv::Mat show_img = display_wtrans_wrot(100, T, idxs_, delta_us_, thetas_, param_u, delta_vs_, param_v);
            cv::namedWindow("debug", cv::WINDOW_NORMAL);
            cv::imshow("debug", show_img);
            cv::imwrite("wtrans_wrot.jpg", show_img);
            cv::waitKey(100);
        }

    } else if (method_ == WITH_TRANS_WITHOUT_ROT) {
        double T = ccam_model_->GetWidth();
        // fitting delta_v
        std::vector<double> param_v = FittingWTransWoRot(idxs_, delta_vs_);

        // fitting delta_u
        std::vector<double> param_u = FittingWTransWoRot(idxs_, delta_us_);

        peak_u_ = fabs(param_u[0] * T / (2 * M_PI));
        peak_v_ = fabs(param_v[0] * T / (2 * M_PI));
        // convert to transformation
        double angle_Rxy = param_v[0];
//        // TODO: double check the sign of param_v[1]
        double axis_Rxy = -param_v[1];
        double gamma = T / (2 * M_PI);
        double yaw = param_u[2] / gamma;

        double roll = angle_Rxy * cos(axis_Rxy);
        double pitch = angle_Rxy * sin(axis_Rxy);
//        double roll = angle_Rxy / (sqrt(1 + pow(tan(axis_Rxy), 2)));
//        double pitch = roll * tan(axis_Rxy);
        std::cout << "YPR = " << (yaw * 180.0 / M_PI) << ", "
                  << (pitch * 180.0 / M_PI) << ", "
                  << (roll * 180.0 / M_PI) << std::endl;

        Eigen::Matrix3d R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();

//        Eigen::Vector3d euler_angles = R.eulerAngles(2, 1, 0); // yaw pitch roll
//        std::cout << "yaw, pitch, roll = " << euler_angles.transpose() << std::endl;
        cv::eigen2cv(R, R_);

        if (DEBUG) {
            // display
            cv::Mat show_img = display_wtrans_worot(100, T, idxs_, delta_us_, param_u, delta_vs_, param_v);
            cv::namedWindow("debug", cv::WINDOW_NORMAL);
            cv::imshow("debug", show_img);
            cv::imwrite("wtrans_worot.jpg", show_img);
            cv::waitKey(100);
        }

    } else if (method_ == WITHOUT_TRANS_WITH_ROT) {
        double T = ccam_model_->GetWidth();
        // fitting delta_v
        std::vector<double> param_v = FittingWoTransWRot(idxs_, delta_vs_, thetas_);

        // fitting delta_u
        double param_u[1] = {0.0};
        ceres::Problem problem_u;
        for (int i = 0; i < idxs_.size(); ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<OnlyYawCostFunction, 1, 1>
                            (new OnlyYawCostFunction(delta_us_[i]));
            problem_u.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5), param_u);
        }

        ceres::Solver::Summary summary_u;
        ceres::Solve(options, &problem_u, &summary_u);

        std::cout << "delta_us estimated Amplitude, phase, offset = ";
        for (auto a:param_u) {
            std::cout << a << " ";
        }
        std::cout << std::endl;

        peak_u_ = fabs(param_u[0] * T / (2 * M_PI));
        peak_v_ = fabs(param_v[0] * T / (2 * M_PI));
        // convert to transformation
        double angle_Rxy = param_v[0];
        // TODO: double check the sign of param_v[1]
        double axis_Rxy = -param_v[1];
        double gamma = T / (2 * M_PI);
        double yaw = param_u[0] / gamma;

        double roll = angle_Rxy * cos(axis_Rxy);
        double pitch = angle_Rxy * sin(axis_Rxy);
//        double roll = angle_Rxy / (sqrt(1 + pow(tan(axis_Rxy), 2)));
//        double pitch = roll * tan(axis_Rxy);
        std::cout << "YPR = " << (yaw * 180.0 / M_PI) << ", "
                  << (pitch * 180.0 / M_PI) << ", "
                  << (roll * 180.0 / M_PI) << std::endl;

        Eigen::Matrix3d R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
        cv::eigen2cv(R, R_);

        if (DEBUG) {
            // display
            cv::Mat show_img = display_wotrans_wrot(100, T, idxs_, delta_us_, thetas_, param_u, delta_vs_, param_v);
            cv::namedWindow("debug", cv::WINDOW_NORMAL);
            cv::imshow("debug", show_img);
            cv::imwrite("wotrans_wrot.jpg", show_img);
            cv::waitKey(100);
        }
    } else if (method_ == WITHOUT_TRANS_WITHOUT_ROT) {
        double T = ccam_model_->GetWidth();
        // fitting delta_v
        std::vector<double> param_v = FittingWoTransWoRot(idxs_, delta_vs_);

        // fitting delta_u
        double param_u[1] = {0.0};
        ceres::Problem problem_u;
        for (int i = 0; i < idxs_.size(); ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<OnlyYawCostFunction, 1, 1>
                            (new OnlyYawCostFunction(delta_us_[i]));
            problem_u.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5), param_u);
        }

        ceres::Solver::Summary summary_u;
        ceres::Solve(options, &problem_u, &summary_u);

        std::cout << "delta_us estimated Amplitude, phase, offset = ";
        for (auto a:param_u) {
            std::cout << a << " ";
        }
        std::cout << std::endl;

        peak_u_ = fabs(param_u[0] * T / (2 * M_PI));
        peak_v_ = fabs(param_v[0] * T / (2 * M_PI));
        // convert to transformation
        double angle_Rxy = param_v[0];
        // TODO: double check the sign of param_v[1]
        double axis_Rxy = -param_v[1];
        double gamma = T / (2 * M_PI);
        double yaw = param_u[0] / gamma;

        double roll = angle_Rxy * cos(axis_Rxy);
        double pitch = angle_Rxy * sin(axis_Rxy);
//        double roll = angle_Rxy / (sqrt(1 + pow(tan(axis_Rxy), 2)));
//        double pitch = roll * tan(axis_Rxy);
        std::cout << "YPR = " << (yaw * 180.0 / M_PI) << ", "
                  << (pitch * 180.0 / M_PI) << ", "
                  << (roll * 180.0 / M_PI) << std::endl;

        Eigen::Matrix3d R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
        cv::eigen2cv(R, R_);
        if (DEBUG) {
            // display
            cv::Mat show_img = display_wotrans_worot(100, T, idxs_, delta_us_, param_u, delta_vs_, param_v);
            cv::namedWindow("debug", cv::WINDOW_NORMAL);
            cv::imshow("debug", show_img);
            cv::imwrite("wotrans_worot.jpg", show_img);
            cv::waitKey(100);
        }
    } else {
        R_ = cv::Mat::eye(3, 3, CV_64F);
    }
}

cv::Mat CTracking::ImgRegisterVideo() {
    std::cout << "num of correspondences: " << idxs_.size() << std::endl;
    // Solver
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-8;

    if (method_ == WITH_TRANS_WITH_ROT) {
        // fitting delta_v and thetas to get roll and pitch
        double param_v[3] = {0.0, 0.0, 0.0};
        double T = ccam_model_->GetWidth();
        double coeff = T / (2 * M_PI) * 0.1;
        ceres::Problem problem_v;
        for (int i = 0; i < idxs_.size(); ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<WithRotationCostFunction, 1, 3>
                            (new WithRotationCostFunction(idxs_[i], delta_vs_[i] - coeff * thetas_[i], T, coeff));
            problem_v.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5), param_v);
        }
        ceres::Solver::Summary summary_v;
        ceres::Solve(options, &problem_v, &summary_v);

        std::cout << "delta_vs estimated Amplitude, phase, offset = ";
        for (auto a:param_v) {
            std::cout << a << " ";
        }
        std::cout << std::endl;

        // fitting delta_u to get yaw
        double param_u[3] = {0.0, 0.0, 0.0};
        ceres::Problem problem_u;
        for (int i = 0; i < idxs_.size(); ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<TranslationCostFunction, 1, 3>
                            (new TranslationCostFunction(idxs_[i], delta_us_[i], T));
            problem_u.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5), param_u);
        }

        ceres::Solver::Summary summary_u;
        ceres::Solve(options, &problem_u, &summary_u);

        std::cout << "delta_us estimated Amplitude, phase, offset = ";
        for (auto a:param_u) {
            std::cout << a << " ";
        }
        std::cout << std::endl;

        peak_u_ = fabs(param_u[0] * T / (2 * M_PI));
        peak_v_ = fabs(param_v[0] * T / (2 * M_PI));
        // convert to transformation
        double angle_Rxy = param_v[0];
        // TODO: double check the sign of param_v[1]
        double axis_Rxy = -param_v[1];
        double gamma = T / (2 * M_PI);
        double yaw = param_u[2] / gamma;
        double roll = angle_Rxy * cos(axis_Rxy);
        double pitch = angle_Rxy * sin(axis_Rxy);
//        double roll = angle_Rxy / (sqrt(1 + pow(tan(axis_Rxy), 2)));
//        double pitch = roll * tan(axis_Rxy);
        std::cout << "YPR = " << (yaw * 180.0 / M_PI) << ", "
                  << (pitch * 180.0 / M_PI) << ", "
                  << (roll * 180.0 / M_PI) << std::endl;

        Eigen::Matrix3d R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
        cv::eigen2cv(R, R_);
        if (DEBUG) {
            // display
            cv::Mat show_img = display_wtrans_wrot(100, T, idxs_, delta_us_, thetas_, param_u, delta_vs_, param_v);
            return show_img;
        }

    } else if (method_ == WITH_TRANS_WITHOUT_ROT) {
        // fitting delta_v
        double param_v[3] = {0.0, 0.0, 0.0};
        double T = ccam_model_->GetWidth();
        ceres::Problem problem_v;
        for (int i = 0; i < idxs_.size(); ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<TranslationCostFunction, 1, 3>
                            (new TranslationCostFunction(idxs_[i], delta_vs_[i], T));
            problem_v.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5), param_v);
        }

        ceres::Solver::Summary summary_v;
        ceres::Solve(options, &problem_v, &summary_v);

        // display results
//        std::cout << summary_v.FullReport() << std::endl;
        std::cout << "delta_vs estimated Amplitude, phase, offset = ";
        for (auto a:param_v) {
            std::cout << a << " ";
        }
        std::cout << std::endl;

        // fitting delta_u
        double param_u[3] = {0.0, 0.0, 0.0};
        ceres::Problem problem_u;
        for (int i = 0; i < idxs_.size(); ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<TranslationCostFunction, 1, 3>
                            (new TranslationCostFunction(idxs_[i], delta_us_[i], T));
            problem_u.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5), param_u);
        }

        ceres::Solver::Summary summary_u;
        ceres::Solve(options, &problem_u, &summary_u);

        std::cout << "delta_us estimated Amplitude, phase, offset = ";
        for (auto a:param_u) {
            std::cout << a << " ";
        }
        std::cout << std::endl;

        peak_u_ = fabs(param_u[0] * T / (2 * M_PI));
        peak_v_ = fabs(param_v[0] * T / (2 * M_PI));
        // convert to transformation
        double angle_Rxy = param_v[0];
//        // TODO: double check the sign of param_v[1]
        double axis_Rxy = -param_v[1];
        double gamma = T / (2 * M_PI);
        double yaw = param_u[2] / gamma;

        double roll = angle_Rxy * cos(axis_Rxy);
        double pitch = angle_Rxy * sin(axis_Rxy);
//        double roll = angle_Rxy / (sqrt(1 + pow(tan(axis_Rxy), 2)));
//        double pitch = roll * tan(axis_Rxy);
        std::cout << "YPR = " << (yaw * 180.0 / M_PI) << ", "
                  << (pitch * 180.0 / M_PI) << ", "
                  << (roll * 180.0 / M_PI) << std::endl;

        Eigen::Matrix3d R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();

//        Eigen::Vector3d euler_angles = R.eulerAngles(2, 1, 0); // yaw pitch roll
//        std::cout << "yaw, pitch, roll = " << euler_angles.transpose() << std::endl;
        cv::eigen2cv(R, R_);

        if (DEBUG) {
            // display
            cv::Mat show_img = display_wtrans_worot(100, T, idxs_, delta_us_, param_u, delta_vs_, param_v);
            return show_img;
        }

    } else if (method_ == WITHOUT_TRANS_WITH_ROT) {
        // fitting delta_v
        double param_v[2] = {0.0, 0.0};
        double T = ccam_model_->GetWidth();
        double coeff = T / (2 * M_PI) * 0.1;
        ceres::Problem problem_v;
        for (int i = 0; i < idxs_.size(); ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<WithoutTransWithRotCostFunction, 1, 2>
                            (new WithoutTransWithRotCostFunction(idxs_[i], delta_vs_[i] - coeff * thetas_[i], T,
                                                                 coeff));
            problem_v.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5), param_v);
        }

        ceres::Solver::Summary summary_v;
        ceres::Solve(options, &problem_v, &summary_v);

        // display results
//        std::cout << summary_v.FullReport() << std::endl;
        std::cout << "delta_vs estimated Amplitude, phase, offset = ";
        for (auto a:param_v) {
            std::cout << a << " ";
        }
        std::cout << std::endl;

        // fitting delta_u
        double param_u[1] = {0.0};
        ceres::Problem problem_u;
        for (int i = 0; i < idxs_.size(); ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<OnlyYawCostFunction, 1, 1>
                            (new OnlyYawCostFunction(delta_us_[i]));
//            problem_u.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5), param_u);
            problem_u.AddResidualBlock(cost_function, NULL, param_u);
        }

        ceres::Solver::Summary summary_u;
        ceres::Solve(options, &problem_u, &summary_u);

        std::cout << "delta_us estimated Amplitude, phase, offset = ";
        for (auto a:param_u) {
            std::cout << a << " ";
        }
        std::cout << std::endl;

        peak_u_ = fabs(param_u[0] * T / (2 * M_PI));
        peak_v_ = fabs(param_v[0] * T / (2 * M_PI));
        // convert to transformation
        double angle_Rxy = param_v[0];
        // TODO: double check the sign of param_v[1]
        double axis_Rxy = -param_v[1];
        double gamma = T / (2 * M_PI);
        double yaw = param_u[0] / gamma;

        double roll = angle_Rxy * cos(axis_Rxy);
        double pitch = angle_Rxy * sin(axis_Rxy);
//        double roll = angle_Rxy / (sqrt(1 + pow(tan(axis_Rxy), 2)));
//        double pitch = roll * tan(axis_Rxy);
        std::cout << "YPR = " << (yaw * 180.0 / M_PI) << ", "
                  << (pitch * 180.0 / M_PI) << ", "
                  << (roll * 180.0 / M_PI) << std::endl;

        Eigen::Matrix3d R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
        cv::eigen2cv(R, R_);

        if (DEBUG) {
            // display
            cv::Mat show_img = display_wotrans_wrot(100, T, idxs_, delta_us_, thetas_, param_u, delta_vs_, param_v);
            return show_img;
        }
    } else if (method_ == WITHOUT_TRANS_WITHOUT_ROT) {
        // fitting delta_v
        double param_v[2] = {0.0, 0.0};
        double T = ccam_model_->GetWidth();
        ceres::Problem problem_v;
        for (int i = 0; i < idxs_.size(); ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<WithoutTransCostFunction, 1, 2>
                            (new WithoutTransCostFunction(idxs_[i], delta_vs_[i], T));
            problem_v.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5), param_v);
        }

        ceres::Solver::Summary summary_v;
        ceres::Solve(options, &problem_v, &summary_v);

        // display results
//        std::cout << summary_v.FullReport() << std::endl;
        std::cout << "delta_vs estimated Amplitude, phase, offset = ";
        for (auto a:param_v) {
            std::cout << a << " ";
        }
        std::cout << std::endl;

        // fitting delta_u
        double param_u[1] = {0.0};
        ceres::Problem problem_u;
        for (int i = 0; i < idxs_.size(); ++i) {
            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<OnlyYawCostFunction, 1, 1>
                            (new OnlyYawCostFunction(delta_us_[i]));
//            problem_u.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5), param_u);
            problem_u.AddResidualBlock(cost_function, NULL, param_u);
        }

        ceres::Solver::Summary summary_u;
        ceres::Solve(options, &problem_u, &summary_u);

        std::cout << "delta_us estimated Amplitude, phase, offset = ";
        for (auto a:param_u) {
            std::cout << a << " ";
        }
        std::cout << std::endl;

        peak_u_ = fabs(param_u[0] * T / (2 * M_PI));
        peak_v_ = fabs(param_v[0] * T / (2 * M_PI));
        // convert to transformation
        double angle_Rxy = param_v[0];
        // TODO: double check the sign of param_v[1]
        double axis_Rxy = -param_v[1];
        double gamma = T / (2 * M_PI);
        double yaw = param_u[0] / gamma;

        double roll = angle_Rxy * cos(axis_Rxy);
        double pitch = angle_Rxy * sin(axis_Rxy);
//        double roll = angle_Rxy / (sqrt(1 + pow(tan(axis_Rxy), 2)));
//        double pitch = roll * tan(axis_Rxy);
        std::cout << "YPR = " << (yaw * 180.0 / M_PI) << ", "
                  << (pitch * 180.0 / M_PI) << ", "
                  << (roll * 180.0 / M_PI) << std::endl;

        Eigen::Matrix3d R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
        cv::eigen2cv(R, R_);
        if (DEBUG) {
            // display
            cv::Mat show_img = display_wotrans_worot(100, T, idxs_, delta_us_, param_u, delta_vs_, param_v);
            return show_img;
        }
    } else {
        R_ = cv::Mat::eye(3, 3, CV_64F);
        return cv::Mat::eye(100,100, CV_8UC3);
    }
}

