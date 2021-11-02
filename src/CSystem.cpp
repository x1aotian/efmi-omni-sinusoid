//
// Created by xuqw on 12/25/19.
//


#include "misc_1.h"
#include "CSystem.h"

CSystem::CSystem() {
    ccam_model_ = NULL;
    ctracker_ = NULL;
    first_flag_ = true;
    T_ = cv::Mat::eye(3, 3, CV_64F);
    kf_flag_ = false;
    input_type_ = OMNI;
    match_method_ = FMI;
}

CSystem::CSystem(const CCamModel &ccam_model, const CTracking &ctracker) {
    ccam_model_ = new CCamModel(ccam_model);
    ctracker_ = new CTracking(ctracker);
    first_flag_ = true;
    T_ = cv::Mat::eye(3, 3, CV_64F);
    kf_flag_ = false;
    input_type_ = OMNI;
    match_method_ = FMI;
}

CSystem::CSystem(const CCamModel &ccam_model,
                 const CTracking &ctracker,
                 Match_Method match_method) {
    ccam_model_ = new CCamModel(ccam_model);
    ctracker_ = new CTracking(ctracker);
    first_flag_ = true;
    T_ = cv::Mat::eye(3, 3, CV_64F);
    kf_flag_ = false;
    input_type_ = OMNI;
    match_method_ = match_method;
}

CSystem::CSystem(const CCamModel &ccam_model,
                 const CTracking &ctracker,
                 InputType input_type) {
    ccam_model_ = new CCamModel(ccam_model);
    ctracker_ = new CTracking(ctracker);
    first_flag_ = true;
    T_ = cv::Mat::eye(3, 3, CV_64F);
    kf_flag_ = false;
    input_type_ = input_type;
    match_method_ = FMI;
}

CSystem::CSystem(cv::Mat oimg1,
                 cv::Mat oimg2,
                 double cx,
                 double cy,
                 double lr,
                 double sr,
                 int edge,
                 int step,
                 Reg_Method method) {
    ccam_model_ = new CCamModel(cx, cy, lr, sr);
    ctracker_ = new CTracking(*ccam_model_, oimg1, oimg2, edge, step, method);
    T_ = cv::Mat::eye(3, 3, CV_64F);
    kf_flag_ = false;
    input_type_ = OMNI;
    match_method_ = FMI;
}

void CSystem::InitializeCamModel(double cx, double cy, double lr, double sr) {
    ccam_model_ = new CCamModel(cx, cy, lr, sr);
}

void CSystem::InitializeTracker(int edge, int step, Reg_Method method) {
    ctracker_ = new CTracking(*ccam_model_, edge, step, method);
}

void CSystem::Tracking(int id, cv::Mat img) {
    if (first_flag_) {
        ctracker_->SetFirstImg(img);
        first_flag_ = false;
        kf_flag_ = true;
    } else {
        ctracker_->SetSecondImg(img);

        // Motion Vector from Optical flow or iFMI
        if (match_method_ == OpticalFlow) {
            ctracker_->GetMotionVecOptical();
        } else {
            ctracker_->GetMotionVec();
        }
        ctracker_->ImgRegisterRANSAC();
        std::cout << "average distance: " << AverDistance(ctracker_->delta_us_, ctracker_->delta_vs_) << std::endl;

        if (AverDistance(ctracker_->delta_us_, ctracker_->delta_vs_) > 0.01) { // disparity > threshold  -> register
            T_ = T_ * ctracker_->R_;
            kf_flag_ = true;
            ctracker_->RenewLastFrame();
        } else {
            kf_flag_ = false;
        }
    }
}

cv::Mat CSystem::TrackingVideo(int id, cv::Mat img, cv::Mat &pano_1, cv::Mat &pano_2) {
    if (first_flag_) {
        ctracker_->SetFirstImg(id, img);
        first_flag_ = false;
        kf_flag_ = true;
        return cv::Mat::eye(3, 3, CV_8UC3);
    } else {
        ctracker_->SetSecondImg(id, img);

        // Motion Vector from Optical flow or iFMI
        if (match_method_ == OpticalFlow) {
            ctracker_->GetMotionVecOptical();
        } else {
            ctracker_->GetMotionVec();
        }
        cv::Mat show_img = ctracker_->ImgRegisterVideo();

        ctracker_->cf1_->pano_img_.copyTo(pano_1);
        ctracker_->cf2_->pano_img_.copyTo(pano_2);

        if (AverDistance(ctracker_->delta_us_, ctracker_->delta_vs_) > 0.01) { // disparity > threshold  -> register
//            T_ = T_ * ctracker_->R_;
            T_ = ctracker_->R_;
            kf_flag_ = true;
//            ctracker_->RenewLastFrame();
        } else {
            kf_flag_ = false;
        }

        return show_img;
    }
}

void CSystem::Register(int id, cv::Mat img) {
    if (first_flag_) {
        ctracker_->SetFirstImg(img);
        first_flag_ = false;
        kf_flag_ = false;
    } else {
        ctracker_->SetSecondImg(img);

        // Motion Vector from Optical flow or iFMI
        if (match_method_ == OpticalFlow) {
            ctracker_->GetMotionVecOptical();
        } else {
            ctracker_->GetMotionVec();
        }
        ctracker_->ImgRegister();
        T_ = ctracker_->R_;
        kf_flag_ = true;
    }
}

cv::Mat CSystem::RegisterVideo(int id, cv::Mat img) {
    if (first_flag_) {
        ctracker_->SetFirstImg(id, img);
        first_flag_ = false;
        kf_flag_ = false;
        return cv::Mat::eye(3, 3, CV_8UC3);
    } else {
        ctracker_->SetSecondImg(id, img);

        cv::imshow("pano 1", ctracker_->cf2_->pano_img_);

        // Motion Vector from Optical flow or iFMI
        if (match_method_ == OpticalFlow) {
            ctracker_->GetMotionVecOptical();
        } else {
            ctracker_->GetMotionVec();
        }
        cv::Mat show_img = ctracker_->ImgRegisterVideo();
        T_ = ctracker_->R_;
        kf_flag_ = true;

        return show_img;
    }
}