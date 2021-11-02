//
// Created by xuqw on 12/25/19.
//

#ifndef OMNI_SINUSOID_CSYSTEM_H
#define OMNI_SINUSOID_CSYSTEM_H

#include "CCamModel.h"
#include "CTracking.h"

enum Match_Method{FMI, OpticalFlow};

class CSystem {
public:
    CSystem();
    CSystem(const CCamModel & ccam_model,
            const CTracking & ctracker);
    CSystem(const CCamModel & ccam_model,
            const CTracking & ctracker,
            Match_Method match_method);
    CSystem(const CCamModel & ccam_model,
            const CTracking & ctracker,
            InputType input_type);
    CSystem(cv::Mat oimg1,
            cv::Mat oimg2,
            double cx,
            double cy,
            double lr,
            double sr,
            int edge,
            int step,
            Reg_Method method);
    ~CSystem(){}

    void InitializeCamModel(double cx, double cy, double lr, double sr);
    void InitializeTracker(int edge, int step, Reg_Method method);

    void Tracking(int id, cv::Mat img);

    cv::Mat TrackingVideo(int id, cv::Mat img, cv::Mat & pano_1, cv::Mat & pano_2);

    void Register(int id, cv::Mat img);

    cv::Mat RegisterVideo(int id, cv::Mat img);

    cv::Mat T_;
    bool kf_flag_;

    CTracking * ctracker_;

private:
    CCamModel * ccam_model_;
    bool first_flag_;
    InputType input_type_;
    Match_Method match_method_;
};


#endif //OMNI_SINUSOID_CSYSTEM_H
