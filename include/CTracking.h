//
// Created by xuqw on 12/6/19.
//

#ifndef OMNI_SINUSOID_CTRACKING_H
#define OMNI_SINUSOID_CTRACKING_H

#include "CCamModel.h"
#include "CFrame.h"
#include "Tracking.h"
#include "fmiImage.hh"
#include "fmiRegistration.hh"
#include <ceres/ceres.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

#define DEBUG 1

enum Reg_Method {WITH_TRANS_WITH_ROT, WITH_TRANS_WITHOUT_ROT, WITHOUT_TRANS_WITH_ROT, WITHOUT_TRANS_WITHOUT_ROT};

class CTracking : public Tracking{
public:
    CTracking();
    CTracking(const CTracking & ctracker);
    CTracking(CCamModel & ccam_model, int edge, int step, Reg_Method method);
    CTracking(CCamModel & ccam_model, int edge, int step, double square_coeff, Reg_Method method);
    CTracking(CCamModel & ccam_model, int edge, int step, double square_coeff, Reg_Method method, InputType input_type);
    CTracking(CCamModel & ccam_model, cv::Mat & oimg1, cv::Mat & oimg2, int edge, int step, Reg_Method method);
    CTracking(CCamModel & ccam_model, cv::Mat & oimg1, cv::Mat & oimg2, int edge, int step, double square_coeff, Reg_Method method);
    ~CTracking(){}

    void SetImgs(cv::Mat & oimg1, cv::Mat & oimg2);
    void SetFirstImg(cv::Mat & oimg1);
    void SetFirstImg(int idx, cv::Mat & oimg1);
    void SetSecondImg(cv::Mat &oimg2);
    void SetSecondImg(int idx, cv::Mat &oimg2);
    void SetNewImg(cv::Mat & oimg2);
    void RenewLastFrame();
    void GetMotionVec();
    void GetMotionVecEfmi();
    void GetMotionTransFlow( std::vector< std::vector<double> > & set_dusline, std::vector< std::vector<double> > & set_dvsline);
    void GetMotionVecOptical(); // can only works WITHOUT_ROTATION_SCALING
    void ImgRegister();

    void ImgRegisterRANSAC();

    // to display image for paper
    cv::Mat ImgRegisterVideo();

    std::vector<double> FittingWTransWRot(std::vector<double> & idx,
                                          std::vector<double> & delta_v,
                                          std::vector<double> &theta);
    std::vector<double> FittingWoTransWRot(std::vector<double> & idx,
                                           std::vector<double> & delta_v,
                                           std::vector<double> &theta);
    std::vector<double> FittingWTransWoRot(std::vector<double> & idx, std::vector<double> & delta_v);
    std::vector<double> FittingWoTransWoRot(std::vector<double> & idx, std::vector<double> & delta_v);

    struct TranslationCostFunction {
        TranslationCostFunction(double t, double y, double W) : t_(t), y_(y), W_(W) {}

        // Fit Function : f(t) = A * sin(w * t + p) + c
        // Parameters
        // Parameters[0] : Amplitude (A)
        // Parameters[1] : phase (p)
        // Parameters[2] : offset (c)
        template <typename T>
        bool operator() (const T* const parameters, T* residual) const {
            residual[0] = T(y_) - (W_/(2 * M_PI) * parameters[0] *
                                           ceres::sin((2 * M_PI / W_) * t_ + parameters[1])
                                   + parameters[2]);
            return true;
        }

    // Need to debug... using roll, pitch directly
//        template <typename T>
//        bool operator() (const T* const parameters, T* residual) const {
//            residual[0] = T(y_) - (W_/(2 * M_PI) * sqrt(pow(parameters[0], 2)
//                                                        + pow(parameters[1], 2)) *
//                                           ceres::sin((2 * M_PI / W_) * t_ - atan2(parameters[1], parameters[0])) + parameters[2]);
//            return true;
//        }

    private:
        double t_, y_;
        double W_;
    };

    struct WithoutTransCostFunction {
        WithoutTransCostFunction(double t, double y, double W) : t_(t), y_(y), W_(W) {}

        // Fit Function : f(t) = A * sin(w * t + p) + c
        // Parameters
        // Parameters[0] : Amplitude (A)
        // Parameters[1] : phase (p)
        template <typename T>
        bool operator() (const T* const parameters, T* residual) const {
            residual[0] = T(y_) - (W_/(2 * M_PI) * parameters[0] *
                                   ceres::sin((2 * M_PI / W_) * t_ + parameters[1]));
            return true;
        }

    private:
        double t_, y_;
        double W_;
    };

    struct WithRotationCostFunction {
        // lamda is the weight coefficient
        WithRotationCostFunction(double t, double y, double W, double lamda) : t_(t), y_(y), W_(W), lamda_(lamda) {}

        // Fit Function : f(t) = W/2pi * A * sin(w * t + p) + c + lamda * A * cos(w * t + p)
        // Parameters
        // Parameters[0] : Amplitude (A)
        // Parameters[1] : phase (p)
        // Parameters[2] : offset (c)
        template <typename T>
        bool operator() (const T* const parameters, T* residual) const {
            residual[0] = T(y_) -
                    (W_/(2 * M_PI) *parameters[0] *
                             ceres::sin((2 * M_PI / W_) * t_ + parameters[1]) + parameters[2]
                             - lamda_ * parameters[0] *
                               ceres::cos((2 * M_PI / W_) * t_ + parameters[1]));
            return true;
        }

    private:
        double t_, y_;
        double W_;
        double lamda_;
    };

    struct WithoutTransWithRotCostFunction {
        // lamda is the weight coefficient
        WithoutTransWithRotCostFunction(double t, double y, double W, double lamda) : t_(t), y_(y), W_(W), lamda_(lamda) {}

        // Fit Function : f(t) = A * sin(w * t + p) + A * sin(w * t + p)
        // Parameters
        // Parameters[0] : Amplitude (A)
        // Parameters[1] : phase (p)
        template <typename T>
        bool operator() (const T* const parameters, T* residual) const {
            residual[0] = T(y_) -
                          (W_/(2 * M_PI) *parameters[0] *
                           ceres::sin((2 * M_PI / W_) * t_ + parameters[1])
                           - lamda_ * parameters[0] *
                           ceres::cos((2 * M_PI / W_) * t_ + parameters[1]));
            return true;
        }

    private:
        double t_, y_;
        double W_;
        double lamda_;
    };

    struct OnlyYawCostFunction {
        OnlyYawCostFunction(double y) : y_(y) {}

        // Fit Function : f(t) = c
        // Parameters
        // Parameters[0] : offset c
        template <typename T>
        bool operator() (const T* const parameters, T* residual) const {
            residual[0] = T(y_) - parameters[0];
            return true;
        }

    private:
        double y_;
    };


    cv::Mat R_;
    double peak_u_;
    double peak_v_;
    double peak_theta_;
    double peak_s_;
    std::vector<double> idxs_;
    std::vector<double> delta_us_;
    std::vector<double> delta_vs_;
    std::vector<double> thetas_;
    std::vector<double> scalings_;

    CFrame * cf1_;
    CFrame * cf2_;
    Reg_Method method_;
protected:
    CCamModel * ccam_model_;
    InputType input_type_;
    double square_coeff_;
};


#endif //OMNI_SINUSOID_CTRACKING_H
