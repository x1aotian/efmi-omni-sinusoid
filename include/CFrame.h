//
// Created by xuqw on 12/5/19.
//

#ifndef OMNI_SINUSOID_CFRAME_H
#define OMNI_SINUSOID_CFRAME_H

#include <vector>
#include "Frame.h"
#include "CCamModel.h"

enum divide_flag {NonPadding, Padding};
enum InputType{OMNI, PANO};

class CFrame : public Frame{
public:
    CFrame();
    CFrame(int edge, int step, CCamModel & ccam_model);
    CFrame(int edge, int step, CCamModel & ccam_model, InputType input_type);
    CFrame(const cv::Mat & img, int edge, int step, CCamModel & ccam_model);
    CFrame(const CFrame & frame);
    ~CFrame(){}

    void SetImg(cv::Mat & img);
    void SetImg(int idx, cv::Mat & img);

    std::vector<cv::Mat> GetSubframes();
    std::vector<cv::Point2d> GetCorners();
    cv::Mat GetPanoImg();
    int GetStep();
    int GetEdge();

    /*
     * convert omni image to panorama image
     */
    void unwrapImage();

    /* Divide image I into several subimages with size of edge*edge
     * if edge equals to step, non overlap; otherwise overlap
     * flag: padding or not
     * shift: shifted how may pixels from the center
     */
    void divideByEdge(int flag, int shift);

    cv::Mat pano_img_;
    int idx_;
protected:
    CCamModel * ccam_model_;
    std::vector<cv::Mat> sub_images_;
    std::vector<cv::Point2d> corners_;
    int edge_;
    int step_;
    InputType input_type_;
};


#endif //OMNI_SINUSOID_CFRAME_H
