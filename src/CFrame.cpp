//
// Created by xuqw on 12/5/19.
//

#include <opencv2/opencv.hpp>
#include "CFrame.h"

CFrame::CFrame() {
    input_type_ = OMNI;
}

CFrame::CFrame(const CFrame &frame) :
        edge_(frame.edge_),
        step_(frame.step_),
        sub_images_(frame.sub_images_),
        corners_(frame.corners_),
        input_type_(frame.input_type_),
        idx_(frame.idx_){
    frame.pano_img_.copyTo(pano_img_);
    frame.img_.copyTo(img_);
}

CFrame::CFrame(int edge,
               int step,
               CCamModel &ccam_model) :
        edge_(edge),
        step_(step){
    sub_images_.clear();
    corners_.clear();
    ccam_model_ = &ccam_model;
    input_type_ = OMNI;
}

CFrame::CFrame(int edge,
               int step,
               CCamModel &ccam_model,
               InputType input_type) :
        edge_(edge),
        step_(step) {
    sub_images_.clear();
    corners_.clear();
    ccam_model_ = &ccam_model;
    input_type_ = input_type;
}

CFrame::CFrame(const cv::Mat &img,
               int edge,
               int step,
               CCamModel &ccam_model) :
        edge_(edge),
        step_(step){
    img.copyTo(img_);
    sub_images_.clear();
    corners_.clear();
    ccam_model_ = &ccam_model;
    input_type_ = OMNI;
    unwrapImage();
    divideByEdge(Padding, 0);
}

void CFrame::SetImg(cv::Mat & img){
    sub_images_.clear();
    corners_.clear();
    if (input_type_ == OMNI) {
        img.copyTo(img_);
        unwrapImage();
    }else{
        img.copyTo(pano_img_);
    }
    divideByEdge(Padding, 0);
}

void CFrame::SetImg(int idx, cv::Mat & img){
    idx_ = idx;
    sub_images_.clear();
    corners_.clear();
    if (input_type_ == OMNI) {
        img.copyTo(img_);
        unwrapImage();
    }else{
        img.copyTo(pano_img_);
    }
    divideByEdge(Padding, 0);
}

std::vector<cv::Mat> CFrame::GetSubframes() {
    return sub_images_;
}

std::vector<cv::Point2d> CFrame::GetCorners() {
    return corners_;
}

cv::Mat CFrame::GetPanoImg() {
    return pano_img_;
}

int CFrame::GetStep() {
    return step_;
}

int CFrame::GetEdge() {
    return edge_;
}

void CFrame::unwrapImage() {
    double Hd = ccam_model_->GetHeight();
    double Wd = ccam_model_->GetWidth();
    cv::Mat dst,map_x,map_y;
    dst.create(static_cast<int>(Hd), static_cast<int>(Wd), img_.type());
    map_x.create(dst.size(), CV_32FC1);
    map_y.create(dst.size(), CV_32FC1);
    for( int j=0;j<dst.rows;j++)
    {
        for(int i=0;i<dst.cols;i++)
        {
            cv::Point2d ppixel = cv::Point2d(i, j);
            cv::Point2d opixel = ccam_model_->PanoToOmni(ppixel);

            map_x.at<float>(j,i) = static_cast<float>(opixel.x);
            map_y.at<float>(j,i) = static_cast<float>(opixel.y);
        }
    }
    cv::remap(img_, dst, map_x, map_y, cv::INTER_LINEAR);
    pano_img_ = dst.clone();
//    cv::Mat reversed_pano = dst.clone();
//    cv::flip(reversed_pano, pano_img_, 1);
//    cv::imwrite("pano_img.jpg", pano_img_);

//    cv::namedWindow("pano image", cv::WINDOW_NORMAL);
//    cv::imshow("pano image", pano_img_);
//    cv::waitKey(0);

//    std::cout << "size: " << Hd << ", " << Wd << std::endl;
}

void CFrame::divideByEdge(int flag, int shift) {
    cv::Mat Img;
    int padding_shift= 0;
    if(flag == Padding){
        padding_shift -= edge_/2;
        // Padding in the left and right
        Img.create(pano_img_.rows, pano_img_.cols + edge_, pano_img_.type());
        pano_img_.copyTo(Img.colRange(edge_ / 2, pano_img_.cols + edge_ / 2));
//        cv::Mat reverse_front, reverse_back;
//        flip(pano_img_.colRange(0,edge_ / 2), reverse_front, 1);
//        reverse_front.copyTo(Img.colRange(pano_img_.cols, pano_img_.cols + edge_ / 2));
//        flip(pano_img_.colRange(pano_img_.cols-edge_ / 2, pano_img_.cols), reverse_back, 1);
//        reverse_back.copyTo(Img.colRange(0, edge_ / 2));
        pano_img_.colRange(0,edge_ / 2).copyTo(Img.colRange(pano_img_.cols + edge_ / 2, pano_img_.cols + edge_));
        pano_img_.colRange(pano_img_.cols-edge_ / 2, pano_img_.cols).copyTo(Img.colRange(0, edge_ / 2));

    }else {
        pano_img_.copyTo(Img);
    }
    int width = Img.cols;
    int height = Img.rows;
    int curr_w = 0;
    int curr_h = 0;
    while (curr_h + edge_ <= height) {
        while (curr_w + edge_ < width) {
            cv::Mat subImg;
            subImg.create(edge_, edge_, Img.type());
            cv::Point2d corner;
            if ((curr_h + edge_ < height) && (curr_w + edge_ < width)) {
                Img.rowRange(curr_h, curr_h + edge_).colRange(curr_w, curr_w + edge_).copyTo(subImg);
                corner = cv::Point2d(curr_w + edge_ / 2. + shift + padding_shift, curr_h + edge_ / 2. + shift);
            } else if ((curr_h + edge_ < height) && (curr_w + edge_ >= width)) {
                Img.rowRange(curr_h, curr_h + edge_).colRange(width - edge_, width).copyTo(subImg);
                corner = cv::Point2d(width - edge_ / 2. + shift + padding_shift, curr_h + edge_ / 2. + shift);
            } else if ((curr_h + edge_ >= height) && (curr_w + edge_ < width)) {
                Img.rowRange(height - edge_, height).colRange(curr_w, curr_w + edge_).copyTo(subImg);
                corner = cv::Point2d(curr_w + edge_ / 2. + shift+ padding_shift, height - edge_ / 2. + shift);
            } else {
                Img.rowRange(height - edge_, height).colRange(width - edge_, width).copyTo(subImg);
                corner = cv::Point2d(width - edge_ / 2. + shift+ padding_shift, height - edge_ / 2. + shift);
            }
            sub_images_.push_back(subImg);
            corners_.push_back(corner);
            if (curr_w + edge_ >= width) {
                break;
            }
            curr_w += step_;
        }
        curr_w = 0;
        if (curr_h + edge_ >= height) {
            break;
        }
        curr_h += step_;
    }
}
