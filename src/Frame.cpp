//
// Created by xuqw on 12/4/19.
//

#include "Frame.h"

Frame::Frame(cv::Mat &img) {
    img.copyTo(img_);
}

Frame::Frame(const Frame &frame) {
    img_ = frame.img_;
}

