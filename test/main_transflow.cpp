// Created by xiaotian on 09/05/20.

#include "CTracking.h"
#include <ctime>
#include <opencv2/imgproc.hpp>
#include "misc_1.h"

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include "string"

int main(int argc, char ** argv){
    // read two images
    cv::Mat oimg1 = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat oimg2 = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);

    // camera model parameters
    /* // original
    double cx = 675.0;
    double cy = 350.0;
    double lr = 230.0;
    double sr = 120.0;
    */

    // PanoraMIS camera model parameters
    double cx = 310;
    double cy = 310;
    double lr = 310;
    double sr = 60;
    CCamModel ccam_model = CCamModel(cx, cy, lr, sr); // Image size: 250 * 1125

    // create tracker
    int edge = 250; // original: 110; OVMIS: 250;
    int step = 10; // original: 20
    double square_coeff = 1; // original: 1.8
    CTracking ctracker = CTracking(ccam_model, oimg1, oimg2, edge, step, square_coeff, WITH_TRANS_WITHOUT_ROT);

    std::vector< std::vector<double>> set_duslines;
    std::vector< std::vector<double>> set_dvslines;


    for (int i = 0; i < 1; ++i) {
        clock_t start_t, end1_t, end2_t;
        start_t = clock();
        ctracker.GetMotionTransFlow(set_duslines,set_dvslines);
        //cv::imshow(std::to_string(i), ctracker.cf1_->GetPanoImg());
        //cv::imshow(std::to_string(i), ctracker.cf2_->GetPanoImg());
        //cv::waitKey(0);
    }

    std::ofstream outFile;
    outFile.open("./result/22_23/new_results/du_data.txt");

    for (int i = 0; i < set_duslines.size()-1 ; i++){
        for (int j = 0; j < set_duslines[i].size()-1 ; j++) {
            outFile << set_duslines[i][j] << " ";
        }
        outFile << std::endl;
    }
    outFile.close();

    outFile.open("./result/22_23/new_results/dv_data.txt");

    for (int i = 0; i < set_dvslines.size()-1 ; i++){
        for (int j = 0; j < set_dvslines[i].size()-1 ; j++) {
            outFile << set_dvslines[i][j] << " ";
        }
        outFile << std::endl;
    }
    outFile.close();


    return 0;
}
