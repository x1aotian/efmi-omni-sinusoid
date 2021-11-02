//
// Created by xuqw on 12/26/19.
//

#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#define Cx 670
//#define Cy 350
//#define R1 120
//#define R2 230

#define Cx 680
#define Cy 730
#define R1 200
#define R2 650

int Hd = 0;
int Wd = 0;

cv::Mat dst, map_x, map_y;

void update_map( void )
{
    float r=0.0, theta=0.0, Xs=0.0, Ys=0.0;
    for( int j=0;j<dst.rows;j++)
    {
        for(int i=0;i<dst.cols;i++)
        {
            r=(float)j / (float)Hd *(R2 - R1) + R1;
            theta = static_cast<float>((float)i / (float)Wd * 2 * M_PI);
            Xs = Cx + r * std::sin(theta);
            Ys = Cy + r * std::cos(theta);
            map_x.at<float>(j,i) = Xs;
            map_y.at<float>(j,i) = Ys;
        }
    }
}

int main(int argc, char ** argv){
    cv::Mat omni_img = cv::imread(argv[1]);

    // calculate mapping
    Hd = R2 - R1;
    Wd = static_cast<int>(M_PI * (R1 + R2) + 1);
    dst.create( Hd, Wd, CV_8UC3 );
    map_x.create( dst.size(), CV_32FC1 );
    map_y.create( dst.size(), CV_32FC1 );
    update_map();

    // Remap
    cv::remap( omni_img, dst, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0,0, 0) );

    cv::imwrite("pano.jpg", dst);
    cv::namedWindow("omni", cv::WINDOW_NORMAL);
    cv::imshow("omni", omni_img);
    cv::namedWindow("pano", cv::WINDOW_NORMAL);
    cv::imshow("pano", dst);
    cv::waitKey(0);

    return 0;
}