//
// Created by xuqw on 12/26/19.
//

#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define Cx 544
#define Cy 544
#define R 544

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
            r=(float)j / (float)Hd * R;
            theta = static_cast<float>((float)i / (float)Wd * 2 * M_PI);
            Xs = Cx + r * std::sin(theta);
            Ys = Cy + r * std::cos(theta);
            map_x.at<float>(j,i) = Xs;
            map_y.at<float>(j,i) = Ys;
        }
    }
}

int main(int argc, char ** argv){
    cv::Mat fish_img = cv::imread(argv[1]);

    // calculate mapping
    Hd = R;
    Wd = static_cast<int>(2* M_PI * R);
    dst.create( Hd, Wd, CV_8UC3 );
    map_x.create( dst.size(), CV_32FC1 );
    map_y.create( dst.size(), CV_32FC1 );
    update_map();

    // Remap
    cv::remap( fish_img, dst, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0,0, 0) );

    cv::namedWindow("fisheye", cv::WINDOW_NORMAL);
    cv::imshow("fisheye", fish_img);
    cv::namedWindow("pano", cv::WINDOW_NORMAL);
    cv::imshow("pano", dst);
    cv::waitKey(0);

    return 0;
}