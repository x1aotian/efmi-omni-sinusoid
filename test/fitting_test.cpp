//
// Created by xuqw on 12/6/19.
//

#include <string>
#include <iostream>
#include <fstream>
#include <ceres/ceres.h>
#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

cv::Mat display_wotrans_worot(int height, int width,
                              std::vector<double> idxs,
                              std::vector<double> delta_u,
                              double param_u[3]) {
    double scale = 10;
    cv::Mat show_img = cv::Mat(height * scale, width, CV_8UC3, cv::Scalar(255, 255, 255));
    //plot coordinates
    for (int i = -10; i <= 10; ++i) {
        cv::line(show_img, cv::Point(0, (i * 10 + height / 2) * scale), cv::Point(width, (i * 10 + height / 2) * scale),
                 cv::Scalar(144, 144, 144));
        cv::putText(show_img, std::to_string(i * 10), cv::Point(0, (i * 10 + height / 2) * scale),
                    cv::FONT_HERSHEY_SIMPLEX, 1.,
                    cv::Scalar(144, 144, 144));
    }
    for (int i = 0; i < idxs.size(); ++i) {
        cv::circle(show_img, cv::Point(idxs[i], (delta_u[i] + height / 2) * scale), 5, cv::Scalar(255, 0, 0), 3);
    }
    for (int j = 0; j < width; ++j) {
        double delta_u = param_u[0] * std::sin((2 * M_PI / width) * j + param_u[1]) + param_u[2];
        cv::circle(show_img, cv::Point(j, (delta_u + height / 2) * scale), 1, cv::Scalar(0, 0, 255), 1);
    }
    return show_img;
}

struct SinusoidCostFunction {
    SinusoidCostFunction(double t, double y, double T) : t_(t), y_(y), T_(T) {}

    // Fit Function : f(t) = A * sin(w * t + p) + c
    // Parameters
    // Parameters[0] : Amplitude (A)
    // Parameters[1] : phase (p)
    // Parameters[2] : offset (c)
    template <typename T>
    bool operator() (const T* const parameters, T* residual) const {
        residual[0] = T(y_) - (parameters[0] * ceres::sin((2 * M_PI / T_) * t_ + parameters[1]) + parameters[2]);
        return true;
    }

private:
    double t_, y_;
    double T_;
};

int main(int argc, char ** argv){
    std::ifstream file("/home/xuqw/pitch2_6_shift.txt");
//    std::ifstream file("/home/xuqw/gitlab/code/omni-sinusoid/omni-sinusoid/bin/test.txt");

    std::vector<double> data;
    std::vector<double> delta_x;
    std::vector<double> delta_y;

    for (std::string line; getline(file, line);) {

        std::istringstream in(line);
        std::string str;
        int count = 0;

        while (in >> str) {
            if (count == 0) {
                data.push_back(stod(str));
            } else if (count == 1) {
                delta_x.push_back(stod(str));
            } else if (count == 2) {
                delta_y.push_back(stod(str));
            }
            count++;
        }
    }

    file.close();

    // add noise
    for (int j = 0; j < delta_y.size(); ++j) {
        delta_y[j] += 0.01*(rand()%10);
    }
    for (int j = 1; j < 11; ++j) {
        delta_y[j] = 0;
    }

    double parameters[3] = {0.0, 0.0, 0.0};
    ceres::Problem problem;
    for (int i = 0; i < data.size(); ++i) {
        ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<SinusoidCostFunction, 1, 3> (new SinusoidCostFunction(data[i], delta_y[i], 1100.0));
        problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.5), parameters);
    }

    // Solver
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-8;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // results
    std::cout << summary.FullReport() << std::endl;
    std::cout << "estimated Amplitude, phase, offset = ";
    for (auto a:parameters) {
        std::cout << a << " ";
    }
    std::cout << std::endl;

    // ransac
    int max_iter = 50;
    int cnt = 5;
    double last_min_cost = INFINITY;
    double last_cost = INFINITY;
    double param[3] = {0., 0., 0.};
    for (int k = 0; k < max_iter; ++k) {
        std::vector<double> sample_i(cnt);
        std::vector<double> sample_y(cnt);
        for (int j = 0; j < cnt; ++j) {
            int idx = rand()%data.size();
            sample_i[j] = data[idx];
            sample_y[j] = delta_y[idx];
        }
        double sub_param[3] = {0.0, 0.0, 0.0};
        ceres::Problem sub_problem;
        for (int i = 0; i < cnt; ++i) {
            ceres::CostFunction* cost_function =
                    new ceres::AutoDiffCostFunction<SinusoidCostFunction, 1, 3> (new SinusoidCostFunction(sample_i[i], sample_y[i], 1100.0));
            sub_problem.AddResidualBlock(cost_function, NULL, sub_param);
        }
        ceres::Solver::Summary sub_summary;
        ceres::Solve(options, &sub_problem, &sub_summary);

        //calculate distance
        double cost = 0.;
        for (int i = 0; i < data.size(); ++i) {
            cost += fabs(delta_y[i] - (sub_param[0] * ceres::sin((2 * M_PI / 1100.0) * data[i] + sub_param[1]) + sub_param[2]));
        }

        if(cost < last_cost){
            last_cost = cost;
            param[0] = sub_param[0];
            param[1] = sub_param[1];
            param[2] = sub_param[2];
            if(fabs(last_min_cost - last_cost) < 1e-5){
                last_cost = cost;
                param[0] = sub_param[0];
                param[1] = sub_param[1];
                param[2] = sub_param[2];
                break;
            }
            last_min_cost = last_cost;
        }
    }
    std::cout << "estimated Amplitude, phase, offset = " << param[0] << ", " <<  param[1] << ", " <<  param[2] << std::endl;

    cv::Mat show_image = display_wotrans_worot(100, 1100,
                                 data,
                                 delta_y, parameters);
    cv::Mat show_image_2 = display_wotrans_worot(100, 1100,
                                               data,
                                               delta_y, param);

    cv::namedWindow("fitting", CV_WINDOW_NORMAL);
    cv::imshow("fitting", show_image);
    cv::namedWindow("fitting 2", CV_WINDOW_NORMAL);
    cv::imshow("fitting 2", show_image_2);
    cv::waitKey(0);
}
