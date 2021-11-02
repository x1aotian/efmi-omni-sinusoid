#include "fmiRegistration.hh"
//#include "NetworkImageFMIImage.hh"

#include <stdio.h>

#define LOC  fprintf(stderr, "[in %s@line %d] ", __FILE__, __LINE__);
#define ERR(...) do{LOC fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n");} while(0)

#include <math.h>
#include <iostream>
#include <opencv2/core/mat.hpp>

using namespace jacobs_robotics;
using namespace std;
//using namespace network_image;

vector<int> sort_indexes(const vector<double> &v) {

    // 初始化索引向量
    vector<int> idx(v.size());
    //使用iota对向量赋0~？的连续值
    iota(idx.begin(), idx.end(), 0);

    // 通过比较v的值对索引idx进行排序
    sort(idx.begin(), idx.end(),
         [&v](size_t i1, size_t i2) {
             return v[i1] > v[i2];
         });
    return idx;
}

void FMIRegistration::findMax(const FMIReal *in, const unsigned int &size, FMIReal &maxVal, unsigned int &maxCoord) {
    maxVal = *in;
    maxCoord = 0;
    const FMIReal *inP = in;
    for (unsigned int count = 0; count < size; count++, inP++) {
        if (*inP > maxVal) {
            maxVal = *inP;
            maxCoord = count;
        }
    }
}

void FMIRegistration::findMaxRay(const FMIReal *in, const unsigned int &size, FMIReal &maxVal, unsigned int &maxCoord) {
    // the angle resolution should be the divisor of 360
    // and should be the same as the angle_resolution in the init
    int num_tons = 360 / angleResolution;
    std::vector<double> tons(num_tons, 0.);
    std::vector<std::vector<int>> tons_index(num_tons);

    const FMIReal *inP = in;
    const int *rayTons = rayMatrices;
    for (unsigned int count = 0; count < size; count++, inP++) {
        auto idx = static_cast<size_t>(*(rayTons + count));
        tons[idx] += (*inP) * (*inP);
        tons_index[idx].push_back(count);
    }

    std::vector<int> ordered_indices = sort_indexes(tons);

    // search highest peak in this tons, which is for findmultiple peaks
    // later we will use 1D phase shift to replace it
    maxVal = *(in + tons_index[ordered_indices[0]][0]);
    maxCoord = static_cast<unsigned int>(tons_index[ordered_indices[0]][0]);
    int resPlusOne = resolution + 1;
    for (int i : tons_index[ordered_indices[0]]) {
        inP = in + i;

        FMIReal posX = (double) (i % (resPlusOne)) + 1.0;
        FMIReal posY = ceil((double) (i) / (double) (resPlusOne));
//        cout << i << ", " << posX << ", " << posY << ": "<< *inP << endl;

        if (*inP > maxVal) {
            maxVal = *inP;
            maxCoord = static_cast<unsigned int>(i);
        }
    }
    cout << "max cood is " << maxCoord << " "
         << (double) (maxCoord % resPlusOne) + 1.0 << ", "
         << ceil((double) (maxCoord) / (double) (resPlusOne)) << endl;
}

std::vector<double> FMIRegistration::findMaxRayAndSampling(const FMIReal *in, const unsigned int &size, FMIReal &maxVal,
                                                           unsigned int &maxCoord) {

    std::vector<double> sampled1Darray(resolution/2, 0.0);
    // the angle resolution should be the divisor of 360
    // and should be the same as the angle_resolution in the init
    int num_tons = 360 / angleResolution;
    int resPlusOne = resolution + 1;
    std::vector<double> tons(num_tons, 0.);
    std::vector<std::vector<int>> tons_index(num_tons);

    const FMIReal *inP = in;
    const int *rayTons = rayMatrices;
    for (unsigned int count = 0; count < size; count++, inP++) {
        auto idx = static_cast<size_t>(*(rayTons + count));
        tons[idx] += (*inP) * (*inP);
        tons_index[idx].push_back(count);
    }

    std::vector<int> ordered_indices = sort_indexes(tons);

    // search highest peak in this tons, which is for findmultiple peaks
    // later we will use 1D phase shift to replace it
    maxVal = *(in + tons_index[ordered_indices[0]][0]);
    maxCoord = static_cast<unsigned int>(tons_index[ordered_indices[0]][0]);
    for (int i : tons_index[ordered_indices[0]]) {
        inP = in + i;
        if (*inP > maxVal) {
            maxVal = *inP;
            maxCoord = static_cast<unsigned int>(i);
        }
    }
//    cout << "max cood is " << maxCoord << " "
//         << (double) (maxCoord % resPlusOne) + 1.0 << ", "
//         << ceil((double) (maxCoord) / (double) (resPlusOne)) << endl;

    // the ordered_indices[0]^{th} ton
    // the tons[ordered_indices[0]] stores the maximum square sum energy
    // the tons_index[ordered_indices[0]] stores all the indices corresponding to the maximum square sum energy
    // calculate the distances between the center and pixels in this tons
    double max_angle = (ordered_indices[0] * angleResolution + angleResolution / 2.) / 180. * M_PI;
    int num_of_len = 0;
    if (fabs(max_angle - M_PI / 2.) < 1e-5 || fabs(max_angle - 3. * M_PI / 2.) < 1e-5) {
        num_of_len = resolution / 2;
    } else {
        num_of_len = abs(static_cast<int>(ceil((double) resolution / (2. * cos(max_angle)))));
        if(num_of_len > 0.7071*resolution){
            num_of_len = abs(static_cast<int>(ceil((double) resolution / (2. * sin(max_angle)))));
        }
    }
    std::vector<std::vector<double>> lens(num_of_len);
    for (int i : tons_index[ordered_indices[0]]) {
        inP = in + i;
        FMIReal posX = (double) (i % (resPlusOne)) + 1.0;
        FMIReal posY = ceil((double) (i) / (double) (resPlusOne));
        auto dis = static_cast<int>(sqrt(pow(posX - 1 - resPlusOne / 2, 2) + pow(posY - 1 - resPlusOne / 2, 2)));
        if(dis >= num_of_len){
            continue;
        }
        lens[dis].push_back(*inP);
    }
    cv::Mat array1D = cv::Mat::zeros(1, num_of_len, CV_64F);
    for (int i = 0; i < num_of_len; ++i) {
        if(!lens[i].empty()){
//            array1D.at<double>(0, i) = std::accumulate(lens[i].begin(), lens[i].end(), 0.0) / (double) lens[i].size();
            double max_value = *max_element(lens[i].begin(), lens[i].end());
            if(max_value > 0.3*maxVal){
                array1D.at<double>(0, i) = max_value;
            }
        }
    }

//    cout << "array1D : ";
//    for (int i = 0; i < num_of_len; ++i) {
//        cout << array1D.at<double>(0, i) << " ";
//    }
//    cout << endl;

    cv::Mat resized_array1D = cv::Mat::zeros(1, resolution/2, CV_64F);
    cv::resize(array1D, resized_array1D, resized_array1D.size());

//    cout << "resized_array1D : ";
//    for (int i = 0; i < resolution/2; ++i) {
//        cout << resized_array1D.at<double>(0, i) << " ";
//    }
//    cout << endl;

    sampled1Darray[0] = array1D.at<double>(0, 0);
    for(int i = 1; i < resolution/2; ++i) {
        sampled1Darray[i] = resized_array1D.at<double>(0, i);
    }

    return sampled1Darray;
}

std::vector<double> FMIRegistration::findMaxRayAndSampling_All(const FMIReal *in, const unsigned int &size, FMIReal &maxVal,
                                                            unsigned int &maxCoord, double &maxAngle) {
    //std::vector<double> sampled1Darray(resolution, 0.0);

    int num_tons = 360 / angleResolution;
    int resPlusOne = resolution + 1;
    std::vector<double> tons(num_tons, 0.);
    std::vector<std::vector<int>> tons_index(num_tons);

    const FMIReal *inP = in;
    const int *rayTons = rayMatrices;
    for (unsigned int count = 0; count < size; count++, inP++) {
        auto idx = static_cast<size_t>(*(rayTons + count));
        tons[idx] += (*inP) * (*inP); // +=energy
        tons_index[idx].push_back(count); // tons_index[i] to store all indexes in i-th angle
    }

    std::vector<int> ordered_indices = sort_indexes(tons); // sort tons

    maxVal = *(in + tons_index[ordered_indices[0]][0]);
    maxCoord = static_cast<unsigned int>(tons_index[ordered_indices[0]][0]);
    for (int i : tons_index[ordered_indices[0]]) {
        inP = in + i;
        if (*inP > maxVal) {
            maxVal = *inP;
            maxCoord = static_cast<unsigned int>(i);
        }
    }
    //cout << "maxVal: " << maxVal << endl;

    //cout << "max index: " << ordered_indices[0] << endl;
    double max_angle = (ordered_indices[0] * angleResolution + angleResolution / 2.) / 180. * M_PI;

    maxAngle = max_angle;

    int num_of_len = 0;
    if (fabs(max_angle - M_PI / 2.) < 1e-5 || fabs(max_angle - 3. * M_PI / 2.) < 1e-5
        || fabs(max_angle - M_PI) < 1e-5 || fabs(max_angle - 2. * M_PI) < 1e-5 ) {
        num_of_len = resolution / 2;
    } else {
        num_of_len = abs(static_cast<int>(ceil((double) resolution / (2. * cos(max_angle)))));
        if(num_of_len > 0.7071*resolution){
            num_of_len = abs(static_cast<int>(ceil((double) resolution / (2. * sin(max_angle)))));
        }
    }

    //std::cout << "max_angle: " << max_angle << std::endl;

    //double diag_angle = max_angle + M_PI;

    int len_size = 2 * num_of_len - 1; // because the whole line needs to be recorded, not the half.
    int cen_index = num_of_len - 1;
    int diag_index = num_tons - 1 - ordered_indices[0];

//    cout << "! max_angle: " << max_angle << endl;
//    cout << "! len_size: " << len_size << endl;
//    cout << "! len_size*cos: " << len_size * cos(max_angle) << endl;
//    cout << "! len_size*sin: " << len_size * sin(max_angle) << endl;

    std::vector<std::vector<double>> lens(len_size); // lens = 1st version of trans energy vector
    for (int i : tons_index[ordered_indices[0]]) {
        inP = in + i;
        FMIReal posX = (double) (i % (resPlusOne)) + 1.0;
        FMIReal posY = ceil((double) (i) / (double) (resPlusOne));
        auto dis = static_cast<int>(sqrt(pow(posX - 1 - resPlusOne / 2, 2) + pow(posY - 1 - resPlusOne / 2, 2))); // maybe no robust
        if(dis >= num_of_len){
            continue;
        }
        lens[cen_index + dis].push_back(*inP);
        //cout << cen_index + dis << " " << *inP << endl;
    }

    for (int i : tons_index[diag_index]) {
        inP = in + i;
        FMIReal posX = (double) (i % (resPlusOne)) + 1.0;
        FMIReal posY = ceil((double) (i) / (double) (resPlusOne));
        auto dis = static_cast<int>(sqrt(pow(posX - 1 - resPlusOne / 2, 2) + pow(posY - 1 - resPlusOne / 2, 2)));
        if(dis >= num_of_len){
            continue;
        }
        lens[cen_index - dis].push_back(*inP);
        //cout << cen_index - dis << " " << *inP << endl;
    }

    cv::Mat array1D = cv::Mat::zeros(1, len_size, CV_64F);
    for (int i = 0; i < len_size; ++i) {
        if(!lens[i].empty()){
//            array1D.at<double>(0, i) = std::accumulate(lens[i].begin(), lens[i].end(), 0.0) / (double) lens[i].size();
            double max_value = *max_element(lens[i].begin(), lens[i].end());  // WHY??
            if(max_value > 0.3*maxVal){
                array1D.at<double>(0, i) = max_value;
                //cout << i << " " << max_value<< endl;
            }
        }
        //cout << array1D.at<double>(0, i) << endl;
    }

    //cv::Mat resized_array1D = cv::Mat::zeros(1, resolution, CV_64F);
    //cv::resize(array1D, resized_array1D, resized_array1D.size());

    int res_size = len_size;
    std::vector<double> sampled1Darray (res_size, 0.0);
    for(int i = 0; i < res_size; ++i) {
        sampled1Darray[i] = array1D.at<double>(0, i); // array1D or resized_array1D. I THINK it should be no resize
    }

    return sampled1Darray;
}


void FMIRegistration::findMaxByAngle(const FMIReal *in,
                                     const unsigned int &resolution,
                                     FMIReal &maxVal,
                                     unsigned int &maxCoord) {
    unsigned int cx = resolution / 2;
    unsigned int cy = resolution / 2;

    // Determine whether the peak is in the center
    FMIReal p_maxVal;
    unsigned int p_maxCoord;
    findMax(in, resolution * resolution, p_maxVal, p_maxCoord);
    FMIReal posX = (double) (p_maxCoord % resolution) + 1.0;
    FMIReal posY = ceil((double) (p_maxCoord) / (double) (resolution));
    if (sqrt(pow(posX - cx, 2) + pow(posY - cy, 2)) < 5.) {
        maxVal = p_maxVal;
        maxCoord = p_maxCoord;
        return;
    }

    // one degree per region
    double max_value = 0.0;
    double max_angle = 0.0;
    unsigned int max_cood = 0;
    const FMIReal *inP = in;
    for (int i = 0; i < 360; ++i) {
        double sum = 0;
        unsigned int cood = 0;
        double theta = double(i) / 180. * M_PI;
        double line_max = 0.;
        unsigned int line_idx = 0;
        if (fabs(theta - M_PI / 2.) < 1e-6) {
            for (int y = 0; y < cy; ++y) {
                int v = cy + y;
                inP = in + v * resolution + cx;
                sum += pow(*inP, 2);
                if (*inP > line_max) {
                    line_max = *inP;
                    line_idx = static_cast<unsigned int>(v);
                }
            }
            cood = line_idx * resolution + cx;
        } else if (fabs(theta - M_PI * 1.5) < 1e-6) {
            for (int y = 0; y < cy; ++y) {
                int v = cy - y;
                inP = in + v * resolution + cx;
                sum += pow(*inP, 2);
                if (*inP > line_max) {
                    line_max = *inP;
                    line_idx = static_cast<unsigned int>(v);
                }
            }
            cood = line_idx * resolution + cx;
        } else if (theta < M_PI / 2. || theta > M_PI * 1.5) {
            if (theta <= M_PI / 4. || theta >= M_PI * 7. / 4.) {
                for (int x = 0; x < cx; ++x) {
                    int u = cx + x;
                    auto v = static_cast<int>(cy + floor((tan(theta) * x)));
                    if (v >= 0 && v < resolution) {
                        inP = in + v * resolution + u;
                        sum += pow(*inP, 2);
                        if (*inP > line_max) {
                            line_max = *inP;
                            line_idx = static_cast<unsigned int>(x);
                        }
                    }
                }
                cood = static_cast<unsigned int>((cy + floor(tan(theta) * line_idx)) * resolution + cx + line_idx);
            } else if (theta > M_PI * 1.5) {
                for (int y = 0; y < cy; ++y) {
                    int v = cy - y;
                    auto u = static_cast<int>(cx - floor(y / tan(theta)));
                    if (u >= 0 && u < resolution) {
                        inP = in + v * resolution + u;
                        sum += pow(*inP, 2);
                        if (*inP > line_max) {
                            line_max = *inP;
                            line_idx = static_cast<unsigned int>(y);
                        }
                    }
                }
                cood = static_cast<unsigned int>((cy - line_idx) * resolution + cx - floor(line_idx / tan(theta)));
            } else {
                for (int y = 0; y < cy; ++y) {
                    int v = cy + y;
                    auto u = static_cast<int>(cx + y / tan(theta));
                    if (u >= 0 && u < resolution) {
                        inP = in + v * resolution + u;
                        sum += pow(*inP, 2);
                        if (*inP > line_max) {
                            line_max = *inP;
                            line_idx = static_cast<unsigned int>(y);
                        }
                    }
                }
                cood = static_cast<unsigned int>((cy + line_idx) * resolution + cx + line_idx / tan(theta));
            }
        } else {
            if (theta >= 3. * M_PI / 4. && theta <= 5. * M_PI / 4.) {
                for (int x = 0; x < cx; ++x) {
                    int u = cx - x;
                    auto v = static_cast<int>(cy - floor(tan(theta) * x));
                    if (v >= 0 && v < resolution) {
                        inP = in + v * resolution + u;
                        sum += pow(*inP, 2);
                        if (*inP > line_max) {
                            line_max = *inP;
                            line_idx = static_cast<unsigned int>(x);
                        }
                    }
                }

                cood = static_cast<unsigned int>((cy - floor(tan(theta) * line_idx)) * resolution + cx - line_idx);
            } else if (theta < 3. * M_PI / 4.) {
                for (int y = 0; y < cy; ++y) {
                    int v = cy + y;
                    auto u = static_cast<int>(cx + y / tan(theta));
                    if (u >= 0 && u < resolution) {
                        inP = in + v * resolution + u;
                        sum += pow(*inP, 2);
                        if (*inP > line_max) {
                            line_max = *inP;
                            line_idx = static_cast<unsigned int>(y);
                        }
                    }
                }
                cood = static_cast<unsigned int>((cy + line_idx) * resolution + cx + line_idx / tan(theta));
            } else {
                for (int y = 0; y < cy; ++y) {
                    int v = cy - y;
                    auto u = static_cast<int>(cx - floor(y / tan(theta)));
                    if (u >= 0 && u < resolution) {
                        inP = in + v * resolution + u;
                        sum += pow(*inP, 2);
                        if (*inP > line_max) {
                            line_max = *inP;
                            line_idx = static_cast<unsigned int>(y);
                        }
                    }
                }
                cood = static_cast<unsigned int>((cy - line_idx) * resolution + cx - floor(line_idx / tan(theta)));
            }
        }

        if (sum > max_value) {
            max_value = sum;
            max_angle = theta;
            max_cood = cood;
            maxVal = line_max;
        }
    }
    cout << "max value is " << maxVal << endl;
    cout << "max angle is " << max_angle << endl;
    cout << "max cood is " << max_cood << " "
         << (double) (max_cood % resolution) + 1.0 << ", "
         << ceil((double) (max_cood) / (double) (resolution)) << endl;
    maxCoord = max_cood;
}

// 3x3 matrix multiplication
void FMIRegistration::matMult3x3(const double in1[3][3], const double in2[3][3], double out[3][3]) {
    for (unsigned int m = 0; m < 3; m++) {
        for (unsigned int k = 0; k < 3; k++) {
            out[m][k] = in1[m][0] * in2[0][k] + in1[m][1] * in2[1][k] + in1[m][2] * in2[2][k];
        }
    }
}

void FMIRegistration::init() {
    create2DSpectralWindow();
    createPolarLogInterpolationTables();
    create1DGaussianWindow();
    createRayTons();
}

// Lines 488 - 510 in FMI_reg.cc
void FMIRegistration::create2DSpectralWindow() {
    FMIReal *wind2D = new FMIReal[squaredResolution];
    // first we generate a 1D 
    FMIReal window1D[resolution];
    // TODO: play around here, maybe other values work better!?
    for (unsigned int i = 0; i < resolution; i++) {
        window1D[i] = 0.5 * (1 - cos(2. * M_PI * double(i + 1) / double(resolution + 1)));
    }
    // now calculate the 2D spectral window
    for (unsigned int y = 0; y < resolution; y++) {
        for (unsigned int x = 0; x < resolution; x++) {
            wind2D[y * resolution + x] = window1D[x] * window1D[y];
        }
    }
    spectralWindow2D = wind2D;
}

void FMIRegistration::createPolarLogInterpolationTables() {

    FMIReal *polLogIntMat = new FMIReal[squaredPolLogRes * 4];
    unsigned int *polLogIntCoords = new unsigned int[squaredPolLogRes * 4];

    FMIReal *tempPowTab = new FMIReal[polarLogResolution];

    // Polar-Log resampling - pre-calculations
    const double imageResolution = static_cast<double>(resolution);
    const double polLogRes = static_cast<double>(polarLogResolution);

    const double f1 = (imageResolution / 2. - 1.) / (polLogRes - 1.);
    const double f2 = (imageResolution / 2. + 1.);
    // 1-D temporary lookup-table for the logarithmic sampling
    for (unsigned int m = 0; m < polarLogResolution; m++) { // radial coordinate
        tempPowTab[m] = pow(polLogRes - 1., (static_cast<double>(m) / (polLogRes - 1.)));
    }
    FMIReal u, v, xp1, xp2, yp1, yp2;
    // generate interpolation coordinates/weightings table
    for (unsigned int m = 0; m < polarLogResolution; m++) { // radial coordinate
        for (unsigned int k = 0; k < polarLogResolution; k++) { // angular coordinate

            // u is the real x-coordinate for the lookup-point
            u = f1 * tempPowTab[m] * cos((M_PI * (double) k) / polLogRes) + f2;
            // v is the real y-coordinate for the lookup-point
            v = f1 * tempPowTab[m] * sin((M_PI * (double) k) / polLogRes) + f2;

            // Now the coordinates X1, X2, Y1 and Y2 for the four neighboring points around u and v are calculated
            // first the y values
            // we pre-multiply the y values with the resolution...
            // (any call would lateron look like this: mat[y * resolution + x] - so we premultiply with resolution to save computation time lateron
            polLogIntCoords[0 + 4 * m + (polarLogResolution * 4) * k] = static_cast<int>(ceil(u) - 1.0) * resolution;
            polLogIntCoords[1 + 4 * m + (polarLogResolution * 4) * k] = static_cast<int>(floor(u) - 1.0) * resolution;
            // now the x values
            polLogIntCoords[2 + 4 * m + (polarLogResolution * 4) * k] = static_cast<int>(ceil(v) - 1.0);
            polLogIntCoords[3 + 4 * m + (polarLogResolution * 4) * k] = static_cast<int>(floor(v) - 1.0);

            // now we calculate the weight for each of the four points
            // first just 1-dimensional for each axis
            xp1 = 1.0 - (v - floor(v));
            xp2 = v - floor(v);
            yp1 = 1.0 - (u - floor(u));
            yp2 = u - floor(u);
            // now with 2D weights by simply multiplying
            polLogIntMat[0 + 4 * m + (polarLogResolution * 4) * k] = xp1 * yp1;
            polLogIntMat[1 + 4 * m + (polarLogResolution * 4) * k] = xp2 * yp1;
            polLogIntMat[2 + 4 * m + (polarLogResolution * 4) * k] = xp1 * yp2;
            polLogIntMat[3 + 4 * m + (polarLogResolution * 4) * k] = xp2 * yp2;

        }
    }
    // we actually need the tables in transposed order
    // so we have to resort the stupid matrices...
    // TODO: create the matrices correctly in the first place
    FMIReal *polLogIntMat2 = new FMIReal[squaredPolLogRes * 4];
    unsigned int *polLogIntCoords2 = new unsigned int[squaredPolLogRes * 4];
    for (unsigned int m = 0; m < polarLogResolution; m++) { // radial coordinate
        for (unsigned int k = 0; k < polarLogResolution; k++) { // angular coordinate
            polLogIntMat2[0 + 4 * m + (polarLogResolution * 4) * k] = polLogIntMat[0 + 4 * k +
                                                                                   (polarLogResolution * 4) * m];
            polLogIntMat2[1 + 4 * m + (polarLogResolution * 4) * k] = polLogIntMat[1 + 4 * k +
                                                                                   (polarLogResolution * 4) * m];
            polLogIntMat2[2 + 4 * m + (polarLogResolution * 4) * k] = polLogIntMat[2 + 4 * k +
                                                                                   (polarLogResolution * 4) * m];
            polLogIntMat2[3 + 4 * m + (polarLogResolution * 4) * k] = polLogIntMat[3 + 4 * k +
                                                                                   (polarLogResolution * 4) * m];

            polLogIntCoords2[0 + 4 * m + (polarLogResolution * 4) * k] = polLogIntCoords[0 + 4 * k +
                                                                                         (polarLogResolution * 4) * m];
            polLogIntCoords2[1 + 4 * m + (polarLogResolution * 4) * k] = polLogIntCoords[1 + 4 * k +
                                                                                         (polarLogResolution * 4) * m];
            polLogIntCoords2[2 + 4 * m + (polarLogResolution * 4) * k] = polLogIntCoords[2 + 4 * k +
                                                                                         (polarLogResolution * 4) * m];
            polLogIntCoords2[3 + 4 * m + (polarLogResolution * 4) * k] = polLogIntCoords[3 + 4 * k +
                                                                                         (polarLogResolution * 4) * m];
        }
    }
    delete polLogIntMat;
    delete polLogIntCoords;
    polLogInterpolationMatrices = polLogIntMat2;
    polLogInterpolationCoordinates = polLogIntCoords2;
    delete tempPowTab;
}


void FMIRegistration::create1DGaussianWindow() {
    FMIReal *wind1D = new FMIReal[polarLogResolution];
    FMIReal *windP = wind1D;
    int m = -((int) floor((double) polarLogResolution / 2.0));
    int end = (int) ceil((double) (polarLogResolution - 1) / 2.0);
    double resSquareVal = 2.0 * (0.3 * (double) polarLogResolution * 0.3 * (double) polarLogResolution);
    for (; m < end; m++, windP++) {
        *windP = 1.0 / (2.0 * M_PI) * exp(-((double) (m * m) / (resSquareVal)));
    }

    unsigned int maxCoord = 0;
    FMIReal maxVal;
    findMax(wind1D, polarLogResolution, maxVal, maxCoord);
    FMIReal oneOverMax = 1. / maxVal;
    // normalize
    windP = wind1D;
    const FMIReal *const endP = windP + polarLogResolution;
    for (; windP < endP; windP++) {
        *windP *= oneOverMax;
    }
    gaussianWindow1D = wind1D;
}

void FMIRegistration::createRayTons() {
    int num_tons = 360 / angleResolution;
    int resPlusOne = resolution + 1;
    int size = (resolution + 1) * (resolution + 1);

    int cx = resPlusOne / 2;
    int cy = cx;

    auto *rayMatrices2 = new int[size];
    for (unsigned int count = 0; count < size; count++) {
        FMIReal posX = (double) (count % resPlusOne) + 1.0;
        FMIReal posY = ceil((double) (count) / (double) (resPlusOne));

        auto u = posX - 1 - cx;
        auto v = posY - 1 - cy;

        if (abs(u) < 1e-6) { // k = infinity
            if (v < 0) {
                rayMatrices2[count] = num_tons / 4;
            } else {
                rayMatrices2[count] = 3 * num_tons / 4;
            }
        } else {
            double theta = atan2((double) v, (double) u);
            if (theta < 0) {
                theta += 2 * M_PI;
            }
            rayMatrices2[count] = static_cast<int>((theta * 180. / M_PI) / (double) angleResolution);
        }
    }
    rayMatrices = rayMatrices2;
}











