//
// Created by xuqw on 6/11/20.
//

#ifndef OMNI_SINUSOID_MISC_0_H
#define OMNI_SINUSOID_MISC_0_H

#include <vector>
#include <deque>
#include "fftMath.hh"

using namespace jacobs_robotics;

void phaseShift1D(const FMIReal *in1, const FMIReal *in2, ComplexFMIReal *out, int resolution) {

    gsl_fft_complex_wavetable *gslWavetable;
    gsl_fft_complex_workspace *gslWorkspace;

    gslWavetable = gsl_fft_complex_wavetable_alloc(static_cast<size_t>(resolution));
    gslWorkspace = gsl_fft_complex_workspace_alloc(static_cast<size_t>(resolution));


    // copy to complex data structure or we can use real data directly
    auto *complex_x = new ComplexFMIReal[resolution];
    ComplexFMIReal *outx = complex_x;
    const FMIReal *inPtr1 = in1;
    for (int i = 0; i < resolution; ++i) {
        outx->real = *inPtr1;
        outx->imag = 0.;
        outx++;
        inPtr1++;
    }

    auto *complex_y = new ComplexFMIReal[resolution];
    ComplexFMIReal *outy = complex_y;
    const FMIReal *inPtr2 = in2;
    for (int i = 0; i < resolution; ++i) {
        outy->real = *inPtr2;
        outy->imag = 0.;
        outy++;
        inPtr2++;
    }

    gsl_fft_complex_forward(reinterpret_cast<double *>(complex_x), 1,
                            static_cast<const size_t>(resolution),
                            gslWavetable,
                            gslWorkspace);

    gsl_fft_complex_forward(reinterpret_cast<double *>(complex_y), 1,
                            static_cast<const size_t>(resolution),
                            gslWavetable,
                            gslWorkspace);

    // phase correlation: (f1 x f2*)/(abs(f1) x abs(f2))
    outx = complex_x;
    outy = complex_y;
    double eps = 1e-15;
    auto *correlated = new ComplexFMIReal[resolution];
    ComplexFMIReal *corr_xy = correlated;
    for (int i = 0; i < resolution; ++i) {
        corr_xy->real = (outx->real * outy->real + outx->imag * outy->imag);
        corr_xy->imag = (outx->imag * outy->real - outx->real * outy->imag);
        double denom = sqrt(pow(corr_xy->real, 2) + pow(corr_xy->imag, 2)) + eps;
        corr_xy->real /= denom;
        corr_xy->imag /= denom;
        outx++;
        outy++;
        corr_xy++;
    }
    gsl_fft_complex_backward(reinterpret_cast<double *>(correlated), 1,
                             static_cast<const size_t>(resolution),
                             gslWavetable,
                             gslWorkspace);

    // fft shift
    corr_xy = correlated + resolution / 2;
//    auto * shifted = new ComplexFMIReal[resolution];
    ComplexFMIReal *shift = out;
    for (int i = 0; i < resolution / 2; ++i) {
        shift->real = corr_xy->real;
        shift->imag = corr_xy->imag;
        shift++;
        corr_xy++;
    }
    corr_xy = correlated;
    for (int j = 0; j < resolution / 2; ++j) {
        shift->real = corr_xy->real;
        shift->imag = corr_xy->imag;
        shift++;
        corr_xy++;
    }

    gsl_fft_complex_wavetable_free(gslWavetable);
    gsl_fft_complex_workspace_free(gslWorkspace);
}

void phaseShift1D(const std::vector<double> &in1,
                  const std::vector<double> &in2,
                  ComplexFMIReal *out,
                  int resolution) {
    gsl_fft_complex_wavetable *gslWavetable;
    gsl_fft_complex_workspace *gslWorkspace;

    gslWavetable = gsl_fft_complex_wavetable_alloc(static_cast<size_t>(resolution));
    gslWorkspace = gsl_fft_complex_workspace_alloc(static_cast<size_t>(resolution));


    // copy to complex data structure or we can use real data directly
    auto *complex_x = new ComplexFMIReal[resolution];
    ComplexFMIReal *outx = complex_x;
    for (int i = 0; i < resolution; ++i) {
        outx->real = in1[i];
        outx->imag = 0.;
        outx++;
    }

    auto *complex_y = new ComplexFMIReal[resolution];
    ComplexFMIReal *outy = complex_y;
    for (int i = 0; i < resolution; ++i) {
        outy->real = in2[i];
        outy->imag = 0.;
        outy++;
    }

    gsl_fft_complex_forward(reinterpret_cast<double *>(complex_x), 1,
                            static_cast<const size_t>(resolution),
                            gslWavetable,
                            gslWorkspace);

    gsl_fft_complex_forward(reinterpret_cast<double *>(complex_y), 1,
                            static_cast<const size_t>(resolution),
                            gslWavetable,
                            gslWorkspace);

    // phase correlation: (f1 x f2*)/(abs(f1) x abs(f2))
    outx = complex_x;
    outy = complex_y;
    double eps = 1e-15;
    auto *correlated = new ComplexFMIReal[resolution];
    ComplexFMIReal *corr_xy = correlated;
    for (int i = 0; i < resolution; ++i) {
        corr_xy->real = (outx->real * outy->real + outx->imag * outy->imag);
        corr_xy->imag = (outx->imag * outy->real - outx->real * outy->imag);
        double denom = sqrt(pow(corr_xy->real, 2) + pow(corr_xy->imag, 2)) + eps;
        corr_xy->real /= denom;
        corr_xy->imag /= denom;
        outx++;
        outy++;
        corr_xy++;
    }
    gsl_fft_complex_backward(reinterpret_cast<double *>(correlated), 1,
                             static_cast<const size_t>(resolution),
                             gslWavetable,
                             gslWorkspace);

    // fft shift
    corr_xy = correlated + resolution / 2;
//    auto * shifted = new ComplexFMIReal[resolution];
    ComplexFMIReal *shift = out;
    for (int i = 0; i < resolution / 2; ++i) {
        shift->real = corr_xy->real;
        shift->imag = corr_xy->imag;
        shift++;
        corr_xy++;
    }
    corr_xy = correlated;
    for (int j = 0; j < resolution / 2; ++j) {
        shift->real = corr_xy->real;
        shift->imag = corr_xy->imag;
        shift++;
        corr_xy++;
    }

    gsl_fft_complex_wavetable_free(gslWavetable);
    gsl_fft_complex_workspace_free(gslWorkspace);
}

int search1D(const std::vector<double> &in1, const std::vector<double> &in2) {
    int shift = -static_cast<int>(in1.size());

    std::deque<double> container1(2 * in1.size(), 0);
    for (size_t i = in1.size(); i < 2 * in1.size(); ++i) {
        container1[i] = in1[i - in1.size()];
    }
    std::deque<double> container2(2 * in2.size(), 0);
    for (size_t i = 0; i < in2.size(); ++i) {
        container2[i] = in2[i];
    }

    int min_shift = 0;
    double min_dis = 10000;
    // interval 1
    for (size_t i = 0; i < in1.size(); ++i) {
        double dis = 0.;
        for (size_t j = 0; j < container1.size(); ++j) {
            dis += fabs(container1[j] - container2[j]);
        }

        if (dis < min_dis) {
            min_dis = dis;
            min_shift = shift;
        }

        shift++;
        container1.pop_front();
        container1.push_back(0.);
    }
    //interval 2
    for (size_t i = in1.size(); i < 2 * in1.size(); ++i) {
        double dis = 0.;
        for (size_t j = 0; j < container1.size(); ++j) {
            dis += fabs(container1[j] - container2[j]);
        }

        if (dis < min_dis) {
            min_dis = dis;
            min_shift = shift;
        }

        shift++;
        container2.pop_back();
        container2.push_front(0.);
    }

    return min_shift;
}

// Returns interpolated value at x from parallel arrays ( xData, yData )
//   Assumes that xData has at least two elements, is sorted and is strictly monotonic increasing
//   boolean argument extrapolate determines behaviour beyond ends of array (if needed)

double interpolate(std::vector<double> &xData, std::vector<double> &yData, double x, bool extrapolate) {
    int size = xData.size();

    int i = 0;                                                                  // find left end of interval for interpolation
    if (x >= xData[size - 2])                                                 // special case: beyond right end
    {
        i = size - 2;
    } else {
        while (x > xData[i + 1]) i++;
    }
    double xL = xData[i], yL = yData[i], xR = xData[i + 1], yR = yData[i +
                                                                       1];      // points on either side (unless beyond ends)
    if (!extrapolate)                                                         // if beyond ends of array and not extrapolating
    {
        if (x < xL) yR = yL;
        if (x > xR) yL = yR;
    }

    double dydx = (yR - yL) / (xR - xL);                                    // gradient

    return yL + dydx * (x - xL);                                              // linear interpolation
}

// linear interpolation: the sizes of in and out are known
void interpolate1D(const std::vector<double> &in, std::vector<double> &out) {
    size_t in_len = in.size();
    size_t out_len = out.size();

    std::vector<double> copy_in(in_len);
    for (int i = 0; i < in_len; ++i) {
        copy_in[i] = in[i];
    }

    double factor = double(out_len)/double(in_len);

    std::vector<double> ids(in.size(), 0);
    for (int i = 0; i < ids.size(); ++i) {
        ids[i] = ceil(factor * i);
    }

    for (int i = 0; i < out_len; ++i) {
        out[i] = interpolate( ids, copy_in, double(i), true);
    }

}

void meanfilter1D(const std::vector<double> &in, std::vector<double> &out, double factor) {
    size_t  in_len = in.size();
    size_t  out_len = out.size();

    std::vector<double> temp_out(out_len, 0.);

    for (int i = 0; i < in_len; ++i) {
        auto out_i = static_cast<int>(i * factor);
        temp_out[out_i] = in[i];
    }

    // apply mean filter on out
    if(temp_out.size() >=2){
        out[1] = 0.5*(temp_out[0] + temp_out[1]);
        for (int i = 1; i < out_len-1; ++i) {
            out[i] = (temp_out[i-1]+temp_out[i]+temp_out[i+1])/3.;
        }
        out[out_len-1] = 0.5*(temp_out[out_len-2] + temp_out[out_len-1]);
    }else{
        for (int i = 0; i < out_len; ++i) {
            out[i] = temp_out[i];
        }
    }
}

/*
 * Compare with search1D, search1DResize assumes that the phase shift line will get wider or thiner when their phase are different.
 */
double search1DResize(const std::vector<double> &in1, const std::vector<double> &in2) {

    std::vector<double> copy_in1(in1.size());
    for (size_t i = 0; i < in1.size(); ++i) {
        copy_in1[i] = in1[i];
    }
    std::vector<double> copy_in2(in2.size());
    for (size_t i = 0; i < in2.size(); ++i) {
        copy_in2[i] = in2[i];
    }

    double res = 0.002;
    double f = res;
    double min_dis = INFINITY;
    double min_f = f;
    while(f < 10.){
        if (f < 1){
            // resize copy_in2 according to factor 1/f
            auto len_n2 = static_cast<int>(1./ f * double(in2.size()));
            std::vector<double> resized_in2(len_n2);
            meanfilter1D(copy_in2, resized_in2, 1./f);

            // calculated distance between resized_in2 and copy_in1
            double dis = 0.;
            int zero_cnt = 0;
            for (int i = 0; i < in1.size(); ++i) {
                if(resized_in2[i]==0 || copy_in1[i]==0){
                    zero_cnt++;
                }
                dis += fabs(resized_in2[i]-copy_in1[i]);
            }
            if((dis < min_dis) && (zero_cnt < in1.size())){
                min_dis = dis;
                min_f = f;
            }

        }else{
            // resize copy_in1 according to factor f
            auto len_n1 = static_cast<int>(f * double(in1.size()));
            std::vector<double> resized_in1(len_n1);
            meanfilter1D(copy_in1, resized_in1, f);

            // calculated distance between resized_in1 and copy_in2
            double dis = 0.;
            int zero_cnt = 0;
            for (int i = 0; i < in2.size(); ++i) {
                if(resized_in1[i]==0 || copy_in2[i]==0){
                    zero_cnt++;
                }
                dis += fabs(resized_in1[i]-copy_in2[i]);
            }
            if((dis < min_dis) && (zero_cnt < in1.size())){
                min_dis = dis;
                min_f = f;
            }
        }
        f+=res;
    }

    double factor = min_f;

    return factor;
}

void reverseline(std::vector<double> & line){
    auto len = line.size();
    std::vector<double> copy_line(len);
    for (int i = 0; i < len; ++i) {
        copy_line[i] = line[i];
    }

    //reverse copy line and store in line;
    for (int i = 0; i < len; ++i) {
        line[i] = copy_line[len - 1 - i];
    }

}


int medIdxNonZero(const std::vector<double> &arr) {
    std::vector<int> index;
    for (int i = 0; i < arr.size(); ++i) {
        if (arr[i] > 0.0001) {
            index.push_back(i);
        }
    }
    auto len = index.size();
    if (len % 2 == 0) {
        return (index[len / 2 - 1] + index[len / 2]) / 2;
    } else {
        return index[len / 2];
    }
}

#endif //OMNI_SINUSOID_MISC_H
