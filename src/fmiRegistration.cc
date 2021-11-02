#define DEBUG 1

#include "fmiRegistration.hh"

#include <stdio.h>

#define LOC  fprintf(stderr, "[in %s@line %d] ", __FILE__, __LINE__);
#define ERR(...) do{LOC fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n");} while(0)


#include <assert.h>
#include <limits>
#include <iostream>
#include <fstream>
#include <math.h>

// #include "NetworkImageFMIImage.hh"

using namespace jacobs_robotics;
using namespace std;
//using namespace network_image;

#define ROUND(d) ((long) ( (d) + ((d) > 0 ? 0.5 : -0.5)))

#define ERROR_NOISE (1234567.89)

////////////////////////////////////////////////////////////

struct Peak {
    unsigned int x, y;
    FMIReal value;

    inline bool isNear(const unsigned int &xx, const unsigned int &yy, const unsigned int &dist) {
        return ((int) (x) >= (int) (xx) - (int) (dist) && x <= xx + dist && (int) (y) >= (int) (yy) - (int) (dist) &&
                y <= yy + dist);
    }
};

struct Peaks {
    vector<Peak> peaks;

    bool isPeakNear(const unsigned int &x, const unsigned int &y, const unsigned int &dist) {
        vector<Peak>::iterator itr = peaks.begin();
        for (; itr != peaks.end(); itr++) {
            if (itr->isNear(x, y, dist)) return true;
        }
        return false;
    }

    bool findPeakNear(const unsigned int &x,
                      const unsigned int &y,
                      const unsigned int &dist,
                      int &n_idx) {
        vector<Peak>::iterator itr = peaks.begin();
        int cnt = 0;
        for (; itr != peaks.end(); itr++) {
            if (itr->isNear(x, y, dist)) {
                n_idx = cnt;
                return true;
            }
            cnt++;
        }
        return false;
    }
};

std::vector<int> sort_index_by_trans(const std::vector<FMIResult> &v) {

    // 初始化索引向量
    std::vector<int> idx(v.size());
    //使用iota对向量赋0~？的连续值
    iota(idx.begin(), idx.end(), 0);

    // 通过比较v的值对索引idx进行排序
    sort(idx.begin(), idx.end(),
         [&v](size_t i1, size_t i2) {
             return pow(v[i1].transX, 2) + pow(v[i1].transY, 2)
                    > pow(v[i2].transX, 2) + pow(v[i2].transY, 2);
         });
    return idx;
}

std::vector<int> sort_index_by_scales(const std::vector<FMIResult> &v) {

    // 初始化索引向量
    std::vector<int> idx(v.size());
    //使用iota对向量赋0~？的连续值
    iota(idx.begin(), idx.end(), 0);

    // 通过比较v的值对索引idx进行排序
    sort(idx.begin(), idx.end(),
         [&v](size_t i1, size_t i2) {
             return pow(v[i1].scale, 2)
                    > pow(v[i2].scale, 2);
         });
    return idx;
}

/*
 * search along the peaks
 */
std::vector<double> searchAlongPeaks(const FMIReal *const in,
                                     const unsigned int &resolution,
                                     const unsigned int range,
                                     Peaks &peaks,
                                     unsigned int minDistance) {
    std::vector<double> peaksline(resolution / 2, 0.0);
    unsigned int cx = resolution / 2;
    unsigned int cy = resolution / 2;
    Peak maxPeak = peaks.peaks[0];
    // search along the line cross the highest peak and center
    const FMIReal *inP;
    if (maxPeak.x == cx) {
        // k = \inf
        if (maxPeak.y > cy) {
            peaksline[maxPeak.y - cy] = maxPeak.value;
            for (unsigned int i = cy; i < resolution; ++i) {
                inP = in + (i - 1) * resolution + cx - 2;
                for (unsigned int j = cx - 2; j < cx + 2; ++j, inP++) {
                    if (*inP > maxPeak.value * 0.5) {
                        int n_idx;
                        if (peaks.findPeakNear(j + 1, i, minDistance, n_idx)) {
                            if (*inP > peaks.peaks[n_idx].value) {
                                peaksline[peaks.peaks[n_idx].y - cy] = 0.;
                                peaksline[i - cy] = *inP;
                                peaks.peaks[n_idx].x = j + 1;
                                peaks.peaks[n_idx].y = i;
                                peaks.peaks[n_idx].value = *inP;
                            }
                            continue;
                        }
                        peaksline[i - cy] = *inP;
                        Peak other_peak;
                        other_peak.x = j + 1;
                        other_peak.y = i;
                        other_peak.value = *inP;
                        peaks.peaks.push_back(other_peak);
                    }
                }
            }
        } else {
            peaksline[cy - maxPeak.y] = maxPeak.value;
            for (unsigned int i = 0; i < cy; ++i) {
                inP = in + (i - 1) * resolution + cx - 2;
                for (unsigned int j = cx - 2; j < cx + 2; ++j, inP++) {
                    if (*inP > maxPeak.value * 0.5) {
                        int n_idx;
                        if (peaks.findPeakNear(j + 1, i, minDistance, n_idx)) {
                            if (*inP > peaks.peaks[n_idx].value) {
                                peaksline[cy - peaks.peaks[n_idx].y] = 0.;
                                peaksline[cy - i] = *inP;
                                peaks.peaks[n_idx].x = j + 1;
                                peaks.peaks[n_idx].y = i;
                                peaks.peaks[n_idx].value = *inP;
                            }
                            continue;
                        }
                        peaksline[cy - i] = *inP;
                        Peak other_peak;
                        other_peak.x = j + 1;
                        other_peak.y = i;
                        other_peak.value = *inP;
                        peaks.peaks.push_back(other_peak);
                    }
                }
            }
        }
    } else {
        double theta = atan2(int(maxPeak.y - cy), int(maxPeak.x - cx));  // theta \in [-pi, pi]
        cout << "theta = " << theta << endl;
        // handle with interval when theta \in [-pi, 0] to make theta \in [0, 2*pi]
        if (theta < 0) {
            theta += 2 * M_PI;
        }
        if ((theta >= 3. * M_PI / 4. && theta <= 5. * M_PI / 4.)
            || (theta <= M_PI / 4.)
            || (theta >= 7. * M_PI / 4.)) {
            auto h_idx = static_cast<int>(sqrt(pow((maxPeak.y - 1) - cy, 2)
                                               + pow((maxPeak.x - 1) - cx, 2)));
            peaksline[h_idx] = maxPeak.value;
            for (unsigned int j = 0; j < cx; ++j) {
                unsigned int v_low = 0, v_up = 0, u = 0;
                if (theta < M_PI / 2. || theta > 3. * M_PI / 2.) {
                    u = cx + j;
//                v_low = static_cast<unsigned int>(cy + tan(theta - range / 180. * M_PI) * j);
//                v_up = static_cast<unsigned int>(cy + tan(theta + range / 180. * M_PI) * j);
                    v_low = static_cast<unsigned int>(cy + floor(tan(theta) * j) - range);
                    v_up = static_cast<unsigned int>(cy + floor(tan(theta) * j) + range);
                } else {
                    u = cx - j;
//                v_low = static_cast<unsigned int>(cy - tan(theta - range / 180. * M_PI) * j);
//                v_up = static_cast<unsigned int>(cy - tan(theta + range / 180. * M_PI) * j);
                    v_low = static_cast<unsigned int>(cy - floor(tan(theta) * j) - range);
                    v_up = static_cast<unsigned int>(cy - floor(tan(theta) * j) + range);
                }
                for (unsigned int i = v_low; i <= v_up; ++i) {
                    inP = in + (i - 1) * resolution + (u - 1);
                    if (*inP > maxPeak.value * 0.5) {
                        int n_idx;
                        int idx;
                        if (peaks.findPeakNear(u, i, minDistance, n_idx)) {
                            if (*inP > peaks.peaks[n_idx].value) {
                                idx = static_cast<int>(sqrt(pow((peaks.peaks[n_idx].y - 1) - cy, 2)
                                                            + pow((peaks.peaks[n_idx].x - 1) - cx, 2)));
                                peaksline[idx] = 0.;
                                idx = static_cast<int>(sqrt(pow((i - 1) - cy, 2) + pow((u - 1) - cx, 2)));
                                peaksline[idx] = *inP;
                                peaks.peaks[n_idx].x = u;
                                peaks.peaks[n_idx].y = i;
                                peaks.peaks[n_idx].value = *inP;
                            }
                            continue;
                        }
                        idx = static_cast<int>(sqrt(pow((i - 1) - cy, 2) + pow((u - 1) - cx, 2)));
                        peaksline[idx] = *inP;
                        Peak other_peak;
                        other_peak.x = u;
                        other_peak.y = i;
                        other_peak.value = *inP;
                        peaks.peaks.push_back(other_peak);
                    }
                }
            }
        } else {
            auto h_idx = static_cast<int>(sqrt(pow((maxPeak.y - 1) - cy, 2)
                                               + pow((maxPeak.x - 1) - cx, 2)));
            peaksline[h_idx] = maxPeak.value;
            for (unsigned int i = 0; i < cy; ++i) {
                unsigned int u_low = 0, u_up = 0, v = 0;
                if (theta > M_PI / 4. && theta < 3. * M_PI / 4.) {
                    v = cy + i;
                    u_low = static_cast<unsigned int>(cx + floor(i / tan(theta)) - range);
                    u_up = static_cast<unsigned int>(cx + floor(i / tan(theta)) + range);
                } else {
                    v = cy - i;
                    u_low = static_cast<unsigned int>(cx - floor(i / tan(theta)) - range);
                    u_up = static_cast<unsigned int>(cx - floor(i / tan(theta)) + range);
                }
                for (unsigned int j = u_low; j < u_up; ++j) {
                    inP = in + (v - 1) * resolution + (j - 1);
                    if (*inP > maxPeak.value * 0.5) {
                        int n_idx;
                        int idx;
                        if (peaks.findPeakNear(j, v, minDistance, n_idx)) {
                            if (*inP > peaks.peaks[n_idx].value) {
                                idx = static_cast<int>(sqrt(pow((peaks.peaks[n_idx].y - 1) - cy, 2)
                                                            + pow((peaks.peaks[n_idx].x - 1) - cx, 2)));
                                peaksline[idx] = 0.;
                                idx = static_cast<int>(sqrt(pow((v - 1) - cy, 2) + pow((j - 1) - cx, 2)));
                                peaksline[idx] = *inP;
                                peaks.peaks[n_idx].x = j;
                                peaks.peaks[n_idx].y = v;
                                peaks.peaks[n_idx].value = *inP;
                            }
                            continue;
                        }
                        idx = static_cast<int>(sqrt(pow((v - 1) - cy, 2) + pow((j - 1) - cx, 2)));
                        peaksline[idx] = *inP;
                        Peak other_peak;
                        other_peak.x = j;
                        other_peak.y = v;
                        other_peak.value = *inP;
                        peaks.peaks.push_back(other_peak);
                    }
                }
            }
        }
    }
    return peaksline;
}

/*
 * range: search [range] pixel shifts around the peak
 */
void findMultiplePeaks(const FMIReal *const in,
                       const unsigned int &resolution,
                       const unsigned int range, Peaks &peaks, unsigned int minDistance) {
//    cout << "in(258, 227): " << *(in + (258-1)*resolution + 227-1) << endl;
    unsigned int cx = resolution / 2;
    unsigned int cy = resolution / 2;
    Peak maxPeak = peaks.peaks[0];
    // search along the line cross the highest peak and center
    const FMIReal *inP;
    if (maxPeak.x == cx) {
        // k = \inf
        if (maxPeak.y > cy) {
            for (unsigned int i = cy; i < resolution; ++i) {
                inP = in + (i - 1) * resolution + cx - 2;
                for (unsigned int j = cx - 2; j < cx + 2; ++j, inP++) {
                    if (*inP > maxPeak.value * 0.5) {
                        int n_idx;
                        if (peaks.findPeakNear(j + 1, i, minDistance, n_idx)) {
                            if (*inP > peaks.peaks[n_idx].value) {
                                peaks.peaks[n_idx].x = j + 1;
                                peaks.peaks[n_idx].y = i;
                                peaks.peaks[n_idx].value = *inP;
                            }
                            continue;
                        }
                        Peak other_peak;
                        other_peak.x = j + 1;
                        other_peak.y = i;
                        other_peak.value = *inP;
                        peaks.peaks.push_back(other_peak);
                    }
                }
            }
        } else {
            for (unsigned int i = 0; i < cy; ++i) {
                inP = in + (i - 1) * resolution + cx - 2;
                for (unsigned int j = cx - 2; j < cx + 2; ++j, inP++) {
                    if (*inP > maxPeak.value * 0.5) {
                        int n_idx;
                        if (peaks.findPeakNear(j + 1, i, minDistance, n_idx)) {
                            if (*inP > peaks.peaks[n_idx].value) {
                                peaks.peaks[n_idx].x = j + 1;
                                peaks.peaks[n_idx].y = i;
                                peaks.peaks[n_idx].value = *inP;
                            }
                            continue;
                        }
                        Peak other_peak;
                        other_peak.x = j + 1;
                        other_peak.y = i;
                        other_peak.value = *inP;
                        peaks.peaks.push_back(other_peak);
                    }
                }
            }
        }
    } else {
        double theta = atan2(int(maxPeak.y - cy), int(maxPeak.x - cx));  // theta \in [-pi, pi]
        cout << "theta = " << theta << endl;
        // handle with interval when theta \in [-pi, 0] to make theta \in [0, 2*pi]
        if (theta < 0) {
            theta += 2 * M_PI;
        }
        if ((theta >= 3. * M_PI / 4. && theta <= 5. * M_PI / 4.)
            || (theta <= M_PI / 4.)
            || (theta >= 7. * M_PI / 4.)) {
            for (unsigned int j = 0; j < cx; ++j) {
                unsigned int v_low = 0, v_up = 0, u = 0;
                if (theta < M_PI / 2. || theta > 3. * M_PI / 2.) {
                    u = cx + j;
//                v_low = static_cast<unsigned int>(cy + tan(theta - range / 180. * M_PI) * j);
//                v_up = static_cast<unsigned int>(cy + tan(theta + range / 180. * M_PI) * j);
                    v_low = static_cast<unsigned int>(cy + floor(tan(theta) * j) - range);
                    v_up = static_cast<unsigned int>(cy + floor(tan(theta) * j) + range);
                } else {
                    u = cx - j;
//                v_low = static_cast<unsigned int>(cy - tan(theta - range / 180. * M_PI) * j);
//                v_up = static_cast<unsigned int>(cy - tan(theta + range / 180. * M_PI) * j);
                    v_low = static_cast<unsigned int>(cy - floor(tan(theta) * j) - range);
                    v_up = static_cast<unsigned int>(cy - floor(tan(theta) * j) + range);
                }
                if (v_low < 1) {
                    v_low = 1;
                }
                if (v_low > resolution + 1) {
                    v_low = resolution + 1;
                }
                if (v_up < 1) {
                    v_up = 1;
                }
                if (v_up > resolution + 1) {
                    v_up = resolution + 1;
                }
                for (unsigned int i = v_low; i <= v_up; ++i) {
                    inP = in + (i - 1) * resolution + (u - 1);
                    if (*inP > maxPeak.value * 0.5) {
                        int n_idx;
                        if (peaks.findPeakNear(u, i, minDistance, n_idx)) {
                            if (*inP > peaks.peaks[n_idx].value) {
                                peaks.peaks[n_idx].x = u;
                                peaks.peaks[n_idx].y = i;
                                peaks.peaks[n_idx].value = *inP;
                            }
                            continue;
                        }
                        Peak other_peak;
                        other_peak.x = u;
                        other_peak.y = i;
                        other_peak.value = *inP;
                        peaks.peaks.push_back(other_peak);
                    }
                }
            }
        } else {
            for (unsigned int i = 0; i < cy; ++i) {
                unsigned int u_low = 0, u_up = 0, v = 0;
                if (theta > M_PI / 4. && theta < 3. * M_PI / 4.) {
                    v = cy + i;
                    u_low = static_cast<unsigned int>(cx + floor(i / tan(theta)) - range);
                    u_up = static_cast<unsigned int>(cx + floor(i / tan(theta)) + range);
                } else {
                    v = cy - i;
                    u_low = static_cast<unsigned int>(cx - floor(i / tan(theta)) - range);
                    u_up = static_cast<unsigned int>(cx - floor(i / tan(theta)) + range);
                }
                if (u_low < 1) {
                    u_low = 1;
                }
                if (u_low > resolution + 1) {
                    u_low = resolution + 1;
                }
                if (u_up < 1) {
                    u_up = 1;
                }
                if (u_up > resolution + 1) {
                    u_up = resolution + 1;
                }
                for (unsigned int j = u_low; j < u_up; ++j) {
                    inP = in + (v - 1) * resolution + (j - 1);
                    if (*inP > maxPeak.value * 0.5) {
                        int n_idx;
                        if (peaks.findPeakNear(j, v, minDistance, n_idx)) {
                            if (*inP > peaks.peaks[n_idx].value) {
                                peaks.peaks[n_idx].x = j;
                                peaks.peaks[n_idx].y = v;
                                peaks.peaks[n_idx].value = *inP;
                            }
                            continue;
                        }
                        Peak other_peak;
                        other_peak.x = j;
                        other_peak.y = v;
                        other_peak.value = *inP;
                        peaks.peaks.push_back(other_peak);
                    }
                }
            }
        }
    }
}

void findColPeaks(const FMIReal *const in,
                  const unsigned int &resolution,
                  const unsigned int range,
                  Peaks &peaks, vector<double> &peaksline) {
    Peak maxPeak = peaks.peaks[0];
    peaks.peaks.clear();
    peaksline.clear();
    const FMIReal *inP;

    int start_idx = 1;
    int end_idx = resolution - 1;
    if (maxPeak.y <= resolution / 2) {
        end_idx = resolution / 2;
        for (int i = end_idx; i >= start_idx; --i) { // row index
            vector<FMIReal> values;
            FMIReal max_value = 0;
            for (int j = std::max(static_cast<int>(maxPeak.x - range), 0);
                 j < std::min(maxPeak.x + range, resolution); ++j) { // column near the highest
                inP = in + (i - 1) * resolution + (j - 1);
                if (*inP > maxPeak.value * 0.55) { // high value
                    values.push_back(*inP);
                }
            }
            if (!values.empty()) {
                Peak tmp_peak;
                tmp_peak.x = maxPeak.x;
                tmp_peak.y = static_cast<unsigned int>(i);
                max_value = *max_element(values.begin(), values.end());
                tmp_peak.value = max_value;
                peaks.peaks.push_back(tmp_peak);
            }
            peaksline.push_back(max_value);
        }

    } else {
        start_idx = resolution / 2 + 1;
        for (int i = start_idx; i <= end_idx; ++i) { // row index
            vector<FMIReal> values;
            FMIReal max_value = 0;
            for (int j = std::max(static_cast<int>(maxPeak.x - range), 0);
                 j < std::min(maxPeak.x + range, resolution); ++j) { // column near the highest
                inP = in + (i - 1) * resolution + (j - 1);
                if (*inP > maxPeak.value * 0.55) { // high value
                    values.push_back(*inP);
                }
            }
            if (!values.empty()) {
                Peak tmp_peak;
                tmp_peak.x = maxPeak.x;
                tmp_peak.y = static_cast<unsigned int>(i);
                max_value = *max_element(values.begin(), values.end());
                tmp_peak.value = max_value;
                peaks.peaks.push_back(tmp_peak);
            }
            peaksline.push_back(max_value);
        }
    }
}

void findOtherPeaks(const FMIReal *const in, const unsigned int &resolution, Peaks &peaks, unsigned int minDistance) {
    Peak maxPeak;
    maxPeak.value = 0;
    maxPeak.x = 0;
    maxPeak.y = 0;
    const FMIReal *inP = in;
    // search the whole matrix
    for (unsigned int y = 0; y < resolution; y++) {
        for (unsigned int x = 0; x < resolution; x++) {
            const FMIReal val = *inP++;
            if (val > maxPeak.value) {
                // minium distance to other peaks is 3 pixel
                if (peaks.isPeakNear(x, y, minDistance)) continue;
                maxPeak.value = val;
                maxPeak.x = x;
                maxPeak.y = y;
            }
        }
    }
    peaks.peaks.push_back(maxPeak);
}

// finds numberOfPeaks other peaks in the matrix in with resolution. The result is saved in peaks.
void
findOtherPeaks(const FMIReal *const in, const unsigned int &resolution, Peaks &peaks, const unsigned int &numberOfPeaks,
               const unsigned int &minDistance) {
    //find as many peaks as specified in numberOfPeaks
    for (unsigned int i = 0; i < numberOfPeaks; i++) {
        findOtherPeaks(in, resolution, peaks, minDistance);
    }
}

void
sumPeakEnergies(const FMIReal *const in, const unsigned int &resolution, Peaks &peaks, const unsigned int &halfSize) {
    vector<Peak>::iterator itr = peaks.peaks.begin();
    for (; itr != peaks.peaks.end(); itr++) {
        itr->value = FMIRegistration::calculateSum(itr->x, itr->y, in, resolution, halfSize);
    }
}


////////////////////////////////////////////////////////////

FMIRegistration::FMIRegistration(const unsigned int &_resolution,
                                 const unsigned int &_polLogResolution, const FMIReal &_minFMISignalToNoise,
                                 const FMIReal &_minFMIsignalFactorOver2nd) :
        resolution(_resolution), squaredResolution(resolution * resolution),
        polarLogResolution(_polLogResolution), squaredPolLogRes(
        polarLogResolution * polarLogResolution), fftMath(
        resolution), fftMathPolar(polarLogResolution),
        minFMISignalToNoise(_minFMISignalToNoise),
        minFMIsignalFactorOver2nd(_minFMIsignalFactorOver2nd),
        spectralWindow2D(0), polLogInterpolationMatrices(0),
        polLogInterpolationCoordinates(0), gaussianWindow1D(0),
        tempMatrix1(new FMIReal[(resolution + 1) * (resolution + 1)]),
        tempMatrix2(new FMIReal[(resolution + 1) * (resolution + 1)]),
        tempMatrix3(new FMIReal[(resolution + 1) * (resolution + 1)]),
//          tempScaleMat(new FMIReal[resolution * resolution * 12 * 100]),
        tempComplexMatrix1(new ComplexFMIReal[squaredResolution]),
        tempComplexMatrix2(new ComplexFMIReal[squaredResolution]),
        tempCompressedMatrix(new FMICompressedReal[resolution * resolution / 2 + resolution]) {
    angleResolution = 1;
    if (resolution % 2) {
        ERR(" Only even resolutions supported!");
        assert(0);
    }
    if (resolution < polarLogResolution) {
        ERR(" The polarLogResolution (%d) has to be smaller or equal to the image resolution (%d)!", polarLogResolution,
            resolution);
        assert(0);
    }
    use2ndRotationPeak = false;
    dumpDebugInfo = true;
    init();
}

FMIRegistration::FMIRegistration(const unsigned int &_resolution, const unsigned int &_polLogResolution,
                                 const unsigned int &_angleReslution, const FMIReal &_minFMISignalToNoise,
                                 const FMIReal &_minFMIsignalFactorOver2nd) :
        resolution(_resolution), squaredResolution(resolution * resolution),
        polarLogResolution(_polLogResolution), squaredPolLogRes(
        polarLogResolution * polarLogResolution), angleResolution(_angleReslution), fftMath(
        resolution), fftMathPolar(polarLogResolution),
        minFMISignalToNoise(_minFMISignalToNoise),
        minFMIsignalFactorOver2nd(_minFMIsignalFactorOver2nd),
        spectralWindow2D(0), polLogInterpolationMatrices(0), polLogInterpolationCoordinates(0), gaussianWindow1D(0),
        tempMatrix1(new FMIReal[(resolution + 1) * (resolution + 1)]),
        tempMatrix2(new FMIReal[(resolution + 1) * (resolution + 1)]),
        tempMatrix3(new FMIReal[(resolution + 1) * (resolution + 1)]),
//          tempScaleMat(new FMIReal[resolution * resolution * 12 * 100]),
        tempComplexMatrix1(new ComplexFMIReal[squaredResolution]),
        tempComplexMatrix2(new ComplexFMIReal[squaredResolution]),
        tempCompressedMatrix(new FMICompressedReal[resolution * resolution / 2 + resolution]) {
    if (resolution % 2) {
        ERR(" Only even resolutions supported!");
        assert(0);
    }
    if (resolution < polarLogResolution) {
        ERR(" The polarLogResolution (%d) has to be smaller or equal to the image resolution (%d)!", polarLogResolution,
            resolution);
        assert(0);
    }
    use2ndRotationPeak = false;
    dumpDebugInfo = true;
    init();
}

FMIRegistration::~FMIRegistration() {
    delete[] tempMatrix1;
    delete[] tempMatrix2;
    delete[] tempMatrix3;
    delete[] tempComplexMatrix1;
    delete[] tempComplexMatrix2;
//	delete[] tempScaleMat;
    delete[] spectralWindow2D;
    delete[] polLogInterpolationMatrices;
    delete[] polLogInterpolationCoordinates;
    delete[] tempCompressedMatrix;
}

bool FMIRegistration::registerMultiTrans(FMIImage *first,
                                         FMIImage *second,
                                         std::vector<FMIResult> &results,
                                         bool over180deg) {
    if (ONLY_TRANS) {
        // check the input data for sanity
        if (first == 0 || second == 0) {
            ERR(" First (%p) or second (%p) image pointer is NULL!", first, second);
            return false;
        }
        if (first->img == 0 || second->img == 0) {
            ERR(" First (%p) or second (%p) image is NULL!", first->img, second->img);
            return false;
        }
        if (resolution != first->resolution || resolution != second->resolution) {
            ERR(" FMI resolution (%d) differs from first (%d) and/ or second (%d) resolution!", resolution,
                first->resolution, second->resolution);
            return false;
        }

        FMIResult result;
        results.push_back(result);
        // make sure we have the phase information
        prepareFMIImage(first, false);
        prepareFMIImage(second, false);

        const unsigned int resPlusOne = resolution + 1;

        // now do the actual matching...
        phaseOnlyMatching(first->phase, second->phase, fftMath.tmpMatrix,
                          resolution, false);
        fftMath.inverseFft2D();
        fftMath.spacialMagnitudeAndFFTShift(fftMath.tmpMatrix, tempMatrix1);
        filter(tempMatrix1, tempMatrix2, resolution);

        if (dumpDebugInfo) {
            dumpValues("afterFilter.txt", tempMatrix2, polarLogResolution + 1, 1); // tempMatrix1 is without filtering
            saveFMIRealImage("result-filtered-trans.png", tempMatrix2, polarLogResolution + 1, 1, 10000.);
        }

        FMIReal maxValUsed;
        unsigned int maxCoordUsed;
        const FMIReal *tmpMatrixUsed = 0;

        FMIReal maxValOrig;
        unsigned int maxCoordOrig;

        findMaxByAngle(tempMatrix2, resPlusOne, maxValOrig, maxCoordOrig);
//        findMax(tempMatrix2, resPlusOne * resPlusOne, maxValOrig, maxCoordOrig);

        // now the stuff for the 180deg rotation...
        FMIReal maxValRotated = 0.;
        unsigned int maxCoordRotated = 0;

        maxValUsed = maxValOrig;
        maxCoordUsed = maxCoordOrig;
        tmpMatrixUsed = tempMatrix2;

        FMIReal posX = (double) (maxCoordUsed % resPlusOne) + 1.0;
        FMIReal posY = ceil((double) (maxCoordUsed) / (double) (resPlusOne));


        FMIReal maxX = posX;
        FMIReal maxY = posY;
        // peaks stuff for the translation...
        Peaks peaks;
        Peak max;
        max.value = maxValOrig;
        max.x = posX;
        max.y = posY;
        peaks.peaks.push_back(max);

        calculateUncertainty(static_cast<int>(posX),
                             static_cast<int>(posY), tmpMatrixUsed, resPlusOne, results[0].signalTrans,
                             results[0].noiseTrans, results[0].covarianceMatrixTrans);
        // interpolate parameters within 3x3 area of determined dirac pulse
        interpolatePeak(tmpMatrixUsed, resPlusOne, posX, posY);
        results[0].transX = posX;
        results[0].transY = posY;

        // find other multiple peaks
        if (sqrt(pow(maxX - resPlusOne / 2., 2) + pow(maxX - resPlusOne / 2., 2)) >= 5.) {
            findMultiplePeaks(tmpMatrixUsed, polarLogResolution + 1, 5, peaks, 3);
            for (auto &peak : peaks.peaks) {
                if (fabs(peak.x - maxX) < 1e-6
                    && fabs(peak.y - maxY) < 1e-6) {
                    continue;
                }
                FMIReal peakX = peak.x;
                FMIReal peakY = peak.y;
                FMIResult tmpResult;
                calculateUncertainty(static_cast<int>(peakX),
                                     static_cast<int>(peakY), tmpMatrixUsed, resPlusOne, tmpResult.signalTrans,
                                     tmpResult.noiseTrans, tmpResult.covarianceMatrixTrans);
                interpolatePeak(tmpMatrixUsed, resPlusOne, peakX, peakY);
                tmpResult.transX = peakX;
                tmpResult.transY = peakY;
                results.push_back(tmpResult);
            }
        }
        return true;
    } else {
        // check the input data for sanity
        if (first == 0 || second == 0) {
            ERR(" First (%p) or second (%p) image pointer is NULL!", first, second);
            return false;
        }
        if (first->img == 0 || second->img == 0) {
            ERR(" First (%p) or second (%p) image is NULL!", first->img, second->img);
            return false;
        }
        if (resolution != first->resolution || resolution != second->resolution) {
            ERR(" FMI resolution (%d) differs from first (%d) and/ or second (%d) resolution!", resolution,
                first->resolution, second->resolution);
            return false;
        }

        // Prepare the images - we just need the polar logarithmic part for now
        prepareFMIImage(first, true);
        prepareFMIImage(second, true);

        // determine rotation/scaling
        //   - phase only matching
        phaseOnlyMatching(second->polarLogPhase, first->polarLogPhase,
                          fftMathPolar.tmpMatrix, polarLogResolution, false);

        fftMathPolar.inverseFft2D();
        // spacial magnitude and fft shift
        fftMathPolar.spacialMagnitudeAndFFTShift(fftMathPolar.tmpMatrix, tempMatrix1);

        // tempMatrix has afterwards the resolution (polarLogResolution + 1)   !
        filter(tempMatrix1, tempMatrix2, polarLogResolution);

        if (dumpDebugInfo) {
            saveFMIRealImage("result-filtered-rot-scale.png", tempMatrix1, polarLogResolution, 1);
            dumpValues("values-rot-scale.txt", tempMatrix2, polarLogResolution + 1, 1);
        }

        bool success = findDiracPulsesScaleRotation(tempMatrix2, results);
        if ((!success) || results.empty()) {
            ERR("error finding scale and rotation...");
            return false;
        }

        if ((results[0].signalFMI / results[0].noiseFMI) < minFMISignalToNoise) {
            // minium signal to noise ratio not reached - we don't need to calculate further...
            cout << " too low signal to noise ratio!" << endl;
            return false;
        }

        // now we have the scale and rotation
        // we find out the translation now

        // first we scale and rotate the second image...
        scaleAndRotate(second->img, (FMIImageReal *) tempMatrix1, results[0].scale,
                       results[0].rotationRad);

        //    network_image::saveFMIRealImage("result-rotated.png", tempMatrix1, resolution);

        // now we prepare the rotated image...
        removeConstantOffsetApplySpectralWindow((FMIImageReal *) tempMatrix1, tempMatrix2);
        fftMath.fft2D(tempMatrix2, tempComplexMatrix1);
        saveAngles(tempComplexMatrix1, tempCompressedMatrix, resolution);

        // make sure we have the phase information
        prepareFMIImage(first, false);

        const unsigned int resPlusOne = resolution + 1;

        // now do the actual matching...
        phaseOnlyMatching(first->phase, tempCompressedMatrix, fftMath.tmpMatrix,
                          resolution, false);
        fftMath.inverseFft2D();
        fftMath.spacialMagnitudeAndFFTShift(fftMath.tmpMatrix, tempMatrix1);
        filter(tempMatrix1, tempMatrix2, resolution);

        if (dumpDebugInfo) {
            dumpValues("afterFilter.txt", tempMatrix2, polarLogResolution + 1, 1); // tempMatrix1 is without filtering
            saveFMIRealImage("result-filtered-trans.png", tempMatrix2, polarLogResolution + 1, 1, 10000.);
        }

        FMIReal maxValUsed;
        unsigned int maxCoordUsed;
        const FMIReal *tmpMatrixUsed = 0;

        FMIReal maxValOrig;
        unsigned int maxCoordOrig;

//        findMax(tempMatrix2, resPlusOne * resPlusOne, maxValOrig, maxCoordOrig);
//        findMaxByAngle(tempMatrix2, resPlusOne, maxValOrig, maxCoordOrig);
        findMaxRay(tempMatrix2, resPlusOne * resPlusOne, maxValOrig, maxCoordOrig);
        cout << maxCoordOrig << endl;

        // now the stuff for the 180deg rotation...
        FMIReal maxValRotated = 0.;
        unsigned int maxCoordRotated = 0;

        if (over180deg) {
            // now do the actual matching...
            phaseOnlyMatching(first->phase, tempCompressedMatrix, fftMath.tmpMatrix,
                              resolution, true);
            fftMath.inverseFft2D();
            fftMath.spacialMagnitudeAndFFTShift(fftMath.tmpMatrix, tempMatrix1);
            filter(tempMatrix1, tempMatrix3, resolution);

            findMax(tempMatrix3, resPlusOne * resPlusOne, maxValRotated, maxCoordRotated);
        }

        // choose the correct rotation...
        if (over180deg) {
            if (maxValOrig >= maxValRotated) {
                maxValUsed = maxValOrig;
                maxCoordUsed = maxCoordOrig;
                tmpMatrixUsed = tempMatrix2;
//             cout<<" using less than 180 deg!"<<endl;
            } else {
                maxValUsed = maxValRotated;
                maxCoordUsed = maxCoordRotated;
                tmpMatrixUsed = tempMatrix3;
//             cout<<" using over 180 deg!"<<endl;
                results[0].rotationRad += M_PI;
            }
        } else {
            maxValUsed = maxValOrig;
            maxCoordUsed = maxCoordOrig;
            tmpMatrixUsed = tempMatrix2;
//            tmpMatrixUsed = tempMatrix1;
        }

        FMIReal posX = (double) (maxCoordUsed % resPlusOne) + 1.0;
        FMIReal posY = ceil((double) (maxCoordUsed) / (double) (resPlusOne));

        FMIReal maxX = posX;
        FMIReal maxY = posY;

        // peaks stuff for the translation...
        Peaks peaks;
        Peak max;
        max.value = maxValOrig;
        max.x = posX;
        max.y = posY;
        peaks.peaks.push_back(max);

        calculateUncertainty(static_cast<int>(posX),
                             static_cast<int>(posY), tmpMatrixUsed, resPlusOne, results[0].signalTrans,
                             results[0].noiseTrans, results[0].covarianceMatrixTrans);

        // interpolate parameters within 3x3 area of determined dirac pulse
        interpolatePeak(tmpMatrixUsed, resPlusOne, posX, posY);
        results[0].transX = posX;
        results[0].transY = posY;

        if (sqrt(pow(posX, 2) + pow(posY, 2)) >= 5.) {
            findMultiplePeaks(tmpMatrixUsed, polarLogResolution + 1, 5, peaks, 3);
            for (auto &peak : peaks.peaks) {
                if (fabs(peak.x - maxX) < 1e-6
                    && fabs(peak.y - maxY) < 1e-6) {
                    continue;
                }
                FMIReal peakX = peak.x;
                FMIReal peakY = peak.y;
                FMIResult tmpResult;
                calculateUncertainty(static_cast<int>(peakX),
                                     static_cast<int>(peakY), tmpMatrixUsed, resPlusOne, tmpResult.signalTrans,
                                     tmpResult.noiseTrans, tmpResult.covarianceMatrixTrans);
                interpolatePeak(tmpMatrixUsed, resPlusOne, peakX, peakY);
                tmpResult.scale = results[0].scale;
                tmpResult.rotationRad = results[0].rotationRad;
                tmpResult.transX = peakX;
                tmpResult.transY = peakY;
                results.push_back(tmpResult);
            }
        }
        return true;
    }
}

bool FMIRegistration::registerMultiTransMultiScale(FMIImage *first,
                                                   FMIImage *second,
                                                   std::vector<FMIResult> &results,
                                                   std::vector<double> &scalesline,
                                                   std::vector<double> &transline,
                                                   bool over180deg) {
    // check the input data for sanity
    if (first == 0 || second == 0) {
        ERR(" First (%p) or second (%p) image pointer is NULL!", first, second);
        return false;
    }
    if (first->img == 0 || second->img == 0) {
        ERR(" First (%p) or second (%p) image is NULL!", first->img, second->img);
        return false;
    }
    if (resolution != first->resolution || resolution != second->resolution) {
        ERR(" FMI resolution (%d) differs from first (%d) and/ or second (%d) resolution!", resolution,
            first->resolution, second->resolution);
        return false;
    }

    // Prepare the images - we just need the polar logarithmic part for now
    prepareFMIImage(first, true);
    prepareFMIImage(second, true);

    // determine rotation/scaling
    //   - phase only matching
    phaseOnlyMatching(second->polarLogPhase, first->polarLogPhase,
                      fftMathPolar.tmpMatrix, polarLogResolution, false);

    fftMathPolar.inverseFft2D();
    // spacial magnitude and fft shift
    fftMathPolar.spacialMagnitudeAndFFTShift(fftMathPolar.tmpMatrix, tempMatrix1);

    // tempMatrix has afterwards the resolution (polarLogResolution + 1)   !
    filter(tempMatrix1, tempMatrix2, polarLogResolution);

    if (dumpDebugInfo) {
        saveFMIRealImage("result-filtered-rot-scale.png", tempMatrix1, polarLogResolution, 1);
        dumpValues("values-rot-scale.txt", tempMatrix2, polarLogResolution + 1, 1);
    }

    bool success = findDiracPulsesMultiScaleRotation(tempMatrix2, results, scalesline);
    if ((!success) || results.empty()) {
        ERR("error finding scale and rotation...");
        return false;
    }

//    if ((results[0].signalFMI / results[0].noiseFMI) < minFMISignalToNoise) {
//        // minium signal to noise ratio not reached - we don't need to calculate further...
//        cout << " too low signal to noise ratio!" << endl;
//        return false;
//    }

    std::vector<std::vector<double>> trans_rays(results.size());
//    std::vector<FMIResult> trans_results(results.size());
    cout << "results.size() = " << results.size() << endl;
    for (int i = 0; i < results.size(); ++i) {
        // now we have the multiple scales and rotation
        // we find out the translation now

        // first we scale and rotate the second image...
//        results[i].scale = 0.77993;
//        results[i].rotationRad = 0.0;
        scaleAndRotate(second->img, (FMIImageReal *) tempMatrix1, results[i].scale,
                       results[i].rotationRad);

        // now we prepare the rotated image...
        removeConstantOffsetApplySpectralWindow((FMIImageReal *) tempMatrix1, tempMatrix2);
        fftMath.fft2D(tempMatrix2, tempComplexMatrix1);
        saveAngles(tempComplexMatrix1, tempCompressedMatrix, resolution);

        // make sure we have the phase information
        prepareFMIImage(first, false);

        const unsigned int resPlusOne = resolution + 1;

        // now do the actual matching...
        phaseOnlyMatching(first->phase, tempCompressedMatrix, fftMath.tmpMatrix,
                          resolution, false);
        fftMath.inverseFft2D();
        fftMath.spacialMagnitudeAndFFTShift(fftMath.tmpMatrix, tempMatrix1);
        filter(tempMatrix1, tempMatrix2, resolution);

        if (dumpDebugInfo) {
            dumpValues("afterFilter.txt", tempMatrix2, polarLogResolution + 1, 1); // tempMatrix1 is without filtering
            saveFMIRealImage("result-filtered-trans.png", tempMatrix2, polarLogResolution + 1, 1, 10000.);
        }

        FMIReal maxValUsed;
        unsigned int maxCoordUsed;
        const FMIReal *tmpMatrixUsed = 0;

        FMIReal maxValOrig;
        unsigned int maxCoordOrig;

//        findMax(tempMatrix2, resPlusOne * resPlusOne, maxValOrig, maxCoordOrig);
//        findMaxByAngle(tempMatrix2, resPlusOne, maxValOrig, maxCoordOrig);
//        findMaxRay(tempMatrix2, resPlusOne * resPlusOne, maxValOrig, maxCoordOrig);
        std::vector<double> trans_ray = findMaxRayAndSampling(tempMatrix2,
                                                              resPlusOne * resPlusOne,maxValOrig,
                                                              maxCoordOrig);

        //cout << "result[0].rotationRad: " << results[0].rotationRad << endl;

        for (int j = 0; j < trans_ray.size(); ++j) {
            //cout << trans_ray[j] << " ";
            trans_ray[j] *= results[i].scaleEnergy / results[0].scaleEnergy;
        }
        trans_rays[i] = trans_ray;



        // now the stuff for the 180deg rotation...
        FMIReal maxValRotated = 0.;
        unsigned int maxCoordRotated = 0;

        if (over180deg) {
            // now do the actual matching...
            phaseOnlyMatching(first->phase, tempCompressedMatrix, fftMath.tmpMatrix,
                              resolution, true);
            fftMath.inverseFft2D();
            fftMath.spacialMagnitudeAndFFTShift(fftMath.tmpMatrix, tempMatrix1);
            filter(tempMatrix1, tempMatrix3, resolution);

            findMax(tempMatrix3, resPlusOne * resPlusOne, maxValRotated, maxCoordRotated);
        }

        // choose the correct rotation...
        if (over180deg) {
            if (maxValOrig >= maxValRotated) {
                maxValUsed = maxValOrig;
                maxCoordUsed = maxCoordOrig;
                tmpMatrixUsed = tempMatrix2;
            } else {
                maxValUsed = maxValRotated;
                maxCoordUsed = maxCoordRotated;
                tmpMatrixUsed = tempMatrix3;
                results[i].rotationRad += M_PI;
            }
        } else {
            maxValUsed = maxValOrig;
            maxCoordUsed = maxCoordOrig;
            tmpMatrixUsed = tempMatrix2;
        }

        FMIReal posX = (double) (maxCoordUsed % resPlusOne) + 1.0;
        FMIReal posY = ceil((double) (maxCoordUsed) / (double) (resPlusOne));

        FMIReal maxX = posX;
        FMIReal maxY = posY;

        // peaks stuff for the translation...
        Peaks peaks;
        Peak max;
        max.value = maxValOrig;
        max.x = posX;
        max.y = posY;
        peaks.peaks.push_back(max);

        calculateUncertainty(static_cast<int>(posX),
                             static_cast<int>(posY), tmpMatrixUsed, resPlusOne, results[i].signalTrans,
                             results[i].noiseTrans, results[i].covarianceMatrixTrans);

        // interpolate parameters within 3x3 area of determined dirac pulse
        interpolatePeak(tmpMatrixUsed, resPlusOne, posX, posY);
        results[i].transX = posX;
        results[i].transY = posY;
//        FMIResult trans_tmp;
//        trans_tmp.scale = results[i].scale;
//        trans_tmp.rotationRad = results[i].rotationRad;
//        trans_tmp.transX = posX;
//        trans_tmp.transY = posY;
//        trans_results[i] = trans_tmp;
    }

    /*
     * 1. Combined all the trans_rays
     * 2. Rescaling (out of this function)
     */
    transline.clear();
    for (int j = 0; j < trans_rays[0].size(); ++j) {
        vector<double> trans_values;
        for (int i = 1; i < trans_rays.size(); ++i) {
            trans_values.push_back(trans_rays[i][j]);
        }
        // use max
        transline.push_back(*max_element(trans_values.begin(), trans_values.end()));
    }


    return true;
}

bool FMIRegistration::registerMultiTransMultiScale_andMaxAngle(FMIImage *first,
                                                   FMIImage *second,
                                                   std::vector<FMIResult> &results,
                                                   std::vector<double> &scalesline,
                                                   std::vector<double> &transline,
                                                   double &MaxAngle,
                                                   bool over180deg) {
    // check the input data for sanity
    if (first == 0 || second == 0) {
        ERR(" First (%p) or second (%p) image pointer is NULL!", first, second);
        return false;
    }
    if (first->img == 0 || second->img == 0) {
        ERR(" First (%p) or second (%p) image is NULL!", first->img, second->img);
        return false;
    }
    if (resolution != first->resolution || resolution != second->resolution) {
        ERR(" FMI resolution (%d) differs from first (%d) and/ or second (%d) resolution!", resolution,
            first->resolution, second->resolution);
        return false;
    }

    // Prepare the images - we just need the polar logarithmic part for now
    prepareFMIImage(first, true);
    prepareFMIImage(second, true);

    // determine rotation/scaling
    //   - phase only matching
    phaseOnlyMatching(second->polarLogPhase, first->polarLogPhase,
                      fftMathPolar.tmpMatrix, polarLogResolution, false);

    fftMathPolar.inverseFft2D();
    // spacial magnitude and fft shift
    fftMathPolar.spacialMagnitudeAndFFTShift(fftMathPolar.tmpMatrix, tempMatrix1);

    // tempMatrix has afterwards the resolution (polarLogResolution + 1)   !
    filter(tempMatrix1, tempMatrix2, polarLogResolution);

    if (dumpDebugInfo) {
        saveFMIRealImage("./result/22_23/original_have/result-filtered-rot-scale.png", tempMatrix1, polarLogResolution, 1);
        dumpValues("./result/22_23/original_have/values-rot-scale.txt", tempMatrix2, polarLogResolution + 1, 1);
    }

    bool success = findDiracPulsesMultiScaleRotation(tempMatrix2, results, scalesline);
    if ((!success) || results.empty()) {
        ERR("error finding scale and rotation...");
        return false;
    }

    std::vector<std::vector<double>> trans_rays(results.size());

    cout << "results.size() = " << results.size() << endl;
    for (int i = 0; i < results.size(); ++i) {

        scaleAndRotate(second->img, (FMIImageReal *) tempMatrix1, results[i].scale,
                       results[i].rotationRad);

        // now we prepare the rotated image...
        removeConstantOffsetApplySpectralWindow((FMIImageReal *) tempMatrix1, tempMatrix2);
        fftMath.fft2D(tempMatrix2, tempComplexMatrix1);
        saveAngles(tempComplexMatrix1, tempCompressedMatrix, resolution);

        // make sure we have the phase information
        prepareFMIImage(first, false);

        const unsigned int resPlusOne = resolution + 1;

        // now do the actual matching...
        phaseOnlyMatching(first->phase, tempCompressedMatrix, fftMath.tmpMatrix,
                          resolution, false);
        fftMath.inverseFft2D();
        fftMath.spacialMagnitudeAndFFTShift(fftMath.tmpMatrix, tempMatrix1);
        filter(tempMatrix1, tempMatrix2, resolution);

        if (dumpDebugInfo) {
            dumpValues("./result/22_23/original_have/afterFilter.txt", tempMatrix2, polarLogResolution + 1, 1); // tempMatrix1 is without filtering
            saveFMIRealImage("./result/22_23/original_have/result-filtered-trans.png", tempMatrix2, polarLogResolution + 1, 1, 10000.);
        }

        FMIReal maxValUsed;
        unsigned int maxCoordUsed;
        const FMIReal *tmpMatrixUsed = 0;

        FMIReal maxValOrig;
        unsigned int maxCoordOrig;

//        findMax(tempMatrix2, resPlusOne * resPlusOne, maxValOrig, maxCoordOrig);
//        findMaxByAngle(tempMatrix2, resPlusOne, maxValOrig, maxCoordOrig);
//        findMaxRay(tempMatrix2, resPlusOne * resPlusOne, maxValOrig, maxCoordOrig);
        double tmp_maxAngle = 0.;

        //----------------modified function HERE----------------//
        // change from sampling half line to sampling the whole line, and also store the maxAngle
        std::vector<double> trans_ray = findMaxRayAndSampling_All(tempMatrix2,
                                                              resPlusOne * resPlusOne,maxValOrig,
                                                              maxCoordOrig, tmp_maxAngle);
        // to store maxAngle
        if (i==0)
            MaxAngle = tmp_maxAngle;

        //cout << "result[0].rotationRad: " << results[0].rotationRad << endl;

        for (int j = 0; j < trans_ray.size(); ++j) {
            //cout << trans_ray[j] << " ";
            trans_ray[j] *= results[i].scaleEnergy / results[0].scaleEnergy;
        }
        //cout << "ratio: " << results[i].scaleEnergy / results[0].scaleEnergy << endl;
        trans_rays[i] = trans_ray;

        // now the stuff for the 180deg rotation...
        FMIReal maxValRotated = 0.;
        unsigned int maxCoordRotated = 0;

        if (over180deg) {
            // now do the actual matching...
            phaseOnlyMatching(first->phase, tempCompressedMatrix, fftMath.tmpMatrix,
                              resolution, true);
            fftMath.inverseFft2D();
            fftMath.spacialMagnitudeAndFFTShift(fftMath.tmpMatrix, tempMatrix1);
            filter(tempMatrix1, tempMatrix3, resolution);

            findMax(tempMatrix3, resPlusOne * resPlusOne, maxValRotated, maxCoordRotated);
        }

        // choose the correct rotation...
        if (over180deg) {
            if (maxValOrig >= maxValRotated) {
                maxValUsed = maxValOrig;
                maxCoordUsed = maxCoordOrig;
                tmpMatrixUsed = tempMatrix2;
            } else {
                maxValUsed = maxValRotated;
                maxCoordUsed = maxCoordRotated;
                tmpMatrixUsed = tempMatrix3;
                results[i].rotationRad += M_PI;
            }
        } else {
            maxValUsed = maxValOrig;
            maxCoordUsed = maxCoordOrig;
            tmpMatrixUsed = tempMatrix2;
        }

        FMIReal posX = (double) (maxCoordUsed % resPlusOne) + 1.0;
        FMIReal posY = ceil((double) (maxCoordUsed) / (double) (resPlusOne));

        FMIReal maxX = posX;
        FMIReal maxY = posY;

        // peaks stuff for the translation...
        Peaks peaks;
        Peak max;
        max.value = maxValOrig;
        max.x = posX;
        max.y = posY;
        peaks.peaks.push_back(max);

        calculateUncertainty(static_cast<int>(posX),
                             static_cast<int>(posY), tmpMatrixUsed, resPlusOne, results[i].signalTrans,
                             results[i].noiseTrans, results[i].covarianceMatrixTrans);

        // interpolate parameters within 3x3 area of determined dirac pulse
        interpolatePeak(tmpMatrixUsed, resPlusOne, posX, posY);
        results[i].transX = posX;
        results[i].transY = posY;
    }

    /*
     * 1. Combined all the trans_rays
     * 2. Rescaling (out of this function)
     */
    //------------------Potential Problem: why use max_element?-------------//
    transline.clear();
    for (int j = 0; j < trans_rays[0].size(); ++j) {
        vector<double> trans_values;
        for (int i = 0; i < trans_rays.size(); ++i) {
            trans_values.push_back(trans_rays[i][j]);
        }
        // use max
        transline.push_back(*max_element(trans_values.begin(), trans_values.end()));
    }

    return true;
}


bool FMIRegistration::registerUpToScale(FMIImage *first,
                                        FMIImage *second,
                                        std::vector<FMIResult> &results,
                                        std::vector<double> &peaksline,
                                        bool over180deg) {
    if (ONLY_TRANS) {
        // check the input data for sanity
        if (first == 0 || second == 0) {
            ERR(" First (%p) or second (%p) image pointer is NULL!", first, second);
            return false;
        }
        if (first->img == 0 || second->img == 0) {
            ERR(" First (%p) or second (%p) image is NULL!", first->img, second->img);
            return false;
        }
        if (resolution != first->resolution || resolution != second->resolution) {
            ERR(" FMI resolution (%d) differs from first (%d) and/ or second (%d) resolution!", resolution,
                first->resolution, second->resolution);
            return false;
        }

        FMIResult result;
        results.push_back(result);
        // make sure we have the phase information
        prepareFMIImage(first, false);
        prepareFMIImage(second, false);

        const unsigned int resPlusOne = resolution + 1;

        // now do the actual matching...
        phaseOnlyMatching(first->phase, second->phase, fftMath.tmpMatrix,
                          resolution, false);
        fftMath.inverseFft2D();
        fftMath.spacialMagnitudeAndFFTShift(fftMath.tmpMatrix, tempMatrix1);
        filter(tempMatrix1, tempMatrix2, resolution);

        if (dumpDebugInfo) {
            dumpValues("afterFilter.txt", tempMatrix2, polarLogResolution + 1, 1); // tempMatrix1 is without filtering
            saveFMIRealImage("result-filtered-trans.png", tempMatrix2, polarLogResolution + 1, 1, 10000.);
        }

        FMIReal maxValUsed;
        unsigned int maxCoordUsed;
        const FMIReal *tmpMatrixUsed = 0;

        FMIReal maxValOrig;
        unsigned int maxCoordOrig;

//        findMaxByAngle(tempMatrix2, resPlusOne, maxValOrig, maxCoordOrig);
//        findMax(tempMatrix2, resPlusOne * resPlusOne, maxValOrig, maxCoordOrig);
        peaksline = findMaxRayAndSampling(tempMatrix2, resPlusOne * resPlusOne, maxValOrig, maxCoordOrig);

        // now the stuff for the 180deg rotation...
        FMIReal maxValRotated = 0.;
        unsigned int maxCoordRotated = 0;

        maxValUsed = maxValOrig;
        maxCoordUsed = maxCoordOrig;
        tmpMatrixUsed = tempMatrix2;

        FMIReal posX = (double) (maxCoordUsed % resPlusOne) + 1.0;
        FMIReal posY = ceil((double) (maxCoordUsed) / (double) (resPlusOne));


        FMIReal maxX = posX;
        FMIReal maxY = posY;
        // peaks stuff for the translation...
        Peaks peaks;
        Peak max;
        max.value = maxValOrig;
        max.x = posX;
        max.y = posY;
        peaks.peaks.push_back(max);

        calculateUncertainty(static_cast<int>(posX),
                             static_cast<int>(posY), tmpMatrixUsed, resPlusOne, results[0].signalTrans,
                             results[0].noiseTrans, results[0].covarianceMatrixTrans);
        // interpolate parameters within 3x3 area of determined dirac pulse
        interpolatePeak(tmpMatrixUsed, resPlusOne, posX, posY);
        results[0].transX = posX;
        results[0].transY = posY;
        return true;
    } else {
        // check the input data for sanity
        if (first == 0 || second == 0) {
            ERR(" First (%p) or second (%p) image pointer is NULL!", first, second);
            return false;
        }
        if (first->img == 0 || second->img == 0) {
            ERR(" First (%p) or second (%p) image is NULL!", first->img, second->img);
            return false;
        }
        if (resolution != first->resolution || resolution != second->resolution) {
            ERR(" FMI resolution (%d) differs from first (%d) and/ or second (%d) resolution!", resolution,
                first->resolution, second->resolution);
            return false;
        }

        // Prepare the images - we just need the polar logarithmic part for now
        prepareFMIImage(first, true);
        prepareFMIImage(second, true);

        // determine rotation/scaling
        //   - phase only matching
        phaseOnlyMatching(second->polarLogPhase, first->polarLogPhase,
                          fftMathPolar.tmpMatrix, polarLogResolution, false);

        fftMathPolar.inverseFft2D();
        // spacial magnitude and fft shift
        fftMathPolar.spacialMagnitudeAndFFTShift(fftMathPolar.tmpMatrix, tempMatrix1);

        // tempMatrix has afterwards the resolution (polarLogResolution + 1)   !
        filter(tempMatrix1, tempMatrix2, polarLogResolution);

        if (dumpDebugInfo) {
            saveFMIRealImage("result-filtered-rot-scale.png", tempMatrix1, polarLogResolution, 1);
            dumpValues("values-rot-scale.txt", tempMatrix2, polarLogResolution + 1, 1);
        }

        bool success = findDiracPulsesScaleRotation(tempMatrix2, results);
        if ((!success) || results.empty()) {
            ERR("error finding scale and rotation...");
            return false;
        }

        if ((results[0].signalFMI / results[0].noiseFMI) < minFMISignalToNoise) {
            // minium signal to noise ratio not reached - we don't need to calculate further...
            cout << " too low signal to noise ratio!" << endl;
            return false;
        }

        // now we have the scale and rotation
        // we find out the translation now

        // first we scale and rotate the second image...
        scaleAndRotate(second->img, (FMIImageReal *) tempMatrix1, results[0].scale,
                       results[0].rotationRad);

        //    network_image::saveFMIRealImage("result-rotated.png", tempMatrix1, resolution);

        // now we prepare the rotated image...
        removeConstantOffsetApplySpectralWindow((FMIImageReal *) tempMatrix1, tempMatrix2);
        fftMath.fft2D(tempMatrix2, tempComplexMatrix1);
        saveAngles(tempComplexMatrix1, tempCompressedMatrix, resolution);

        // make sure we have the phase information
        prepareFMIImage(first, false);

        const unsigned int resPlusOne = resolution + 1;

        // now do the actual matching...
        phaseOnlyMatching(first->phase, tempCompressedMatrix, fftMath.tmpMatrix,
                          resolution, false);
        fftMath.inverseFft2D();
        fftMath.spacialMagnitudeAndFFTShift(fftMath.tmpMatrix, tempMatrix1);
        filter(tempMatrix1, tempMatrix2, resolution);

        if (dumpDebugInfo) {
            dumpValues("afterFilter.txt", tempMatrix2, polarLogResolution + 1, 1); // tempMatrix1 is without filtering
            saveFMIRealImage("result-filtered-trans.png", tempMatrix2, polarLogResolution + 1, 1, 10000.);
        }

        FMIReal maxValUsed;
        unsigned int maxCoordUsed;
        const FMIReal *tmpMatrixUsed = 0;

        FMIReal maxValOrig;
        unsigned int maxCoordOrig;

//        findMax(tempMatrix2, resPlusOne * resPlusOne, maxValOrig, maxCoordOrig);
//        findMaxByAngle(tempMatrix2, resPlusOne, maxValOrig, maxCoordOrig);
        peaksline = findMaxRayAndSampling(tempMatrix2, resPlusOne * resPlusOne, maxValOrig, maxCoordOrig);

        // now the stuff for the 180deg rotation...
        FMIReal maxValRotated = 0.;
        unsigned int maxCoordRotated = 0;

        if (over180deg) {
            // now do the actual matching...
            phaseOnlyMatching(first->phase, tempCompressedMatrix, fftMath.tmpMatrix,
                              resolution, true);
            fftMath.inverseFft2D();
            fftMath.spacialMagnitudeAndFFTShift(fftMath.tmpMatrix, tempMatrix1);
            filter(tempMatrix1, tempMatrix3, resolution);

            findMax(tempMatrix3, resPlusOne * resPlusOne, maxValRotated, maxCoordRotated);
        }

        // choose the correct rotation...
        if (over180deg) {
            if (maxValOrig >= maxValRotated) {
                maxValUsed = maxValOrig;
                maxCoordUsed = maxCoordOrig;
                tmpMatrixUsed = tempMatrix2;
//             cout<<" using less than 180 deg!"<<endl;
            } else {
                maxValUsed = maxValRotated;
                maxCoordUsed = maxCoordRotated;
                tmpMatrixUsed = tempMatrix3;
//             cout<<" using over 180 deg!"<<endl;
                results[0].rotationRad += M_PI;
            }
        } else {
            maxValUsed = maxValOrig;
            maxCoordUsed = maxCoordOrig;
            tmpMatrixUsed = tempMatrix2;
//            tmpMatrixUsed = tempMatrix1;
        }

        FMIReal posX = (double) (maxCoordUsed % resPlusOne) + 1.0;
        FMIReal posY = ceil((double) (maxCoordUsed) / (double) (resPlusOne));

        FMIReal maxX = posX;
        FMIReal maxY = posY;

        // peaks stuff for the translation...
        Peaks peaks;
        Peak max;
        max.value = maxValOrig;
        max.x = posX;
        max.y = posY;
        peaks.peaks.push_back(max);

        calculateUncertainty(static_cast<int>(posX),
                             static_cast<int>(posY), tmpMatrixUsed, resPlusOne, results[0].signalTrans,
                             results[0].noiseTrans, results[0].covarianceMatrixTrans);

        // interpolate parameters within 3x3 area of determined dirac pulse
        interpolatePeak(tmpMatrixUsed, resPlusOne, posX, posY);
        results[0].transX = posX;
        results[0].transY = posY;

//        if (sqrt(pow(posX - resolution / 2., 2) + pow(posY - resolution / 2., 2)) >= 5.) {
//            findMultiplePeaks(tmpMatrixUsed, polarLogResolution + 1, 5, peaks, 3);
//            for (auto &peak : peaks.peaks) {
//                if (fabs(peak.x - maxX) < 1e-6
//                    && fabs(peak.y - maxY) < 1e-6) {
//                    continue;
//                }
//                FMIReal peakX = peak.x;
//                FMIReal peakY = peak.y;
//                FMIResult tmpResult;
//                calculateUncertainty(static_cast<int>(peakX),
//                                     static_cast<int>(peakY), tmpMatrixUsed, resPlusOne, tmpResult.signalTrans,
//                                     tmpResult.noiseTrans, tmpResult.covarianceMatrixTrans);
//                interpolatePeak(tmpMatrixUsed, resPlusOne, peakX, peakY);
//                tmpResult.scale = results[0].scale;
//                tmpResult.rotationRad = results[0].rotationRad;
//                tmpResult.transX = peakX;
//                tmpResult.transY = peakY;
//                results.push_back(tmpResult);
//            }
//        }
        return true;
    }
}

bool FMIRegistration::registerImages(FMIImage *first, FMIImage *second, vector<FMIResult> &results, bool over180deg) {
    if (ONLY_TRANS) {
        // check the input data for sanity
        if (first == 0 || second == 0) {
            ERR(" First (%p) or second (%p) image pointer is NULL!", first, second);
            return false;
        }
        if (first->img == 0 || second->img == 0) {
            ERR(" First (%p) or second (%p) image is NULL!", first->img, second->img);
            return false;
        }
        if (resolution != first->resolution || resolution != second->resolution) {
            ERR(" FMI resolution (%d) differs from first (%d) and/ or second (%d) resolution!", resolution,
                first->resolution, second->resolution);
            return false;
        }

        FMIResult result;
        results.push_back(result);
        // make sure we have the phase information
        prepareFMIImage(first, false);
        prepareFMIImage(second, false);

        const unsigned int resPlusOne = resolution + 1;

        // now do the actual matching...
        phaseOnlyMatching(first->phase, second->phase, fftMath.tmpMatrix,
                          resolution, false);
        fftMath.inverseFft2D();
        fftMath.spacialMagnitudeAndFFTShift(fftMath.tmpMatrix, tempMatrix1);
        filter(tempMatrix1, tempMatrix2, resolution);

        if (dumpDebugInfo) {
            dumpValues("afterFilter.txt", tempMatrix2, polarLogResolution + 1, 1); // tempMatrix1 is without filtering
            saveFMIRealImage("result-filtered-trans.png", tempMatrix2, polarLogResolution + 1, 1, 10000.);
        }

        FMIReal maxValUsed;
        unsigned int maxCoordUsed;
        const FMIReal *tmpMatrixUsed = 0;

        FMIReal maxValOrig;
        unsigned int maxCoordOrig;

        findMax(tempMatrix2, resPlusOne * resPlusOne, maxValOrig, maxCoordOrig);

        // now the stuff for the 180deg rotation...
        FMIReal maxValRotated = 0.;
        unsigned int maxCoordRotated = 0;

        maxValUsed = maxValOrig;
        maxCoordUsed = maxCoordOrig;
        tmpMatrixUsed = tempMatrix2;

        FMIReal posX = (double) (maxCoordUsed % resPlusOne) + 1.0;
        FMIReal posY = ceil((double) (maxCoordUsed) / (double) (resPlusOne));

        // peaks stuff for the translation...
        Peaks peaks;
        Peak max;
        max.value = maxValOrig;
        max.x = posX;
        max.y = posY;
        peaks.peaks.push_back(max);
        // search for other peaks at least 5 pixel (Manhattan distance) away
        findOtherPeaks(tmpMatrixUsed, polarLogResolution + 1, peaks, 5, 20);
        // sum the energy around all peaks (sum the values of all values in 1 (Manhattan) distance away)
        sumPeakEnergies(tmpMatrixUsed, polarLogResolution + 1, peaks, 3);


        calculateUncertainty(static_cast<int>(posX),
                             static_cast<int>(posY), tmpMatrixUsed, resPlusOne, results[0].signalTrans,
                             results[0].noiseTrans, results[0].covarianceMatrixTrans);

        // interpolate parameters within 3x3 area of determined dirac pulse
        interpolatePeak(tmpMatrixUsed, resPlusOne, posX, posY);

//	DBG("done... x %f  y %f", posX, posY);
        results[0].transX = posX;
        results[0].transY = posY;
        return true;

    } else {
        // check the input data for sanity
        if (first == 0 || second == 0) {
            ERR(" First (%p) or second (%p) image pointer is NULL!", first, second);
            return false;
        }
        if (first->img == 0 || second->img == 0) {
            ERR(" First (%p) or second (%p) image is NULL!", first->img, second->img);
            return false;
        }
        if (resolution != first->resolution || resolution != second->resolution) {
            ERR(" FMI resolution (%d) differs from first (%d) and/ or second (%d) resolution!", resolution,
                first->resolution, second->resolution);
            return false;
        }
        // Prepare the images - we just need the polar logarithmic part for now
        prepareFMIImage(first, true);
        prepareFMIImage(second, true);

        // determine rotation/scaling
        //   - phase only matching
        phaseOnlyMatching(second->polarLogPhase, first->polarLogPhase,
                          fftMathPolar.tmpMatrix, polarLogResolution, false);

        fftMathPolar.inverseFft2D();
        // spacial magnitude and fft shift
        fftMathPolar.spacialMagnitudeAndFFTShift(fftMathPolar.tmpMatrix, tempMatrix1);

        // tempMatrix has afterwards the resolution (polarLogResolution + 1)   !
        filter(tempMatrix1, tempMatrix2, polarLogResolution);

        if (dumpDebugInfo) {
            saveFMIRealImage("result-filtered-rot-scale.png", tempMatrix1, polarLogResolution, 1);
            dumpValues("values-rot-scale.txt", tempMatrix2, polarLogResolution + 1, 1);
        }

        bool success = findDiracPulsesScaleRotation(tempMatrix2, results);
        if ((!success) || results.empty()) {
            ERR("error finding scale and rotation...");
            return false;
        }

        if ((results[0].signalFMI / results[0].noiseFMI) < minFMISignalToNoise) {
            // minium signal to noise ratio not reached - we don't need to calculate further...
            cout << " too low signal to noise ratio!" << endl;
            return false;
        }

        // now we have the scale and rotation
        // we find out the translation now

        // first we scale and rotate the second image...
//        results[0].scale = 1.;
//        results[0].rotationRad = -0.285457;
//        results[0].rotationRad = -0.24;
        scaleAndRotate(second->img, (FMIImageReal *) tempMatrix1, results[0].scale,
                       results[0].rotationRad);

//        saveFMIRealImage("result-rotated.png", tempMatrix1, resolution);
//        dumpValues("result-rotated.txt", tempMatrix1, resolution, 1);

        // now we prepare the rotated image...
        removeConstantOffsetApplySpectralWindow((FMIImageReal *) tempMatrix1, tempMatrix2);
        fftMath.fft2D(tempMatrix2, tempComplexMatrix1);
        saveAngles(tempComplexMatrix1, tempCompressedMatrix, resolution);

        // make sure we have the phase information
        prepareFMIImage(first, false);

        const unsigned int resPlusOne = resolution + 1;

        // now do the actual matching...
        phaseOnlyMatching(first->phase, tempCompressedMatrix, fftMath.tmpMatrix,
                          resolution, false);
        fftMath.inverseFft2D();
        fftMath.spacialMagnitudeAndFFTShift(fftMath.tmpMatrix, tempMatrix1);
        filter(tempMatrix1, tempMatrix2, resolution);

        if (dumpDebugInfo) {
            dumpValues("afterFilter.txt", tempMatrix2, polarLogResolution + 1, 1); // tempMatrix1 is without filtering
            saveFMIRealImage("result-filtered-trans.png", tempMatrix2, polarLogResolution + 1, 1, 10000.);
        }

        FMIReal maxValUsed;
        unsigned int maxCoordUsed;
        const FMIReal *tmpMatrixUsed = 0;

        FMIReal maxValOrig;
        unsigned int maxCoordOrig;

        findMax(tempMatrix2, resPlusOne * resPlusOne, maxValOrig, maxCoordOrig);
//        findMaxByAngle(tempMatrix2, resPlusOne, maxValOrig, maxCoordOrig);


        // now the stuff for the 180deg rotation...
        FMIReal maxValRotated = 0.;
        unsigned int maxCoordRotated = 0;

        if (over180deg) {
            // now do the actual matching...
            phaseOnlyMatching(first->phase, tempCompressedMatrix, fftMath.tmpMatrix,
                              resolution, true);
            fftMath.inverseFft2D();
            fftMath.spacialMagnitudeAndFFTShift(fftMath.tmpMatrix, tempMatrix1);
            filter(tempMatrix1, tempMatrix3, resolution);

            findMax(tempMatrix3, resPlusOne * resPlusOne, maxValRotated, maxCoordRotated);
        }

        // choose the correct rotation...
        if (over180deg) {
            if (maxValOrig >= maxValRotated) {
                maxValUsed = maxValOrig;
                maxCoordUsed = maxCoordOrig;
                tmpMatrixUsed = tempMatrix2;
//             cout<<" using less than 180 deg!"<<endl;
            } else {
                maxValUsed = maxValRotated;
                maxCoordUsed = maxCoordRotated;
                tmpMatrixUsed = tempMatrix3;
//             cout<<" using over 180 deg!"<<endl;
                results[0].rotationRad += M_PI;
            }
        } else {
            maxValUsed = maxValOrig;
            maxCoordUsed = maxCoordOrig;
            tmpMatrixUsed = tempMatrix2;
        }

        FMIReal posX = (double) (maxCoordUsed % resPlusOne) + 1.0;
        FMIReal posY = ceil((double) (maxCoordUsed) / (double) (resPlusOne));


        // peaks stuff for the translation...
        Peaks peaks;
        Peak max;
        max.value = maxValOrig;
        max.x = posX;
        max.y = posY;
        peaks.peaks.push_back(max);
//        findMultiplePeaks(tmpMatrixUsed, polarLogResolution + 1, 1, peaks);
        // search for other peaks at least 5 pixel (Manhattan distance) away
        findOtherPeaks(tmpMatrixUsed, polarLogResolution + 1, peaks, 5, 20);
        // sum the energy around all peaks (sum the values of all values in 1 (Manhattan) distance away)
        sumPeakEnergies(tmpMatrixUsed, polarLogResolution + 1, peaks, 3);

//    cout<<"Top Translation peaks:"<<endl;
//    if(true){
//        cout<<endl;
//        unsigned int count=0;
//        for( vector<Peak>::iterator itr = peaks.peaks.begin(); itr != peaks.peaks.end(); itr++){
//            // interpolate the values around the peak in order to achieve sub-pixel accuracy
//            FMIReal x = itr->x;
//            FMIReal y = itr->y;
//            interpolatePeak(tmpMatrixUsed, resPlusOne, x, y);
//            cout<<count++<<": at "<<itr->x<<", "<<itr->y<<" == "<<x<<", "<<y<<" energy: "<<itr->value<<endl;
//        }
//    }




        calculateUncertainty(static_cast<int>(posX),
                             static_cast<int>(posY), tmpMatrixUsed, resPlusOne, results[0].signalTrans,
                             results[0].noiseTrans, results[0].covarianceMatrixTrans);

        // interpolate parameters within 3x3 area of determined dirac pulse
        interpolatePeak(tmpMatrixUsed, resPlusOne, posX, posY);
//	DBG("done... x %f  y %f", posX, posY);
        results[0].transX = posX;
        results[0].transY = posY;
        return true;
    }
}

bool FMIRegistration::registerImagesMul(
        FMIImage *first,
        FMIImage *second,
        vector<FMIResult> &results,
        bool over180deg) {
    // check the input data for sanity
    if (first == 0 || second == 0) {
        ERR(" First (%p) or second (%p) image pointer is NULL!", first, second);
        return false;
    }
    if (first->img == 0 || second->img == 0) {
        ERR(" First (%p) or second (%p) image is NULL!", first->img, second->img);
        return false;
    }
    if (resolution != first->resolution || resolution != second->resolution) {
        ERR(" FMI resolution (%d) differs from first (%d) and/ or second (%d) resolution!", resolution,
            first->resolution, second->resolution);
        return false;
    }
    // Prepare the images - we just need the polar logarithmic part for now
    prepareFMIImage(first, true);
    prepareFMIImage(second, true);

    // determine rotation/scaling
    //   - phase only matching
    phaseOnlyMatching(second->polarLogPhase, first->polarLogPhase,
                      fftMathPolar.tmpMatrix, polarLogResolution, false);

    fftMathPolar.inverseFft2D();
    // spacial magnitude and fft shift
    fftMathPolar.spacialMagnitudeAndFFTShift(fftMathPolar.tmpMatrix, tempMatrix1);

    // tempMatrix has afterwards the resolution (polarLogResolution + 1)   !
    filter(tempMatrix1, tempMatrix2, polarLogResolution);

    if (dumpDebugInfo) {
        saveFMIRealImage("result-filtered-rot-scale.png", tempMatrix2, polarLogResolution + 1, 1, 10000.);
        dumpValues("values-rot-scale.txt", tempMatrix2, polarLogResolution + 1, 1);
    }

    bool success = findDiracPulsesScaleRotation(tempMatrix2, results);
    if ((!success) || results.empty()) {
        ERR("error finding scale and rotation...");
        return false;
    }
    FMIResult result = results[0];

    if ((result.signalFMI / result.noiseFMI) < minFMISignalToNoise) {
        // minium signal to noise ratio not reached - we don't need to calculate further...
        cout << " too low signal to noise ratio!" << endl;
        return false;
    }

    // now we have the scale and rotation
    // we find out the translation now

    // first we scale and rotate the second image...
    scaleAndRotate(second->img, (FMIImageReal *) tempMatrix1, result.scale,
                   result.rotationRad);

    //    network_image::saveFMIRealImage("result-rotated.png", tempMatrix1, resolution);

    // now we prepare the rotated image...
    removeConstantOffsetApplySpectralWindow((FMIImageReal *) tempMatrix1, tempMatrix2);
    fftMath.fft2D(tempMatrix2, tempComplexMatrix1);
    saveAngles(tempComplexMatrix1, tempCompressedMatrix, resolution);

    // make sure we have the phase information
    prepareFMIImage(first, false);

    const unsigned int resPlusOne = resolution + 1;

    // now do the actual matching...
    phaseOnlyMatching(first->phase, tempCompressedMatrix, fftMath.tmpMatrix,
                      resolution, false);
    fftMath.inverseFft2D();
    fftMath.spacialMagnitudeAndFFTShift(fftMath.tmpMatrix, tempMatrix1);
    filter(tempMatrix1, tempMatrix2, resolution);

    if (dumpDebugInfo) {
        dumpValues("afterFilter.txt", tempMatrix2, polarLogResolution + 1, 1);
        saveFMIRealImage("result-filtered-trans.png", tempMatrix2, polarLogResolution + 1, 1, 10000.);
    }

    FMIReal maxValUsed;
    unsigned int maxCoordUsed;
    const FMIReal *tmpMatrixUsed = 0;

    FMIReal maxValOrig;
    unsigned int maxCoordOrig;

    findMax(tempMatrix2, resPlusOne * resPlusOne, maxValOrig, maxCoordOrig);



    // now the stuff for the 180deg rotation...
    FMIReal maxValRotated = 0.;
    unsigned int maxCoordRotated = 0;

    if (over180deg) {
        // now do the actual matching...
        phaseOnlyMatching(first->phase, tempCompressedMatrix, fftMath.tmpMatrix,
                          resolution, true);
        fftMath.inverseFft2D();
        fftMath.spacialMagnitudeAndFFTShift(fftMath.tmpMatrix, tempMatrix1);
        filter(tempMatrix1, tempMatrix3, resolution);

        findMax(tempMatrix3, resPlusOne * resPlusOne, maxValRotated, maxCoordRotated);
    }

    // choose the correct rotation...
    if (over180deg) {
        if (maxValOrig >= maxValRotated) {
            maxValUsed = maxValOrig;
            maxCoordUsed = maxCoordOrig;
            tmpMatrixUsed = tempMatrix2;
        } else {
            maxValUsed = maxValRotated;
            maxCoordUsed = maxCoordRotated;
            tmpMatrixUsed = tempMatrix3;
            result.rotationRad += M_PI;
        }
    } else {
        maxValUsed = maxValOrig;
        maxCoordUsed = maxCoordOrig;
        tmpMatrixUsed = tempMatrix2;
    }

    FMIReal posX = (double) (maxCoordUsed % resPlusOne) + 1.0;
    FMIReal posY = ceil((double) (maxCoordUsed) / (double) (resPlusOne));


    // peaks stuff for the translation...
    Peaks peaks;
    Peak max;
    max.value = maxValOrig;
    max.x = posX;
    max.y = posY;
    peaks.peaks.push_back(max);
    // search for other peaks at least 5 pixel (Manhattan distance) away
    findOtherPeaks(tmpMatrixUsed, polarLogResolution + 1, peaks, 3, 20);
    // sum the energy around all peaks (sum the values of all values in 1 (Manhattan) distance away)
//    sumPeakEnergies(tmpMatrixUsed, polarLogResolution + 1, peaks, 3);

//    dumpValues("values-trans.txt", tmpMatrixUsed, polarLogResolution+1, 1);

    results.clear();
//    cout << "peak i : X Y val"<< endl;
    for (int i = 0; i < peaks.peaks.size(); ++i) {
        if (double(peaks.peaks[i].value) / double(peaks.peaks[0].value) < 0.9) break;
        FMIReal X = peaks.peaks[i].x;
        FMIReal Y = peaks.peaks[i].y;
//        cout << i << ": " << X << " " << Y << " " << peaks.peaks[i].value << endl;
        calculateUncertainty(static_cast<int>(X),
                             static_cast<int>(Y),
                             tmpMatrixUsed,
                             resPlusOne,
                             result.signalTrans,
                             result.noiseTrans,
                             result.covarianceMatrixTrans);

        // interpolate parameters within 3x3 area of determined dirac pulse
        interpolatePeak(tmpMatrixUsed, resPlusOne, X, Y);
        //	DBG("done... x %f  y %f", posX, posY);
        result.transX = X;
        result.transY = Y;
        results.push_back(result);
    }
    return true;
}


bool FMIRegistration::registerImages(FMIImage *first,
                                     FMIImage *second,
                                     std::vector<FMIResult> &results,
                                     std::string iFftFile,
                                     bool over180deg) {
    // check the input data for sanity
    if (first == 0 || second == 0) {
        ERR(" First (%p) or second (%p) image pointer is NULL!", first, second);
        return false;
    }
    if (first->img == 0 || second->img == 0) {
        ERR(" First (%p) or second (%p) image is NULL!", first->img, second->img);
        return false;
    }
    if (resolution != first->resolution || resolution != second->resolution) {
        ERR(" FMI resolution (%d) differs from first (%d) and/ or second (%d) resolution!", resolution,
            first->resolution, second->resolution);
        return false;
    }
    // Prepare the images - we just need the polar logarithmic part for now
    prepareFMIImage(first, true);
    prepareFMIImage(second, true);

    // determine rotation/scaling
    //   - phase only matching
    phaseOnlyMatching(second->polarLogPhase, first->polarLogPhase,
                      fftMathPolar.tmpMatrix, polarLogResolution, false);

    fftMathPolar.inverseFft2D();

    // spacial magnitude and fft shift
    fftMathPolar.spacialMagnitudeAndFFTShift(fftMathPolar.tmpMatrix, tempMatrix1);
    //save the magnitude of inverse FFT into a file
    saveInverseFFT(iFftFile, tempMatrix1, resolution);


    // tempMatrix has afterwards the resolution (polarLogResolution + 1)   !
    filter(tempMatrix1, tempMatrix2, polarLogResolution);

    if (dumpDebugInfo) {
        saveFMIRealImage("result-filtered-rot-scale.png", tempMatrix2, polarLogResolution + 1, 1, 10000.);
        dumpValues("values-rot-scale.txt", tempMatrix2, 600, 1);
    }

    bool success = findDiracPulsesScaleRotation(tempMatrix2, results);
    if ((!success) || results.empty()) {
        ERR("error finding scale and rotation...");
        return false;
    }

    if ((results[0].signalFMI / results[0].noiseFMI) < minFMISignalToNoise) {
        // minium signal to noise ratio not reached - we don't need to calculate further...
        cout << " too low signal to noise ratio!" << endl;
        return false;
    }

    // now we have the scale and rotation
    // we find out the translation now

    // first we scale and rotate the second image...
    scaleAndRotate(second->img, (FMIImageReal *) tempMatrix1, results[0].scale,
                   results[0].rotationRad);

    //    network_image::saveFMIRealImage("result-rotated.png", tempMatrix1, resolution);

    // now we prepare the rotated image...
    removeConstantOffsetApplySpectralWindow((FMIImageReal *) tempMatrix1, tempMatrix2);
    fftMath.fft2D(tempMatrix2, tempComplexMatrix1);
    saveAngles(tempComplexMatrix1, tempCompressedMatrix, resolution);

    // make sure we have the phase information
    prepareFMIImage(first, false);

    const unsigned int resPlusOne = resolution + 1;

    // now do the actual matching...
    phaseOnlyMatching(first->phase, tempCompressedMatrix, fftMath.tmpMatrix,
                      resolution, false);
    fftMath.inverseFft2D();
    fftMath.spacialMagnitudeAndFFTShift(fftMath.tmpMatrix, tempMatrix1);
    filter(tempMatrix1, tempMatrix2, resolution);

    if (dumpDebugInfo) {
//      dumpValues("afterFilter.txt", tempMatrix2, 256, 1);
        saveFMIRealImage("result-filtered-trans.png", tempMatrix2, polarLogResolution + 1, 1, 10000.);
    }

    FMIReal maxValUsed;
    unsigned int maxCoordUsed;
    const FMIReal *tmpMatrixUsed = 0;

    FMIReal maxValOrig;
    unsigned int maxCoordOrig;

    findMax(tempMatrix2, resPlusOne * resPlusOne, maxValOrig, maxCoordOrig);



    // now the stuff for the 180deg rotation...
    FMIReal maxValRotated = 0.;
    unsigned int maxCoordRotated = 0;

    if (over180deg) {
        // now do the actual matching...
        phaseOnlyMatching(first->phase, tempCompressedMatrix, fftMath.tmpMatrix,
                          resolution, true);
        fftMath.inverseFft2D();
        fftMath.spacialMagnitudeAndFFTShift(fftMath.tmpMatrix, tempMatrix1);
        filter(tempMatrix1, tempMatrix3, resolution);

        findMax(tempMatrix3, resPlusOne * resPlusOne, maxValRotated, maxCoordRotated);
    }

    // choose the correct rotation...
    if (over180deg) {
        if (maxValOrig >= maxValRotated) {
            maxValUsed = maxValOrig;
            maxCoordUsed = maxCoordOrig;
            tmpMatrixUsed = tempMatrix2;
//             cout<<" using less than 180 deg!"<<endl;
        } else {
            maxValUsed = maxValRotated;
            maxCoordUsed = maxCoordRotated;
            tmpMatrixUsed = tempMatrix3;
//             cout<<" using over 180 deg!"<<endl;
            results[0].rotationRad += M_PI;
        }
    } else {
        maxValUsed = maxValOrig;
        maxCoordUsed = maxCoordOrig;
        tmpMatrixUsed = tempMatrix2;
    }

    FMIReal posX = (double) (maxCoordUsed % resPlusOne) + 1.0;
    FMIReal posY = ceil((double) (maxCoordUsed) / (double) (resPlusOne));


    // peaks stuff for the translation...
    Peaks peaks;
    Peak max;
    max.value = maxValOrig;
    max.x = posX;
    max.y = posY;
    peaks.peaks.push_back(max);
    // search for other peaks at least 5 pixel (Manhattan distance) away
    findOtherPeaks(tmpMatrixUsed, polarLogResolution + 1, peaks, 5, 20);
    // sum the energy around all peaks (sum the values of all values in 1 (Manhattan) distance away)
    sumPeakEnergies(tmpMatrixUsed, polarLogResolution + 1, peaks, 3);

//	cout<<"Top Translation peaks:"<<endl;
//	if(true){
//		cout<<endl;
//		unsigned int count=0;
//		for( vector<Peak>::iterator itr = peaks.peaks.begin(); itr != peaks.peaks.end(); itr++){
//			// interpolate the values around the peak in order to achieve sub-pixel accuracy
//			FMIReal x = itr->x;
//			FMIReal y = itr->y;
//			interpolatePeak(tmpMatrixUsed, resPlusOne, x, y);
//			cout<<count++<<": at "<<itr->x<<", "<<itr->y<<" == "<<x<<", "<<y<<" energy: "<<itr->value<<endl;
//		}
//	}




    calculateUncertainty(static_cast<int>(posX),
                         static_cast<int>(posY), tmpMatrixUsed, resPlusOne, results[0].signalTrans,
                         results[0].noiseTrans, results[0].covarianceMatrixTrans);

    // interpolate parameters within 3x3 area of determined dirac pulse
    interpolatePeak(tmpMatrixUsed, resPlusOne, posX, posY);
//	DBG("done... x %f  y %f", posX, posY);
    results[0].transX = posX;
    results[0].transY = posY;
    return true;
}

void FMIRegistration::prepareFMIImage(FMIImage *img, bool needJustPolarLogPhase) {

    // if we just need the polarLogPhase and have it we return...
    if (needJustPolarLogPhase && img->polarLogPhase) {
        cout << "we have both..." << endl;
        return;
    }

    // first calculate the phase if needed

    //TODO: Hack because of BUG below
    if (!img->phase) {
        delete img->phase;
    }
    img->phase = new FMICompressedReal[resolution * resolution / 2 + resolution];
    // both in resolution
    removeConstantOffsetApplySpectralWindow(img->img, tempMatrix1);
    // both in resolution
    fftMath.fft2D(tempMatrix1, tempComplexMatrix1);
    // input in resolution, phase in (resolution x (resolution/2 + resolution) due to symmetries
    saveAngles(tempComplexMatrix1, img->phase, resolution);
    // this phase is used lateron to determine the translation
    //}

    // now for the polar logarithmic phase

    // we might already have the phase - check:
    if (img->polarLogPhase)
        return;

    img->polarLogPhase = new FMICompressedReal[polarLogResolution * polarLogResolution / 2 + polarLogResolution];

    //TODO: BUG! if phase existed tempComplexMatrix1 will be just garbage

    // both in resolution
    fftMath.spacialMagnitudeAndFFTShift(tempComplexMatrix1, tempMatrix1);
    FMIReal minimum;
    // in in resolution, out in polarLogResolution, the minimum is the minium value which was already found
    polarLogarithmicSamling(tempMatrix1, tempMatrix2, minimum);
    // in place calculation in polarLogResolution - uses the minium value
    postProcessFMI(tempMatrix2, minimum);
    // both in polarLogResolution
    fftMathPolar.fft2D(tempMatrix2, tempComplexMatrix1);
    // in in polarLogResolution, out in (polarLogResolution x (polarLogResolution/2 + polarLogResolution)
    saveAngles(tempComplexMatrix1, img->polarLogPhase, polarLogResolution);
}

void FMIRegistration::removeConstantOffsetApplySpectralWindow(
        FMIImageReal *inputData, FMIReal *outputData) const {
    FMIImageReal *in = inputData;
    const FMIImageReal *const end = in + squaredResolution;
    // calculate the sum
    FMIReal sum = 0.;
    for (; in < end; in++)
        sum += (*in);
    // create the processed data
    sum = sum / squaredResolution;
    in = inputData;
    FMIReal *out = outputData;
    const FMIReal *window = spectralWindow2D;
    for (; in < end; in++, out++, window++)
        *out = (*in - sum) * (*window);
}

void FMIRegistration::polarLogarithmicSamling(const FMIReal *const in,
                                              FMIReal *const out, FMIReal &minimum) const {
    minimum = numeric_limits<double>::max();
    FMIReal *outP = out;
    const FMIReal *const endP = out + squaredPolLogRes;
    const FMIReal *polLogIntMat = polLogInterpolationMatrices;
    const unsigned int *polLogCoords = polLogInterpolationCoordinates;
    for (; outP < endP; outP++) {
        // the y values (polLogCoords+0 and polLogCoords+1) are already
        // multiplied with the resolution...
        *outP = (*(polLogIntMat + 0)) * in[*(polLogCoords + 1) + *(polLogCoords
                                                                   + 3)] + // y2, x2
                (*(polLogIntMat + 1)) * in[*(polLogCoords + 1) + *(polLogCoords
                                                                   + 2)] + // y2, x1
                (*(polLogIntMat + 2)) * in[*(polLogCoords + 0) + *(polLogCoords
                                                                   + 3)] + // y1, x2
                (*(polLogIntMat + 3)) * in[*(polLogCoords + 0) + *(polLogCoords
                                                                   + 2)]; // y1, x1
        polLogIntMat += 4;
        polLogCoords += 4;

        // as a service for the next step we already look for the minimum...
        if (*outP < minimum)
            minimum = *outP;
    }
}

void FMIRegistration::postProcessFMI(FMIReal *const mat,
                                     const FMIReal &minimum) const {

    const FMIReal *gWind1DP = gaussianWindow1D;
    const FMIReal *const endGWind1DP = gaussianWindow1D + polarLogResolution;

    FMIReal *matP = mat;
    const FMIReal *endMatP;
    for (; gWind1DP < endGWind1DP; gWind1DP++) {
        endMatP = matP + polarLogResolution;
        for (; matP < endMatP; matP++) {
            *matP = (*gWind1DP) * log((*matP) - minimum + 1.0);
        }
    }
}

void
FMIRegistration::saveAngles(const ComplexFMIReal *const in, FMICompressedReal *const out, const unsigned int &res) {
    const ComplexFMIReal *inP = in;
    FMICompressedReal *outP = out;
    const FMICompressedReal *const endP = out + res * res / 2 + res; // half plus one line
    if (sizeof(FMICompressedReal) == sizeof(float)) {
        for (; outP < endP; outP++, inP++) {
            *outP = atan2f(inP->imag, inP->real);
        }
    } else {
        for (; outP < endP; outP++, inP++) {
            *outP = atan2(inP->imag, inP->real);
        }
    }
}

void FMIRegistration::phaseOnlyMatching(const FMICompressedReal *const in1,
                                        const FMICompressedReal *const in2, ComplexFMIReal *const out,
                                        const unsigned int &res, bool turn180deg) const {

    const FMICompressedReal *in1P = in1;
    const FMICompressedReal *in2P = in2;
    ComplexFMIReal *outP = out;
    // first for the half (plus one line) that we have:
    const ComplexFMIReal *const endP = out + res * res / 2 + res;
    // adding instead of substracting equals 180deg rotation
    if (turn180deg) {
        for (; outP < endP; outP++, in1P++, in2P++) {
            const FMIReal phase = (FMIReal) (*in1P) + (FMIReal) (*in2P);
            outP->real = cos(phase);
            outP->imag = sin(phase);
        }
    } else {
        for (; outP < endP; outP++, in1P++, in2P++) {
            const FMIReal phase = (FMIReal) (*in1P) - (FMIReal) (*in2P);
            outP->real = cos(phase);
            outP->imag = sin(phase);
        }
    }
    // now copy the symmetric rest:
    for (unsigned int y = res / 2 + 1; y < res; y++) {
        const unsigned mirrorY = (res - y) * res;
        for (unsigned int x = 0; x < res; x++, outP++) {
            unsigned mirrorX = res - x;
            if (x == 0) {
                mirrorX = 0;
            }
            (*outP) = out[mirrorY + mirrorX];
            // the symmetric rest is the conjugate-complex!
            outP->imag *= -1;
        }
    }
}

// old version - not used anymore - used directly the ffts
void FMIRegistration::phaseOnlyMatching(const ComplexFMIReal *const in1,
                                        const ComplexFMIReal *const in2, ComplexFMIReal *const out,
                                        const unsigned int &res) const {

    double phase;
    const ComplexFMIReal *in1P = in1;
    const ComplexFMIReal *in2P = in2;
    ComplexFMIReal *outP = out;
    const ComplexFMIReal *const endP = out + res * res;
    // calculate angle differences -> complex numbers
    for (; outP < endP; outP++, in1P++, in2P++) {
        phase = atan2(in1P->imag, in1P->real) - atan2(in2P->imag, in2P->real);
        outP->real = cos(phase);
        outP->imag = sin(phase);
    }
}


// [1 1; 1 1] filter(sum square pixels) -> group delay 0.5 in x,y direction
void FMIRegistration::filter(const FMIReal *const in, FMIReal *const out,
                             const unsigned int &res) const {

    const unsigned int resPlusOne = res + 1;

    // corners
    out[0] = in[0];
    out[res] = in[res - 1];
    out[res * resPlusOne] = in[(res - 1) * res];
    out[resPlusOne * resPlusOne - 1] = in[res * res - 1];

    // first/last row
    FMIReal *outFirstRow = out + 1;
    FMIReal *outLastRow = out + 1 + (res * resPlusOne);
    const FMIReal *inFirstRow = in;
    const FMIReal *inLastRow = in + (res - 1) * res;
    const FMIReal *const endInFirstRow = in + res - 1;
    for (; inFirstRow < endInFirstRow; outFirstRow++, outLastRow++, inFirstRow++, inLastRow++) {
        *outFirstRow = (*inFirstRow) + (*(inFirstRow + 1));
        *outLastRow = (*inLastRow) + (*(inLastRow + 1));
    }
    // first/last column
    FMIReal *outPFirstCol = out + resPlusOne;
    FMIReal *outPLastCol = outPFirstCol + res;
    const FMIReal *inPFirstCol = in;
    const FMIReal *inPLastCol = in + (res - 1);
    for (unsigned int ki = 0; ki < (res - 1); ki++) {
        *outPFirstCol = (*inPFirstCol) + (*(inPFirstCol + res));
        *outPLastCol = (*inPLastCol) + (*(inPLastCol + res));
        outPFirstCol += resPlusOne;
        outPLastCol += resPlusOne;
        inPFirstCol += res;
        inPLastCol += res;
    }
    // center
    FMIReal *outP;
    const FMIReal *inP;
    const FMIReal *inPnextLine;
    const FMIReal *endInP;
    for (unsigned int ki = 0; ki < (res - 1); ki++) {
        outP = out + (ki + 1) * resPlusOne + 1;
        inP = in + ki * res;
        inPnextLine = inP + res;
        endInP = inP + res - 1;
        for (; inP < endInP; inP++, inPnextLine++) {
            *(outP++) = *inP + *(inP + 1) + *inPnextLine + *(inPnextLine + 1);
        }
    }
}


bool FMIRegistration::findDiracPulsesScaleRotation(const FMIReal *const in, vector<
        FMIResult> &results) const {

    FMIReal maxVal;
    unsigned int maxCoord;

    // the inputdata has the resolution resPlusOne due to the filtering applied before
    const unsigned int resPlusOne = polarLogResolution + 1;

    // search for the maximum value
    findMax(in, resPlusOne * resPlusOne, maxVal, maxCoord);

    // calculate the coordinates of the peak
    double scalePos = ceil((double) maxCoord
                           / (double) (polarLogResolution + 1));
    double rotPos = (double) (maxCoord % (polarLogResolution + 1)) + 1.0;


    // find the other peaks in order to find out how good the first peak is
    Peaks peaks;
    Peak max;
    max.value = maxVal;
    max.x = rotPos;
    max.y = scalePos;
    peaks.peaks.push_back(max);
    // search for other peaks at least 5 pixel (Manhattan distance) away
    //TODO: Mistake API, here is at least 20 pixels away and find 5 other peaks
//	findOtherPeaks(in, polarLogResolution + 1, peaks, 5, 20);
    findOtherPeaks(in, polarLogResolution + 1, peaks, 5, 5);
    // sum the energy around all peaks (sum the values of all values in 3 (Manhattan) distance away)
    sumPeakEnergies(in, polarLogResolution + 1, peaks, 3);

//  cout<<"Top Rotation and scaling peaks:"<<endl;
//	if(true){
//		cout<<endl;
//		unsigned int count=0;
//		for( vector<Peak>::iterator itr = peaks.peaks.begin(); itr != peaks.peaks.end(); itr++){
//
//            // interpolate the values around the peak in order to achieve sub-pixel accuracy
//            FMIReal rot = itr->x;
//            FMIReal scale = itr->y;
//            interpolatePeak(in, resPlusOne, rot, scale);
//            cout<<count++<<": at "<<itr->x<<", "<<itr->y<<" == "<<
//            pow((double) polarLogResolution - 1.0, scale / ((double) polarLogResolution - 1.0)) <<" scale "<<
//            (rot / (double) polarLogResolution) *180.<<" deg  energy: "<<itr->value<<endl;
//		}
//	}

    if (use2ndRotationPeak) {
        cout << "using 2nd rotation peak! " << endl;
        scalePos = peaks.peaks[1].y;
        rotPos = peaks.peaks[1].x;
    }

    // find out second biggest sum (can be different than the second biggest peak!):
    vector<Peak>::iterator itr2nd = peaks.peaks.begin();
    FMIReal max2ndVal = 0.;
    itr2nd++; // Omit the actual (1st) peak
    for (; itr2nd != peaks.peaks.end(); itr2nd++) {
        if (itr2nd->value > max2ndVal) max2ndVal = itr2nd->value;
    }
    FMIResult result;
    result.peakEnergy = peaks.peaks[0].value;
    result.secondPeakEnergy = max2ndVal;

    // check if the actual peak is big enough compared to the second
    if (max2ndVal * minFMIsignalFactorOver2nd > peaks.peaks[0].value) {
        // we discard this result
        cout << " discarding scan because of too big 2nd peak (value 2nd: " << max2ndVal << "; factor: "
             << minFMIsignalFactorOver2nd << "; peak 1st: " << peaks.peaks[0].value << ")!" << endl;
        results.push_back(result);
        return false;
    }


    // calculate the uncertainty
    calculateUncertainty(static_cast<int>(scalePos),
                         static_cast<int>(rotPos), in, polarLogResolution + 1, result.signalFMI, result.noiseFMI,
                         result.covarianceMatrixFMI);

    // interpolate the values around the peak in order to achieve sub-pixel accuracy
    interpolatePeak(in, resPlusOne, rotPos, scalePos);

    // calculate the actual rotation and scale values
    double angleRad = (rotPos / (double) polarLogResolution) * M_PI; // determine angle from dirac position
    double scale = pow((double) polarLogResolution - 1.0, scalePos
                                                          / ((double) polarLogResolution - 1.0)); // determine scaling

    result.rotationRad = angleRad;
    result.scale = scale;

    results.push_back(result);
    return true;
}

bool FMIRegistration::findDiracPulsesMultiScaleRotation(const FMIReal *const in,
                                                        std::vector<FMIResult> &results,
                                                        std::vector<double> &peaksline) const {
    FMIReal maxVal;
    unsigned int maxCoord;

    // the inputdata has the resolution resPlusOne due to the filtering applied before
    const unsigned int resPlusOne = polarLogResolution + 1;

    // search for the maximum value
    findMax(in, resPlusOne * resPlusOne, maxVal, maxCoord);

    // calculate the coordinates of the peak
    double scalePos = ceil((double) maxCoord
                           / (double) (polarLogResolution + 1));
    double rotPos = (double) (maxCoord % (polarLogResolution + 1)) + 1.0;


    // find multiple scaling peaks
    Peaks peaks;
    Peak max;
    max.value = maxVal;
    max.x = rotPos;
    max.y = scalePos;
    peaks.peaks.push_back(max);
    findColPeaks(in, resPlusOne, 5, peaks, peaksline);

    interpolatePeak(in, resPlusOne, rotPos, scalePos);
    double angleRad = (rotPos / (double) polarLogResolution) * M_PI; // determine angle from dirac position
    double scale = pow((double) polarLogResolution - 1.0, scalePos
                                                          /
                                                          ((double) polarLogResolution - 1.0)); // determine scaling
    FMIResult result;
    result.rotationRad = angleRad;
    result.scale = scale;
    result.scaleEnergy = maxVal;
    results.push_back(result);

    // Sampling scaling according to the min and max value
    sort(peaks.peaks.begin(), peaks.peaks.end(),
         [](const Peak A, const Peak B) {
             return pow(A.y, 2) > pow(B.y, 2);
         }); // descending order
    unsigned int max_y = peaks.peaks[0].y;
    unsigned int min_y = peaks.peaks[peaks.peaks.size() - 1].y;
    int step = 1;
    int cnt = 0;
    int len = (max_y - min_y + 1)/step;
    for (int i = min_y; i <= max_y; i += step) {
        FMIReal rotPosTmp = peaks.peaks[0].x; // all is the same
        FMIReal scalePosTmp = i;
        interpolatePeak(in, resPlusOne, rotPosTmp, scalePosTmp);

        // calculate the actual rotation and scale values
        angleRad = (rotPosTmp / (double) polarLogResolution) * M_PI; // determine angle from dirac position
        scale = pow((double) polarLogResolution - 1.0, scalePosTmp
                                                              /
                                                              ((double) polarLogResolution - 1.0)); // determine scaling

        result.rotationRad = angleRad;
        result.scale = scale;
        int idx = static_cast<int>(double(cnt) / double(len) * double(peaks.peaks.size()));
        result.scaleEnergy = peaks.peaks[idx].value;

        results.push_back(result);
        cnt++;
    }
    return true;
}

FMIReal FMIRegistration::calculateSum(const unsigned int &peakX,
                                      const unsigned int &peakY, const FMIReal *const in,
                                      const unsigned int &resolution, const unsigned int &halfSize) {

    int val;
    // calculate fromX
    val = static_cast<int> (peakX) - halfSize;
    if (val < 0)
        val = 0;
    const unsigned int fromX = val;
    // calculate toX
    val = static_cast<int> (peakX) + halfSize;
    if (val >= (int) resolution)
        val = resolution - 1;
    const unsigned int toX = val;
    // calculate fromY
    val = static_cast<int> (peakY) - halfSize;
    if (val < 0)
        val = 0;
    const unsigned int fromY = val;
    // calculate toY
    val = static_cast<int> (peakY) + halfSize;
    if (val >= (int) resolution)
        val = resolution - 1;
    const unsigned int toY = val;

    FMIReal sum = 0.;
    const FMIReal *inP;
    // iterate over the whole uncertainty area
    for (unsigned int x = fromX; x <= toX; x++) {
        inP = in + x * resolution + fromY;
        for (unsigned int y = fromY; y <= toY; y++, inP++) {
            sum += *inP;
        }
    }

    return sum;
}

void FMIRegistration::calculateUncertainty(const unsigned int &peakX,
                                           const unsigned int &peakY, const FMIReal *const in,
                                           const unsigned int &resolution, FMIReal &signal, FMIReal &noise,
                                           FMIReal cov[2][2]) {

    const unsigned int halfNoiseSize = 10;
    const unsigned int halfSignalSize = 1;

    // calculate signal and noise energies
    FMIReal sum = calculateSum(peakX - 1, peakY - 1, in, resolution, halfNoiseSize);
    signal = calculateSum(peakX - 1, peakY - 1, in, resolution, halfSignalSize);
    noise = (sum - signal) / sum; // normalized
    signal = signal / sum; // normalize

//	cout<<" signal "<<result.signalFMI<<" noise "<<result.noiseFMI<<" sum "<<sum<<endl;

    // Determine in the covariance

    FMIReal val00 = 0.;
    FMIReal val01and10 = 0.;
    FMIReal val11 = 0.;

    int val;
    // calculate fromX
    val = static_cast<int> (peakX) - halfNoiseSize - 1;
    if (val < 0)
        val = 0;
    const unsigned int fromX = val;
    // calculate toX
    val = static_cast<int> (peakX) + halfNoiseSize - 1;
    if (val >= (int) resolution)
        val = resolution - 1;
    const unsigned int toX = val;
    // calculate fromY
    val = static_cast<int> (peakY) - halfNoiseSize - 1;
    if (val < 0)
        val = 0;
    const unsigned int fromY = val;
    // calculate toY
    val = static_cast<int> (peakY) + halfNoiseSize - 1;
    if (val >= (int) resolution)
        val = resolution - 1;
    const unsigned int toY = val;

    const FMIReal *inP;
    // iterate over the whole uncertainty area
    int distX = fromX - peakX + 1;
    const int startDistY = fromY - peakY;
    for (unsigned int x = fromX; x <= toX; x++, distX++) {
        inP = in + x * resolution + fromY;
        bool lineWithSignal = x >= peakX - halfSignalSize - 1 && x <= peakX + halfSignalSize - 1;
        const FMIReal squaredDistX = distX * distX;
        int distY = startDistY + 1;
        for (unsigned int y = fromY; y <= toY; y++, inP++, distY++) {
            // do not use the signal values
            if (lineWithSignal && y >= peakY - halfSignalSize - 1 && y <= peakY + halfSignalSize - 1) {
                continue;
            }
            val00 += *inP * squaredDistX;
            val01and10 += *inP * distX * distY;
            val11 += *inP * distY * distY;
        }
    }
    cov[0][0] = val00 / sum;
    cov[0][1] = val01and10 / sum;
    cov[1][0] = val01and10 / sum;
    cov[1][1] = val11 / sum;
//	cout<<"CoV: "<<result.covarianceMatrixFMI[0][0]<<" "<<result.covarianceMatrixFMI[0][1];
//	cout<<" "<<result.covarianceMatrixFMI[1][0]<<" "<<result.covarianceMatrixFMI[1][1]<<endl;
}

void FMIRegistration::scaleAndRotate(const FMIImageReal *const in,
                                     FMIImageReal *const out, const FMIReal &scale, const FMIReal &rotRad) const {
//	if (true) {
//		scaleAndRotateHeiko(in, out, scale, rotRad);
//		return;
//	}
    if (true) {
        scaleAndRotateCheck(in, out, scale, rotRad);
        return;
    }
    scaleAndRotateInterp(in, out, scale, rotRad);
}

void FMIRegistration::scaleAndRotateInterp(const FMIImageReal *const in,
                                           FMIImageReal *const out, const FMIReal &scale, const FMIReal &rotRad) const {

    double afm_i[3][3];
    double afm[3][3];
    double shift1[3][3], shift2[3][3], shift3[3][3];
    double rot_m[3][3], scale_m[3][3];

    double corfac;
    if (scale > 1.0) { // determine translation offset due to scaling
        corfac = -(double) round(fabs(((double) resolution * scale
                                       - (double) resolution) / 2.0));
    } else {
        corfac = (double) round(fabs(((double) resolution * scale
                                      - (double) resolution) / 2.0));
    }
    // shift3
    shift3[0][0] = 1.0;
    shift3[0][1] = 0.0;
    shift3[0][2] = corfac;
    shift3[1][0] = 0.0;
    shift3[1][1] = 1.0;
    shift3[1][2] = corfac;
    shift3[2][0] = 0.0;
    shift3[2][1] = 0.0;
    shift3[2][2] = 1.0;
    // shift1
    double trans = (double) resolution / 2.0 + -0.5; // shift/reshift to center of image frame
    shift1[0][0] = 1.0;
    shift1[0][1] = 0.0;
    shift1[0][2] = trans;
    shift1[1][0] = 0.0;
    shift1[1][1] = 1.0;
    shift1[1][2] = trans;
    shift1[2][0] = 0.0;
    shift1[2][1] = 0.0;
    shift1[2][2] = 1.0;
    // shift2
    shift2[0][0] = 1.0;
    shift2[0][1] = 0.0;
    shift2[0][2] = -trans;
    shift2[1][0] = 0.0;
    shift2[1][1] = 1.0;
    shift2[1][2] = -trans;
    shift2[2][0] = 0.0;
    shift2[2][1] = 0.0;
    shift2[2][2] = 1.0;
    // scale matrix
    scale_m[0][0] = scale;
    scale_m[0][1] = 0.0;
    scale_m[0][2] = 0.0;
    scale_m[1][0] = 0.0;
    scale_m[1][1] = scale;
    scale_m[1][2] = 0.0;
    scale_m[2][0] = 0.0;
    scale_m[2][1] = 0.0;
    scale_m[2][2] = 1.0;
    // rot matrix
    const double angleCos = cos(rotRad);
    const double angleSin = sin(rotRad);
    rot_m[0][0] = angleCos;
    rot_m[0][1] = -angleSin;
    rot_m[0][2] = 0.0;
    rot_m[1][0] = angleSin;
    rot_m[1][1] = angleCos;
    rot_m[1][2] = 0.0;
    rot_m[2][0] = 0.0;
    rot_m[2][1] = 0.0;
    rot_m[2][2] = 1.0;

    // merge transformations -> shift3*mscale*shift1*mrot*shift2
    matMult3x3(rot_m, shift2, afm);
    matMult3x3(shift1, afm, afm_i);
    matMult3x3(scale_m, afm_i, shift1);
    matMult3x3(shift3, shift1, afm);

    // calculate 3x3 inverse matrix
    const double D = 1. / (afm[0][0] * (afm[1][1] * afm[2][2] - afm[1][2]
                                                                * afm[2][1]) -
                           afm[1][0] * (afm[0][1] * afm[2][2] - afm[0][2]
                                                                * afm[2][1]) +
                           afm[2][0] * (afm[0][1] * afm[1][2] - afm[0][2]
                                                                * afm[1][1]));

    afm_i[0][0] = +(afm[1][1] * afm[2][2] - afm[1][2] * afm[2][1]) * D;
    afm_i[1][0] = -(afm[1][0] * afm[2][2] - afm[1][2] * afm[2][0]) * D;
    //     afm_i[2][0] = +(afm[1][0]*afm[2][1] - afm[1][1]*afm[2][0]) * D;
    afm_i[0][1] = -(afm[0][1] * afm[2][2] - afm[0][2] * afm[2][1]) * D;
    afm_i[1][1] = +(afm[0][0] * afm[2][2] - afm[2][0] * afm[0][2]) * D;
    //     afm_i[2][1] = -(afm[0][0]*afm[2][1] - afm[0][1]*afm[2][0]) * D;
    afm_i[0][2] = +(afm[0][1] * afm[1][2] - afm[0][2] * afm[1][1]) * D;
    afm_i[1][2] = -(afm[0][0] * afm[1][2] - afm[1][0] * afm[0][2]) * D;
    //     afm_i[2][2] = +(afm[0][0]*afm[1][1] - afm[1][0]*afm[0][1]) * D;

    FMIImageReal *outP = out;
    for (unsigned int m = 0; m < resolution; m++) {
        const unsigned int veci1 = static_cast<double> (m);
        for (unsigned int k = 0; k < resolution; k++) {
            // matrix*vector
            const double veco0 = afm_i[0][0] * k + afm_i[0][1] * veci1
                                 + afm_i[0][2];
            const double veco1 = afm_i[1][0] * k + afm_i[1][1] * veci1
                                 + afm_i[1][2];
            const unsigned int left = floor(veco0);
            const unsigned int right = ceil(veco0);
            const unsigned int up = floor(veco1);
            const unsigned int down = ceil(veco1);
            const double leftVal = 1 - (veco0 - floor(veco0));
            double rightVal = 1 - (ceil(veco0) - veco0);
            if (left == right)
                rightVal = 0.;
            const double upVal = 1 - (veco1 - floor(veco1));
            double downVal = 1 - (ceil(veco1) - veco1);
            if (up == down)
                downVal = 0.;
            // inverse access to target matrix
            double upLeft = 0.;
            double upRight = 0.;
            double downLeft = 0.;
            double downRight = 0.;
            if (up < resolution) {
                if (left < resolution)
                    upLeft = leftVal * upVal * in[left + resolution * up];
                if (right < resolution)
                    upRight = rightVal * upVal * in[right + resolution * up];
            }
            if (down < resolution) {
                if (left < resolution)
                    downLeft = leftVal * downVal * in[left + resolution * down];
                if (right < resolution)
                    downRight = rightVal * downVal * in[right + resolution
                                                                * down];
            }
            *outP++ = upLeft + upRight + downLeft + downRight;
        }
    }
}


void FMIRegistration::scaleAndRotateCheck(const FMIImageReal *const in,
                                          FMIImageReal *const out, const FMIReal &scale, const FMIReal &rotRad) const {

    double afm_i[3][3];
    double afm[3][3];
    double shift1[3][3], shift2[3][3], shift3[3][3];
    double rot_m[3][3], scale_m[3][3];

    // First: calculate the matrix used for the image transofrmation

    double corfac;
    if (scale > 1.0) { // determine translation offset due to scaling
        corfac = -(double) round(fabs(((double) resolution * scale
                                       - (double) resolution) / 2.0));
    } else {
        corfac = (double) round(fabs(((double) resolution * scale
                                      - (double) resolution) / 2.0));
    }
    // shift3
    shift3[0][0] = 1.0;
    shift3[0][1] = 0.0;
    shift3[0][2] = corfac;
    shift3[1][0] = 0.0;
    shift3[1][1] = 1.0;
    shift3[1][2] = corfac;
    shift3[2][0] = 0.0;
    shift3[2][1] = 0.0;
    shift3[2][2] = 1.0;
    // shift1
    double trans = (double) resolution / 2.0 + -0.5; // shift/reshift to center of image frame
    shift1[0][0] = 1.0;
    shift1[0][1] = 0.0;
    shift1[0][2] = trans;
    shift1[1][0] = 0.0;
    shift1[1][1] = 1.0;
    shift1[1][2] = trans;
    shift1[2][0] = 0.0;
    shift1[2][1] = 0.0;
    shift1[2][2] = 1.0;
    // shift2
    shift2[0][0] = 1.0;
    shift2[0][1] = 0.0;
    shift2[0][2] = -trans;
    shift2[1][0] = 0.0;
    shift2[1][1] = 1.0;
    shift2[1][2] = -trans;
    shift2[2][0] = 0.0;
    shift2[2][1] = 0.0;
    shift2[2][2] = 1.0;
    // scale matrix
    scale_m[0][0] = scale;
    scale_m[0][1] = 0.0;
    scale_m[0][2] = 0.0;
    scale_m[1][0] = 0.0;
    scale_m[1][1] = scale;
    scale_m[1][2] = 0.0;
    scale_m[2][0] = 0.0;
    scale_m[2][1] = 0.0;
    scale_m[2][2] = 1.0;
    // rot matrix
    const double angleCos = cos(rotRad);
    const double angleSin = sin(rotRad);
    rot_m[0][0] = angleCos;
    rot_m[0][1] = -angleSin;
    rot_m[0][2] = 0.0;
    rot_m[1][0] = angleSin;
    rot_m[1][1] = angleCos;
    rot_m[1][2] = 0.0;
    rot_m[2][0] = 0.0;
    rot_m[2][1] = 0.0;
    rot_m[2][2] = 1.0;

    // merge transformations -> shift3*mscale*shift1*mrot*shift2
    matMult3x3(rot_m, shift2, afm);
    matMult3x3(shift1, afm, afm_i);
    matMult3x3(scale_m, afm_i, shift1);
    matMult3x3(shift3, shift1, afm);

    // calculate 3x3 inverse matrix
    const double D = 1. / (afm[0][0] * (afm[1][1] * afm[2][2] - afm[1][2]
                                                                * afm[2][1]) -
                           afm[1][0] * (afm[0][1] * afm[2][2] - afm[0][2]
                                                                * afm[2][1]) +
                           afm[2][0] * (afm[0][1] * afm[1][2] - afm[0][2]
                                                                * afm[1][1]));

    afm_i[0][0] = +(afm[1][1] * afm[2][2] - afm[1][2] * afm[2][1]) * D;
    afm_i[1][0] = -(afm[1][0] * afm[2][2] - afm[1][2] * afm[2][0]) * D;
    //     afm_i[2][0] = +(afm[1][0]*afm[2][1] - afm[1][1]*afm[2][0]) * D;
    afm_i[0][1] = -(afm[0][1] * afm[2][2] - afm[0][2] * afm[2][1]) * D;
    afm_i[1][1] = +(afm[0][0] * afm[2][2] - afm[2][0] * afm[0][2]) * D;
    //     afm_i[2][1] = -(afm[0][0]*afm[2][1] - afm[0][1]*afm[2][0]) * D;
    afm_i[0][2] = +(afm[0][1] * afm[1][2] - afm[0][2] * afm[1][1]) * D;
    afm_i[1][2] = -(afm[0][0] * afm[1][2] - afm[1][0] * afm[0][2]) * D;
    //     afm_i[2][2] = +(afm[0][0]*afm[1][1] - afm[1][0]*afm[0][1]) * D;

    // fill in the new pixels by getting the according pixel of the unroteded image
    FMIImageReal *outP = out;
    for (unsigned int m = 0; m < resolution; m++) {
        const unsigned int veci1 = static_cast<double> (m);
        for (unsigned int k = 0; k < resolution; k++) {
            // matrix*vector
            const unsigned int veco0 = ROUND(afm_i[0][0] * k + afm_i[0][1]
                                                               * veci1 + afm_i[0][2]);
            const unsigned int veco1 = ROUND(afm_i[1][0] * k + afm_i[1][1]
                                                               * veci1 + afm_i[1][2]);
            // inverse access to target matrix
            // test if we are still within the bounds of the row and column
            // uses the trick that "negative" unsigned numbers are really big
            if (veco0 < resolution && veco1 < resolution) {
                *outP++ = in[veco0 + resolution * veco1];
            } else {
                *outP++ = 0.;
            }
        }
    }
}

/*
void FMIRegistration::scaleAndRotateHeiko(const FMIImageReal * const in,
		FMIImageReal * const out, const FMIReal &scale, const FMIReal &rotRad) const{

	double afm_i[3][3];
	double afm[3][3];
	double shift1[3][3], shift2[3][3], shift3[3][3];
	double rot_m[3][3], scale_m[3][3];

	// new size
	const unsigned int frame = 4 * resolution;
	const unsigned int fWhatever = resolution + 2 * frame;

	// copy image to interims matrix(generate zero frame due to inverse processing)
	const FMIImageReal * inP = in;
	for (unsigned int m = 0; m < resolution; m++) {
		for (unsigned int k = 0; k < resolution; k++) {
			tempScaleMat[(m + frame) + fWhatever * (k + frame)] = inP[m
					+ resolution * k];
		}
	}

	double corfac;
	if (scale > 1.0) { // determine translation offset due to scaling
		corfac = -(double) round(fabs(((double) resolution * scale
				- (double) resolution) / 2.0));
	} else {
		corfac = (double) round(fabs(((double) resolution * scale
				- (double) resolution) / 2.0));
	}
	// shift3
	shift3[0][0] = 1.0;
	shift3[0][1] = 0.0;
	shift3[0][2] = corfac;
	shift3[1][0] = 0.0;
	shift3[1][1] = 1.0;
	shift3[1][2] = corfac;
	shift3[2][0] = 0.0;
	shift3[2][1] = 0.0;
	shift3[2][2] = 1.0;
	// shift1
	double trans = (double) resolution / 2.0 + 0.5; // shift/reshift to center of image frame
	shift1[0][0] = 1.0;
	shift1[0][1] = 0.0;
	shift1[0][2] = trans;
	shift1[1][0] = 0.0;
	shift1[1][1] = 1.0;
	shift1[1][2] = trans;
	shift1[2][0] = 0.0;
	shift1[2][1] = 0.0;
	shift1[2][2] = 1.0;
	// shift2
	shift2[0][0] = 1.0;
	shift2[0][1] = 0.0;
	shift2[0][2] = -trans;
	shift2[1][0] = 0.0;
	shift2[1][1] = 1.0;
	shift2[1][2] = -trans;
	shift2[2][0] = 0.0;
	shift2[2][1] = 0.0;
	shift2[2][2] = 1.0;
	// scale matrix
	scale_m[0][0] = scale;
	scale_m[0][1] = 0.0;
	scale_m[0][2] = 0.0;
	scale_m[1][0] = 0.0;
	scale_m[1][1] = scale;
	scale_m[1][2] = 0.0;
	scale_m[2][0] = 0.0;
	scale_m[2][1] = 0.0;
	scale_m[2][2] = 1.0;
	// rot matrix
	const double angleCos = cos(rotRad);
	const double angleSin = sin(rotRad);
	rot_m[0][0] = angleCos;
	rot_m[0][1] = -angleSin;
	rot_m[0][2] = 0.0;
	rot_m[1][0] = angleSin;
	rot_m[1][1] = angleCos;
	rot_m[1][2] = 0.0;
	rot_m[2][0] = 0.0;
	rot_m[2][1] = 0.0;
	rot_m[2][2] = 1.0;

	// merge transformations -> shift3*mscale*shift1*mrot*shift2
	matMult3x3(rot_m, shift2, afm);
	matMult3x3(shift1, afm, afm_i);
	matMult3x3(scale_m, afm_i, shift1);
	matMult3x3(shift3, shift1, afm);

	// calculate 3x3 inverse matrix
	const double D = 1. / (afm[0][0] * (afm[1][1] * afm[2][2] - afm[1][2]
			* afm[2][1]) - afm[1][0] * (afm[0][1] * afm[2][2] - afm[0][2]
			* afm[2][1]) + afm[2][0] * (afm[0][1] * afm[1][2] - afm[0][2]
			* afm[1][1]));

	afm_i[0][0] = +(afm[1][1] * afm[2][2] - afm[1][2] * afm[2][1]) * D;
	afm_i[1][0] = -(afm[1][0] * afm[2][2] - afm[1][2] * afm[2][0]) * D;
	//     afm_i[2][0] = +(afm[1][0]*afm[2][1] - afm[1][1]*afm[2][0]) * D;
	afm_i[0][1] = -(afm[0][1] * afm[2][2] - afm[0][2] * afm[2][1]) * D;
	afm_i[1][1] = +(afm[0][0] * afm[2][2] - afm[2][0] * afm[0][2]) * D;
	//     afm_i[2][1] = -(afm[0][0]*afm[2][1] - afm[0][1]*afm[2][0]) * D;
	afm_i[0][2] = +(afm[0][1] * afm[1][2] - afm[0][2] * afm[1][1]) * D;
	afm_i[1][2] = -(afm[0][0] * afm[1][2] - afm[1][0] * afm[0][2]) * D;
	//     afm_i[2][2] = +(afm[0][0]*afm[1][1] - afm[1][0]*afm[0][1]) * D;

	const unsigned int frameMinus1 = frame - 1;
	FMIImageReal * outP = out;
	for (unsigned int m = 1; m < (resolution + 1); m++) {
		const double veci1 = static_cast<double> (m);
		for (unsigned int k = 1; k < (resolution + 1); k++) {
			const double veci0 = static_cast<double> (k);
			// matrix*vector
			const double veco0 = afm_i[0][0] * veci0 + afm_i[0][1] * veci1
					+ afm_i[0][2];
			const double veco1 = afm_i[1][0] * veci0 + afm_i[1][1] * veci1
					+ afm_i[1][2];
			// inverse access to target matrix
			*outP++ = tempScaleMat[static_cast<unsigned int> ((ROUND(veco0)
					+ frameMinus1) + fWhatever * (ROUND(veco1) + frameMinus1))];
		}
	}
}
*/

void FMIRegistration::interpolatePeak(const FMIReal *const in,
                                      const unsigned int &res, FMIReal &peakX, FMIReal &peakY) const {

    if (peakX < 2. || peakY < 2. || peakX > (res - 2.) || peakY > (res - 2.)) {
        // we have to use the secure version because we are at the border...
        secureInterpolatePeak(in, res, peakX, peakY);
        return;
    }

    const unsigned int startX = static_cast<unsigned int> (peakX - 2);
    const unsigned int startY = static_cast<unsigned int> (peakY - 2);

    // sum the complete energy of the 3x3 area in fintpk
    double fintpk = 0.0;
    for (unsigned int m = 0; m < 3; m++) {
        for (unsigned int ki = 0; ki < 3; ki++) {
            fintpk += in[startX + ki + (startY + m) * res];
        }
    }

    double xd_ofs = 0.0;
    double yd_ofs = 0.0;
    for (unsigned int m = 0; m < 3; m++) {
        xd_ofs += in[startX + 0 + (startY + m) * res];
        xd_ofs += 2 * in[startX + 1 + (startY + m) * res];
        xd_ofs += 3 * in[startX + 2 + (startY + m) * res];
    }
    for (unsigned int ki = 0; ki < 3; ki++) {
        yd_ofs += in[startX + ki + (startY + 0) * res];
        yd_ofs += 2 * in[startX + ki + (startY + 1) * res];
        yd_ofs += 3 * in[startX + ki + (startY + 2) * res];
    }
    peakX += (xd_ofs / fintpk) - 2.5 - ((res - 1) / 2) -
             1.; // subtract (center(2) + group delay of interpolation filter(0.5))
    peakY += (yd_ofs / fintpk) - 2.5 - ((res - 1) / 2) - 1.;
}

void FMIRegistration::secureInterpolatePeak(const FMIReal *const in,
                                            const unsigned int &res, FMIReal &peakX, FMIReal &peakY) const {

    // if startX or startY get negative they will become really big (overflow)
    // and thus we just have to check for being bigger than resolution
    const unsigned int startX = static_cast<unsigned int> (peakX - 2);
    const unsigned int startY = static_cast<unsigned int> (peakY - 2);

    double fintpk = 0.0;
    for (unsigned int m = 0; m < 3; m++) {
        for (unsigned int ki = 0; ki < 3; ki++) {
            const unsigned int x = startX + ki;
            const unsigned int y = startY + m;
            if (x >= res || y >= res)
                continue;
            fintpk += in[x + y * res];
        }
    }
    double xd_ofs = 0.0;
    double yd_ofs = 0.0;
    for (unsigned int m = 0; m < 3; m++) {
        const unsigned int y = startY + m;
        if (y >= res)
            continue;
        unsigned int x = startX;
        if (x < res)
            xd_ofs += in[x + y * res];
        x++;
        if (x < res)
            xd_ofs += 2 * in[x + y * res];
        x++;
        if (x < res)
            xd_ofs += 3 * in[x + y * res];
    }
    for (unsigned int ki = 0; ki < 3; ki++) {
        const unsigned int x = startX + ki;
        if (x >= res)
            continue;
        unsigned int y = startY;
        if (y < res)
            yd_ofs += in[x + y * res];
        y++;
        if (y < res)
            yd_ofs += 2 * in[x + y * res];
        y++;
        if (y < res)
            yd_ofs += 3 * in[x + y * res];
    }
    peakX += (xd_ofs / fintpk) - 2.5 - ((res - 1) / 2) -
             1.; // subtract (center(2) + group delay of interpolation filter(0.5))
    peakY += (yd_ofs / fintpk) - 2.5 - ((res - 1) / 2) - 1.;
}

bool FMIRegistration::saveInverseFFT(std::string file_name, const FMIReal *inverseFft, const unsigned int &resolution) {
    if (inverseFft == 0) {
        ERR("FMIReal ptr is NULL!");
        return false;
    }
    ofstream out(file_name.c_str());
    const FMIReal *inFft;
    for (unsigned int x = 0; x <= resolution; x++) {
        inFft = inverseFft + x * resolution;
        for (unsigned int y = 0; y <= resolution; y++, inFft++) {
            out << *inFft << " ";
        }
        out << endl;
    }
    out.close();
    return true;
}

