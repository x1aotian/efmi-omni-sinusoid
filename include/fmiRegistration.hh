#ifndef _FMI_REGISTRATION__HH__
#define _FMI_REGISTRATION__HH__

#include "fmiImage.hh"
#include "fftMath.hh"

#include <vector>
#include <numeric>
#include <opencv2/opencv.hpp>

#define ONLY_TRANS 0

namespace jacobs_robotics {

    struct FMIResult;

    class FMIRegistration {
    public:

        FMIRegistration(const unsigned int &resolution, const unsigned int &polLogResolution,
                        const FMIReal &minFMISignalToNoise = 0., const FMIReal &minFMIsignalFactorOver2nd = 0.);

        FMIRegistration(const unsigned int &resolution,
                        const unsigned int &polLogResolution,
                        const unsigned int &angleReslution,
                        const FMIReal &minFMISignalToNoise = 0.,
                        const FMIReal &minFMIsignalFactorOver2nd = 0.);

        ~FMIRegistration();

        /**
         * Registers two FMIImages. Returns the results as a vector of FMIResult. Returns false if the registration failed.
         * @param first  The first image.
         * @param second The second image.
         * @param results The results
         * @return true if the registration was successfull, false if not
         */
        bool
        registerImages(FMIImage *first, FMIImage *second, std::vector<FMIResult> &results, bool over180deg = false);

        /**
         * Registers two FMIImages. Returns the results as a vector of FMIResult. Returns false if the registration failed.
         * @param first  The first image.
         * @param second The second image.
         * @param results The results
         * @param scalesline All the translation values along the peak line, later used for 1D fft
         * @param transline All the translation values along the peak line, later used for 1D fft
         * @return true if the registration was successful, false if not
         */
        bool
        registerMultiTrans(FMIImage *first, FMIImage *second, std::vector<FMIResult> &results, bool over180deg = false);

        bool
        registerMultiTransMultiScale(FMIImage *first, FMIImage *second,
                                     std::vector<FMIResult> &results,
                                     std::vector<double> &scalesline,
                                     std::vector<double> &transline,
                                     bool over180deg = false);

        bool
        registerMultiTransMultiScale_andMaxAngle(FMIImage *first,
                                                                  FMIImage *second,
                                                                  std::vector<FMIResult> &results,
                                                                  std::vector<double> &scalesline,
                                                                  std::vector<double> &transline,
                                                                  double &MaxAngle,
                                                                  bool over180deg = false);

        /**
        * Registers two FMIImages. Returns the results as a vector of FMIResult. Returns false if the registration failed.
        * @param first  The first image.
        * @param second The second image.
        * @param results The results
        * @param peaksline All the values along the peak line, later used for 1D fft
        * @return true if the registration was successful, false if not
        */
        bool registerUpToScale(FMIImage *first, FMIImage *second,
                               std::vector<FMIResult> &results, std::vector<double> &peaksline,
                               bool over180deg = false);

        /**
         * Registers two FMIImages. Returns the multiple results (multiple translation peaks) as a vector of FMIResult.
         * Returns false if the registration failed.
         * @param first  The first image.
         * @param second The second image.
         * @param results The results
         * @return true if the registration was successfull, false if not
         */
        bool
        registerImagesMul(FMIImage *first, FMIImage *second, std::vector<FMIResult> &results, bool over180deg = false);

        /**
        * Registers two FMIImages. Returns the results as a vector of FMIResult. Returns false if the registration failed.
        * @param first  The first image.
        * @param second The second image.
        * @param results The results
        * @param iFftFile store the data of inverse FFT
        * @return true if the registration was successfull, false if not
        */
        bool registerImages(FMIImage *first,
                            FMIImage *second,
                            std::vector<FMIResult> &results,
                            std::string iFftFile,
                            bool over180deg = false);

        void setMinFMISignalToNoise(FMIReal v) {
            minFMISignalToNoise = v;
        }

        void setMinFMIsignalFactorOver2nd(FMIReal v) {
            minFMIsignalFactorOver2nd = v;
        }

        void setUse2ndRotationPeak(bool val) { use2ndRotationPeak = val; }

        void setDumpDebugInfo(bool val) { dumpDebugInfo = val; }

    protected:

        //  INIT STUFF

        /** Creates matrices used during the FMI registration
     * - implemented in fmiRegistrationInit.cc
     */
        void init();

        /** Create 2D Spectral Window
     */
        void create2DSpectralWindow();

        /** Creates the polar logarithmic interpolation
      * lookup tables
     */
        void createPolarLogInterpolationTables();

        /** creates a 1D gaussian lookup table */
        void create1DGaussianWindow();

        /** creates a sector lookup table to divide the image **/
        void createRayTons();

        // END INIT STUFF

        // functions for preparing a single image...

        /** computes the reusable data (the fft) for the image
     * which is save in img->fft
     */
        void prepareFMIImage(FMIImage *img, bool needJustPolarLog);

        /** removes the constant offset of img->img and
     * applies the spectralWindow2D
     * saves the result in outputData (for which tempMatrix is used..)
     */
        void removeConstantOffsetApplySpectralWindow(FMIImageReal *in, FMIReal *outputData) const;

        /** Does a polar logarithmic sampling
      * As a little extra it already finds out the minium value
      * which is needed in the next step...
         * @param in in resolution
         * @param out in polarLogResolution
     */
        void polarLogarithmicSamling(const FMIReal *const in, FMIReal *const out, FMIReal &minimum) const;

        /** applies a logarithm and a 1D window ...
         * @param mat in polarLogResolution
         */
        void postProcessFMI(FMIReal *const mat, const FMIReal &minimum) const;

        void saveAngles(const ComplexFMIReal *const in, FMICompressedReal *const out, const unsigned int &res);

        // Now functions that use both images...

        /** matches the angles of two different images...
         */
        void
        phaseOnlyMatching(const ComplexFMIReal *const in1, const ComplexFMIReal *const in2, ComplexFMIReal *const out,
                          const unsigned int &res) const;

        void phaseOnlyMatching(const FMICompressedReal *const in1, const FMICompressedReal *const in2,
                               ComplexFMIReal *const out, const unsigned int &res, bool turn180deg) const;

        /** applies a [1 1; 1 1] filter(sum square pixels) -> group delay 0.5 in x,y direction
         * the resolution for the result is thus polarLogResolution+1
         */
        void filter(const FMIReal *const in, FMIReal *const out, const unsigned int &res) const;

        /** finds the dirac pulses (currently just one...)
         * @param in has polarLogResolution+1
         * @return if this step was successful
         */
        bool findDiracPulsesScaleRotation(const FMIReal *const in, std::vector<FMIResult> &results) const;

        bool findDiracPulsesMultiScaleRotation(const FMIReal *const in,
                                               std::vector<FMIResult> &results,
                                               std::vector<double> &peaksline) const;

        /** interpolates the given Peak Coordinates...
         * interpolate parameters within 3x3 area of determined dirac pulse
         * @param in
         * @param res
         * @param peakX
         * @param peakY
         */
        void interpolatePeak(const FMIReal *const in, const unsigned int &res, FMIReal &peakX, FMIReal &peakY) const;

        /** same as above except that is also checks for the borders and is thus slower...
         * the secure version is automatically used by interpolatePeak if necessary
         */
        void
        secureInterpolatePeak(const FMIReal *const in, const unsigned int &res, FMIReal &peakX, FMIReal &peakY) const;

        /** scales and rotates the image according to scale and rotRad
        */
        void scaleAndRotate(const FMIImageReal *const in, FMIImageReal *const out, const FMIReal &scale,
                            const FMIReal &rotRad) const;

//            void scaleAndRotateHeiko (const FMIImageReal * const in, FMIImageReal * const out, const FMIReal &scale, const FMIReal &rotRad) const;
        void scaleAndRotateCheck(const FMIImageReal *const in, FMIImageReal *const out, const FMIReal &scale,
                                 const FMIReal &rotRad) const;

        void scaleAndRotateInterp(const FMIImageReal *const in, FMIImageReal *const out, const FMIReal &scale,
                                  const FMIReal &rotRad) const;

        // Utility functions

    public:

        /** Finds the maximum value in in (which has size size)
         * returns the maximum value in maxVal and its
         * coordinate in maxCoord
         */
        static void findMax(const FMIReal *in, const unsigned int &size, FMIReal &maxVal, unsigned int &maxCoord);

        /** Finds the ray with the sum of maximum value in (which has size size)
         * returns the ray with the sum of maximum value in maxVal and its
         * coordinate in maxCoord
         */
         void findMaxRay(const FMIReal *in, const unsigned int &size, FMIReal &maxVal, unsigned int &maxCoord);


        /** Finds the ray with the sum of maximum value in (which has size size)
         * returns the ray with the sum of maximum value in maxVal and its
         * coordinate in maxCoord
         */
         std::vector<double>  findMaxRayAndSampling(const FMIReal *in,
                                                const unsigned int &size,
                                                FMIReal &maxVal,
                                                unsigned int &maxCoord);

        std::vector<double>  findMaxRayAndSampling_All(const FMIReal *in,
                                                   const unsigned int &size,
                                                   FMIReal &maxVal,
                                                   unsigned int &maxCoord,
                                                   double &maxAngle);

        /** Finds the maximum value in in (which has size size)
         * returns the maximum value in maxVal and its
         * coordinate in maxCoord
         */
        static void
        findMaxByAngle(const FMIReal *in, const unsigned int &resolution, FMIReal &maxVal, unsigned int &maxCoord);


        /** 3x3 Matrix Multiplication
         *
         */
        static void matMult3x3(const double in1[3][3], const double in2[3][3], double out[3][3]);

        /** Calculates the sum of values in the area between
         * peakX-halfSize < x <= peakX+halfSize  (the same for y...)
         * @param peakX the peak x coordinate
         * @param peakY the peak y coordinate
         * @param in input matrix
         * @param resolution the resolution of the input matrix
         * @param halfSize actual area size is 2*halfSize + 1 (the center)
         * @return the sum
         */

        static FMIReal calculateSum(const unsigned int &peakX,
                                    const unsigned int &peakY, const FMIReal *const in,
                                    const unsigned int &resolution, const unsigned int &halfSize);

        /**
         * @param peakX
         * @param peakY
         * @param in
         * @param resolution
         * @param signal the resulting signal
         * @param noise the resulting noise
         * @param cov the covariance matrix
         */
        static void calculateUncertainty(const unsigned int &peakX,
                                         const unsigned int &peakY, const FMIReal *const in,
                                         const unsigned int &resolution, FMIReal &signal, FMIReal &noise,
                                         FMIReal cov[2][2]);


        unsigned int getResolution() { return resolution; }

        bool saveInverseFFT(std::string file, const FMIReal *inFft, const unsigned int &resolution);

    protected:
        // Constants

        const unsigned int resolution; /// the resolution of the FMI algo
        const unsigned int squaredResolution; /// resolution * resolution

        const unsigned int polarLogResolution; /// the resolution for the polar logarithmic sampling
        const unsigned int squaredPolLogRes; /// polarLogResolution * polarLogResolution

        unsigned int angleResolution; // the angle resolution for finding the ray with maximum sum energy

        FFTMath fftMath;
        FFTMath fftMathPolar;

        FMIReal minFMISignalToNoise;
        FMIReal minFMIsignalFactorOver2nd;

        // Lookuptables created by init
        /**
         * in resolution
         */
        const FMIReal *spectralWindow2D;

        /** actually 4 matrices - in polarLogResolution
         */
        const FMIReal *polLogInterpolationMatrices; // actually 4 matrices

        /** actually 4 matrices - in polarLogResolution
         */
        const unsigned int *polLogInterpolationCoordinates; // actually 4 matrices

        /*
         * lookup table from image to ray tons
         */
        const int *rayMatrices;

        /** in polarLogResolution
         */
        const FMIReal *gaussianWindow1D;

        FMIReal *const tempMatrix1;
        FMIReal *const tempMatrix2;
        FMIReal *const tempMatrix3;
        //            FMIReal * const tempScaleMat;
        ComplexFMIReal *const tempComplexMatrix1;
        ComplexFMIReal *const tempComplexMatrix2;
        FMICompressedReal *const tempCompressedMatrix;


        bool use2ndRotationPeak;

        bool dumpDebugInfo;
    };

    struct FMIResult {
        FMIResult() : rotationRad(0.), scale(0.), transX(0.), transY(0.), signalFMI(0.), noiseFMI(0.), signalTrans(0.),
                      noiseTrans(0.), peakEnergy(0.), secondPeakEnergy(0.) {}

        FMIReal rotationRad;
        FMIReal scale;
        FMIReal transX, transY;

        FMIReal signalFMI, noiseFMI;
        FMIReal signalTrans, noiseTrans;
        FMIReal peakEnergy, secondPeakEnergy;
        FMIReal scaleEnergy;

        FMIReal covarianceMatrixFMI[2][2];
        FMIReal covarianceMatrixTrans[2][2];
    };
}

#endif
