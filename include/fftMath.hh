#ifndef __FFT_MATH__HH__
#define __FFT_MATH__HH__

#define FFT_MATH_GSL
//#define FFT_MATH_FFTW


#ifdef FFT_MATH_GSL
  #ifdef FFT_MATH_FFTW
    #error "You cannot define both FFT_MATH_GSL and FFT_MATH_FFTW"
  #endif
#endif

#ifndef FFT_MATH_GSL
  #ifndef FFT_MATH_FFTW
    #error "You have to define either FFT_MATH_GSL or FFT_MATH_FFTW"
  #endif
#endif

#ifdef FFT_MATH_GSL
  #include <gsl/gsl_fft_complex.h>
#endif

#ifdef FFT_MATH_FFTW
#include <fftw3.h>
#endif

namespace jacobs_robotics{

    typedef double FMIReal;

    class ComplexFMIReal{
        public:
        FMIReal real;
        FMIReal imag;

        inline FMIReal multiply(const ComplexFMIReal &other) const;
        inline FMIReal magnitude() const;
        inline FMIReal magnitudeWOsquareroot() const;
    };

    class FFTMath{
        public:
            FFTMath(unsigned int resolution);
            ~FFTMath();

            void fft2D(const FMIReal * const in, ComplexFMIReal * const out) const;

            void inverseFft2D(ComplexFMIReal * const mat) const;

            /**
             * Calculates the inverse 2D fft. It uses the variables tmpMatrix for in and output!
             */
            void inverseFft2D();

            void spacialMagnitudeAndFFTShift(const ComplexFMIReal * const in, FMIReal * const out) const;

            ComplexFMIReal * const tmpMatrix;

        private:
            const unsigned int resolution;
            const unsigned int squaredRes;

            ComplexFMIReal * const tmpMatrix2;

#ifdef FFT_MATH_GSL
            gsl_fft_complex_wavetable *gslWavetable;
            gsl_fft_complex_workspace *gslWorkspace;
#endif
#ifdef FFT_MATH_FFTW
            fftw_plan inversePlan;
#endif
            // forbit default and copy constructor
            FFTMath():tmpMatrix(0), resolution(0), squaredRes(0), tmpMatrix2(0){}
            FFTMath(const FFTMath&):tmpMatrix(0), resolution(0), squaredRes(0), tmpMatrix2(0){}

    };
}

#endif
