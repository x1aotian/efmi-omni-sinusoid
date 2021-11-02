//#define DEBUG 1

#include "fftMath.hh"
#include <assert.h>

#include <math.h>
#include <iostream>

#ifdef FFT_MATH_GSL

#include <gsl/gsl_errno.h>

#endif

#ifdef FFT_MATH_FFTW
#include <fftw3.h>
#include <string.h>
#endif

using namespace jacobs_robotics;
using namespace std;

FMIReal ComplexFMIReal::multiply(const ComplexFMIReal &other) const {
	return real * other.real + imag * other.imag;
}
FMIReal ComplexFMIReal::magnitude() const {
	return sqrt(real * real + imag * imag);
}
FMIReal ComplexFMIReal::magnitudeWOsquareroot() const {
	return (real * real + imag * imag);
}

FFTMath::FFTMath(unsigned int _resolution) :
	tmpMatrix(new ComplexFMIReal[_resolution*_resolution]), resolution(_resolution), squaredRes(resolution * resolution), tmpMatrix2(new ComplexFMIReal[squaredRes]) {
	if (sizeof(FMIReal) != sizeof(double)) {
		cerr<<"fft curently only works with doubles as FMIReal!"<<endl;
		assert(0);
	}
#ifdef FFT_MATH_GSL
	gslWavetable = gsl_fft_complex_wavetable_alloc(resolution);
	gslWorkspace = gsl_fft_complex_workspace_alloc(resolution);
#endif
#ifdef FFT_MATH_FFTW
	    printf("Optimizing fftw plan for resolution %d (forward) - this might take some seconds...\n", resolution);
	    double *out = new double[resolution * resolution];
		fftw_plan_dft_r2c_2d(resolution, resolution, (double*)out, (double (*)[2])tmpMatrix, FFTW_PATIENT);
		delete [] out;
	    printf("Optimizing fftw plan for resolution %d (backward) - this might take some seconds...\n", resolution);
	    inversePlan = fftw_plan_dft_2d(resolution, resolution, (double (*)[2])tmpMatrix, (double (*)[2])tmpMatrix, FFTW_BACKWARD, FFTW_PATIENT);
	    printf("Done optimizing fftw plan.\n");
#endif
}

FFTMath::~FFTMath() {
	delete [] tmpMatrix;
	delete [] tmpMatrix2;
#ifdef FFT_MATH_GSL
	gsl_fft_complex_wavetable_free(gslWavetable);
	gslWavetable = 0;
	gsl_fft_complex_workspace_free(gslWorkspace);
	gslWorkspace = 0;
#endif
#ifdef FFT_MATH_FFTW
	fftw_destroy_plan(inversePlan);
#endif
}

void FFTMath::fft2D(const FMIReal * const in, ComplexFMIReal * const out) const {

#ifdef FFT_MATH_FFTW

	//    printf("Creating 2d fftw plan... resolution %d\n", resolution);
	fftw_plan plan1 = fftw_plan_dft_r2c_2d(resolution, resolution, (double*)out, (double (*)[2])tmpMatrix2, FFTW_PATIENT);

	// copy the input data...
	memcpy(out, in, sizeof(FMIReal)*squaredRes);

	fftw_execute(plan1);

	fftw_destroy_plan(plan1);

	// copy symmetrical data...
	const unsigned int halfResolution = resolution * 0.5;
	ComplexFMIReal * outP = out;
	for(unsigned int y = 0; y < resolution; y++) {
		outP = out + y * resolution + halfResolution;
		unsigned mirrorY = resolution - y;
		if(y == 0) mirrorY = y;
		ComplexFMIReal * const endTmpP = tmpMatrix2 + mirrorY * (halfResolution+1);
		ComplexFMIReal * tmpP = endTmpP + halfResolution;
		for(; tmpP >= endTmpP; tmpP--, outP++) {
			*outP = *tmpP;
			outP->imag *= -1;
		}
	}
	// copy the data...
	for(unsigned int y = 0; y < resolution; y++) {
		memcpy(out+y*resolution, tmpMatrix2 + y * (halfResolution + 1), sizeof(ComplexFMIReal) * (halfResolution + 1) );
	}

#endif

#ifdef FFT_MATH_GSL
	// copy in data to the complex data structure
	ComplexFMIReal * outPtr = out;
	const ComplexFMIReal * endPtr = out + squaredRes;
	const FMIReal * inPtr = in;
	for(; outPtr < endPtr; outPtr++, inPtr++) {
		outPtr->real = *inPtr;
		outPtr->imag = 0.;
	}

	// transform rows
	outPtr = out;
	for(; outPtr < endPtr; ) {
		gsl_fft_complex_forward( reinterpret_cast<double*>(outPtr), 1, resolution, gslWavetable, gslWorkspace);
		outPtr += resolution;
	}

	// transform columns
	outPtr = out;
	endPtr = out + resolution;
	for(; outPtr < endPtr; outPtr++) {
		gsl_fft_complex_forward( reinterpret_cast<double*>(outPtr), resolution, resolution, gslWavetable, gslWorkspace);
	}
#endif

}

void FFTMath::inverseFft2D(){
#ifdef FFT_MATH_GSL
	inverseFft2D(tmpMatrix);
	return;
#endif

#ifdef FFT_MATH_FFTW
	fftw_execute(inversePlan);
#endif
}

void FFTMath::inverseFft2D(ComplexFMIReal * const mat) const {

#ifdef FFT_MATH_GSL

	ComplexFMIReal * matP = mat;
	const ComplexFMIReal * endP = mat + squaredRes;

	// transform rows
	for(; matP < endP; ) {
		gsl_fft_complex_inverse( reinterpret_cast<double*>(matP), 1, resolution, gslWavetable, gslWorkspace);
		matP += resolution;
	}
	//    for (ki = 0; ki < NN; ki++){
	//        gsl_fft_complex_inverse((datamat + (ki*2*NN)), 1, NN, WAVETABLE, WORKSPACE); }

	// transform columns (stride = NN)
	matP = mat;
	endP = mat + resolution;
	for(; matP < endP; matP++ ) {
		gsl_fft_complex_inverse( reinterpret_cast<double*>(matP), resolution, resolution, gslWavetable, gslWorkspace);
	}
	// for (ki = 0; ki < NN; ki++){
	//    gsl_fft_complex_inverse((datamat + (ki*2)), NN, NN, WAVETABLE, WORKSPACE); }
#endif

#ifdef FFT_MATH_FFTW
	//    printf("Creating 2d fftw plan... resolution %d\n", resolution);
	fftw_plan plan1 = fftw_plan_dft_2d(resolution, resolution, (double (*)[2])tmpMatrix2, (double (*)[2])tmpMatrix2, FFTW_BACKWARD, FFTW_PATIENT);
	//    printf("doing memcpy...\n");
	memcpy(tmpMatrix2, mat, sizeof(ComplexFMIReal)*squaredRes);
	//    printf("executing plan\n");
	fftw_execute(plan1);
	memcpy(mat, tmpMatrix2, sizeof(ComplexFMIReal)*squaredRes);
	//    printf("destroying plan\n");
	fftw_destroy_plan(plan1);
	//    printf("finished\n");

#endif
}

void FFTMath::spacialMagnitudeAndFFTShift(const ComplexFMIReal * const in,
		FMIReal * const out) const {

	const unsigned int halfRes = static_cast<unsigned int> (resolution * 0.5);
	const unsigned int halfY = halfRes * resolution;

	FMIReal * outp = out;
	const ComplexFMIReal * inp = in;
	const ComplexFMIReal * endp;
	for (unsigned int y = 0; y < halfRes; y++) {
		endp = inp + halfRes;
		for (; inp < endp; outp++, inp++) { // this is x
			// 1. quadrant: [x, y] = [x+1/2, y+1/2]->magnitude()
			*outp = (inp + halfRes + halfY)->magnitude();
			// 2. quadrant: [x+1/2, y] = [x, y+1/2]->magnitude()
			*(outp + halfRes) = (inp + halfY)->magnitude();
			// 3. quadrant: [x, y+1/2] = [x+1/2, y]->magnitude()
			*(outp + halfY) = (inp + halfRes)->magnitude();
			// 1. quadrant: [x+1/2, y+1/2] = [x, y]->magnitude()
			*(outp + halfRes + halfY) = inp->magnitude();
		}
		outp += halfRes;
		inp += halfRes;
	}
}

