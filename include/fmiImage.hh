#ifndef _FMI_IMAGE__HH__
#define _FMI_IMAGE__HH__

#include <string>
#include <QImage>

namespace jacobs_robotics{

	typedef float FMICompressedReal;
	typedef float FMIImageReal;

    /** An FMI image is a square matrix of FMIReal values
     * plus the reusable ffts
     * The polarLogPhase is in the polarLogresolution and not in resolution!
     * 
     * The idea behind the saving of the phases:
     */
    class FMIImage{
        public:
            FMIImage();              /// creates an empty image
            FMIImage(const unsigned int &resolution); /// initializes the arrays
            ~FMIImage();             /// deletes the arrays
            void create(const unsigned int &_resolution); /// creates the img array

            FMIImageReal * img;           /// the original intensity image
            FMICompressedReal *phase; /// phase of the image - reused with a rotated & scaled image for translation
            FMICompressedReal *polarLogPhase; /// phase of the polar logarithmic image - reused when finding out rotation & scale

            unsigned int resolution; /// the resolution of this image
            unsigned int squaredRes; /// resolution * resolution

            // frees the phase pointer
            void saveSomeMemory();
            // frees phase and polarLogPhase
            void saveAllMemory();

        private:
            FMIImage(const FMIImage&){}
    };



        FMIImage * fromQImage(QImage &img);
        FMIImage * loadFMIImage(std::string file);
        bool saveFMIImage(std::string file, const FMIImage * const img);
        bool saveFMIRealImage(std::string file, const double * const img, const unsigned int &resolution, const unsigned int &skip = 1, const double &multy = 256.);
        bool saveFMIRealImage(std::string file, const float * const img, const unsigned int &resolution, const unsigned int &skip = 1, const double &multy = 256.);
        bool dumpValues(std::string file, const double * const img, const unsigned int &resolution, const unsigned int &skip = 1);
        bool dumpValues(std::string file, const float * const img, const unsigned int &resolution, const unsigned int &skip = 1);
    
}

#endif
