#include "fmiImage.hh"

#include <QImage>
#include <fstream>

using namespace std;

#include <stdio.h>
#define LOC  fprintf(stderr, "[in %s@line %d] ", __FILE__, __LINE__);
#define ERR(...) do{LOC fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n");} while(0)

namespace jacobs_robotics{

FMIImage::FMIImage(): img(0), phase(0), polarLogPhase(0), resolution(0), squaredRes(0){
}

FMIImage::FMIImage(const unsigned int &_resolution): resolution(_resolution){
	create(resolution);
}

void FMIImage::create(const unsigned int &_resolution){
	resolution = _resolution;
    squaredRes = resolution*resolution;
    img = new FMIImageReal[squaredRes];
    phase = 0;
    polarLogPhase = 0;
}

FMIImage::~FMIImage(){
    delete [] img;
    img = 0;
    delete [] phase;
    phase = 0;
    delete [] polarLogPhase;
    polarLogPhase = 0;
}

void FMIImage::saveSomeMemory(){
    delete [] phase;
    phase = 0;
}

void FMIImage::saveAllMemory(){
    delete [] phase;
    phase = 0;
    delete [] polarLogPhase;
    polarLogPhase = 0;
}

FMIImage * loadFMIImage(std::string file){
    QImage img(file.c_str());
    return fromQImage(img);
}

FMIImage * fromQImage(QImage &img){
    if(img.isNull()) return 0;
    if(img.width() != img.height()){
        ERR(" Error - width (%d) and height (%d) differ!", img.width(), img.height());
        return 0;
    }
    if(img.depth()==32){
        FMIImage *rtn = new FMIImage(img.width());
        QRgb* pQimg = (QRgb*) img.bits();
        FMIImageReal * ptr = rtn->img;
        FMIImageReal * end = ptr + rtn->squaredRes;
        for( ; ptr < end; ptr++, pQimg++){
            (*ptr) = FMIImageReal( qRed(*pQimg) + qGreen(*pQimg) + qBlue(*pQimg) ) / ( 256 * 3);
        }
        return rtn;
    }else if(img.depth()==8){
        FMIImage *rtn = new FMIImage(img.width());
        uchar* pQimg = img.bits();
        FMIImageReal * ptr = rtn->img;
        FMIImageReal * end = ptr + rtn->squaredRes;
        for( ; ptr < end; ptr++, pQimg++){
            (*ptr) = FMIImageReal( *pQimg ) / ( 256 * 3);
        }
        return rtn;
    }else{
        ERR("Depth of %d bits per pixel currently not supported!", img.depth());
        return 0;
    }
}

bool saveFMIImage(std::string file, const FMIImage * const img){
    if(img == 0){
        ERR("Image ptr is NULL!");
        return false;
    }
    if(img->img == 0){
        ERR("Image is NULL!");
        return false;
    }
    return saveFMIRealImage(file, img->img, img->resolution);
}

bool saveFMIRealImage(std::string file, const double * const img, const unsigned int &resolution, const unsigned int &skip, const double &multy){
    if(img == 0){
        ERR("FMIReal ptr is NULL!");
        return false;
    }
    QImage qImage(resolution, resolution, QImage::Format_RGB32);
    QRgb* pQimg = (QRgb*) qImage.bits();
    const double * ptr = img;
    const double * const end = ptr + resolution * resolution * skip;
    int value;
    for( ; ptr < end; pQimg++){
        value = static_cast<int>((*ptr) * multy);
        *pQimg = qRgb(value, value, value);
        ptr += skip;
    }
    return qImage.save(file.c_str());
}

bool saveFMIRealImage(std::string file, const float * const img, const unsigned int &resolution, const unsigned int &skip, const double &multy){
    if(img == 0){
        ERR("FMIReal ptr is NULL!");
        return false;
    }
    QImage qImage(resolution, resolution, QImage::Format_RGB32);
    QRgb* pQimg = (QRgb*) qImage.bits();
    const float * ptr = img;
    const float * const end = ptr + resolution * resolution * skip;
    int value;
    for( ; ptr < end; pQimg++){
        value = static_cast<int>((*ptr) * multy);
        *pQimg = qRgb(value, value, value);
        ptr += skip;
    }
    return qImage.save(file.c_str());
}

bool dumpValues(std::string file, const double * const img, const unsigned int &resolution, const unsigned int &skip){
    if(img == 0){
        ERR("FMIReal ptr is NULL!");
        return false;
    }
    ofstream out(file.c_str());
    const double * ptr = img;
    const double * const end = ptr + resolution * resolution * skip;
    unsigned int county = 0;
    for( ; ptr < end; ){
        out<<*ptr<<" ";
        ptr += skip;
        if((++county % resolution) == 0) out<<endl;
    }
    out<<endl;
    out.close();
    return true;
}

bool dumpValues(std::string file, const float * const img, const unsigned int &resolution, const unsigned int &skip){
    if(img == 0){
        ERR("FMIReal ptr is NULL!");
        return false;
    }
    ofstream out(file.c_str());
    const float * ptr = img;
    const float * const end = ptr + resolution * resolution * skip;
    for( ; ptr < end; ){
        out<<*ptr<<endl;
        ptr += skip;
    }
    out<<endl;
    out.close();
    return true;
}

}

