#ifndef kmkmSOFT_H
#define kmkmSOFT_H

#include <opencv2/core/core.hpp>

//#warning "HOOOOOOOOOOOOOOOOOOOOO"

//typedef std::vector< std::vector< std::complex<double> > > harmCoeff;


namespace SOFTWRAPP
{

    typedef std::vector< std::vector< std::complex<double> > > harmCoeff;

    void WrapSphCorr2(int bw, const cv::Mat &sphPattern, const cv::Mat &sphSignal, cv::Vec3f &EulerAngle);

    void WrapSphHarm(int bw, const cv::Mat &signal, harmCoeff &coeff);

    void HarmDesc(const harmCoeff &coeff, std::vector< std::complex<double> > &descriptor);

    void HarmDesc(const harmCoeff &coeff, std::vector< double > &descriptor);

    void SampSphFct(int bw,const cv::Mat &pano, cv::Mat &sampFct);

    void WarpS2Rotate(int bw, cv::Vec3f, const harmCoeff&, harmCoeff&);

    void DispSphHarm(const harmCoeff &);

    void DispRotEst(const cv::Vec3f&);

    void SaveSphHarm(const std::string&, const harmCoeff&);

}

#endif // SOFT_H
