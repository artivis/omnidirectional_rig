#ifndef kmkmSOFT_H
#define kmkmSOFT_H

#include <opencv2/core/core.hpp>

namespace SOFTWRAPP
{

    typedef std::vector< std::vector< std::complex<double> > > harmCoeff;

    void wrapSphCorr2(int bw, const cv::Mat &sphPattern, const cv::Mat &sphSignal, cv::Vec3f &EulerAngle);

    void wrapSphHarm(int bw, const cv::Mat &signal, harmCoeff &coeff);

    void harmDesc(const harmCoeff &coeff, std::vector< std::complex<double> > &descriptor);

    void harmDesc(const harmCoeff &coeff, std::vector< double > &descriptor);

    void sampSphFct(int bw,const cv::Mat &pano, cv::Mat &sampFct);

    void warpS2Rotate(int bw, cv::Vec3f, const harmCoeff&, harmCoeff&);

    void dispSphHarm(const harmCoeff &);

    void dispRotEst(const cv::Vec3f&);

    void saveSphHarm(const std::string&, const harmCoeff&);

}

#endif // SOFT_H
