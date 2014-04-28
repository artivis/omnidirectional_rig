#ifndef kmkmSOFT_H
#define kmkmSOFT_H

#include <opencv2/core/core.hpp>

//#warning "HOOOOOOOOOOOOOOOOOOOOO"

typedef std::vector< std::vector< std::complex<double> > > harmCoeff;


namespace SOFTWRAPP
{

    void WrapSphCorr2(int bw, const cv::Mat &sphPattern, const cv::Mat &sphSignal, cv::Vec3f &EulerAngle);

    void DispSphHarm(const std::vector< std::vector< std::complex<double> > > &);

    void DispRotEst(const cv::Vec3f&);

}

#endif // SOFT_H
