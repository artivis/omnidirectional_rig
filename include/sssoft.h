#ifndef kmkmSOFT_H
#define kmkmSOFT_H

#include <opencv2/core/core.hpp>

#define NORM( x ) ( (x[0])*(x[0]) + (x[1])*(x[1]) )


//#warning "HOOOOOOOOOOOOOOOOOOOOO"


class SOFTWRAPP
{

public:

    SOFTWRAPP();

    static void SphericalHarmonics(int, const cv::Mat&, std::vector< std::vector< std::complex<double> > >&);

    static void SphericalHarmonics(int, const cv::Mat&, std::vector< std::complex<double> >&);

    static void CorrSO3(int, int, std::vector< std::complex<double> >&, std::vector< std::complex<double> >&, cv::Vec3f&);

//    void

};

#endif // SOFT_H
