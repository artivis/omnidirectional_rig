#ifndef kmkmSOFT_H
#define kmkmSOFT_H

#include <opencv2/core/core.hpp>

//#warning "HOOOOOOOOOOOOOOOOOOOOO"


class SOFTWRAPP
{

public:

    SOFTWRAPP();

    static void SphericalHarmonics(int, const cv::Mat&, std::vector< std::vector< std::complex<double> > >&);

    static void SphericalHarmonics(int, const cv::Mat&, std::vector< std::complex<double> >&);

    static void CorrSO3(int, int, const std::vector< std::vector< std::complex<double> > >&pattern,
                                  const std::vector< std::vector< std::complex<double> > >&signal,
                                  cv::Vec3f &rotation, int degLim = 0);

    static void DispSphHarm(const std::vector< std::vector< std::complex<double> > > &);

    static void DispRotEst(const cv::Vec3f&);

};

#endif // SOFT_H
