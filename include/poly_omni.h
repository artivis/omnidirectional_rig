#ifndef OMNI_CAMERA_H
#define OMNI_CAMERA_H

#include <omni_camera.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/photo.hpp"

#include <sensor_msgs/PointCloud.h>
#include <boost/utility.hpp>

#include <stdio.h>

class OmniCameraRig : boost::noncopyable
{
public :

        boost::shared_ptr<OmniCamera> camera_1;
        boost::shared_ptr<OmniCamera> camera_2;

        OmniCameraRig();
        OmniCameraRig(const std::vector<std::string> &paramPath);
        OmniCameraRig(const std::vector<std::string> &cameraParamPath, const std::string &extrinPath);

        ~OmniCameraRig();

        bool LoadCalibration(const std::string&);

        void DispParam();

        void MergeLUTWrap(bool heal = false);

        void MergeLUTHeal();

        void MergeLUTSph();

        void RescaleWrapLUT(cv::Size size = cv::Size(1200,400));

        void StitchImage(bool INPAIN_FLAG = 0);

        void SaveImage(const std::string &filename = "panoramicImage.jpg");

        void ApplyBaseline();

        void MessRGBSph(sensor_msgs::PointCloud &);

        void PartiallyFillMess(sensor_msgs::PointCloud &);

        void setImages(const std::vector<cv::Mat>&);

        bool IsInit() {return this->_init;}

        cv::Mat GetExtrin();
        cv::Mat GetPano();
        cv::Mat GetLUT();

        void SetExtrin(const cv::Mat &);
        void SetPanoSize(cv::Size &);
        void SetPanoSize(int,int);

        void DownSample(int sampling_ratio = 1);

        void Sph2Pano();

        void Sph2HealPano();

        void SampSphFct(cv::Mat&, int bandwidth = 64);

private :

        void Rotate90roll();

        cv::Mat _extrin;
        cv::Size _panoSize;
        cv::Mat _pano;
        cv::Mat _LUTsphere;
        cv::Mat _LUT_wrap_im;
        cv::Mat _LUTsph_im;

        bool _init;
        bool _isSampled;

        int _sampling_ratio;
        int _ind_LUTsph;
};


#endif // OMNI_CAMERA_H



