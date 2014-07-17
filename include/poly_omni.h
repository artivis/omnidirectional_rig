#ifndef OMNI_CAMERA_H
#define OMNI_CAMERA_H

#include <omni_camera.h>
#include <feature_extractor.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/photo.hpp"

#include <sensor_msgs/PointCloud.h>

#include <stdio.h>

class PolyOmniCamera {

    ros::NodeHandle nh_param;

    public :

        OmniCamera *camera_1;
        OmniCamera *camera_2;

        PolyOmniCamera(const std::string &);
        PolyOmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &paramPath);

        PolyOmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &cameraParamPath,
                   const std::string &extrinPath);

        ~PolyOmniCamera();

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

        void ReadFrame();

        bool IsInit();

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

        bool _init;

        cv::Size _panoSize;

        cv::Mat _pano;

        cv::Mat _LUTsphere;
        cv::Mat _LUT_wrap_im;
        cv::Mat _LUTsph_im;

        bool _isSampled;
        int _sampling_ratio;

        int _ind_LUTsph;
};



#endif // OMNI_CAMERA_H



