#ifndef OMNI_CAMERA_H
#define OMNI_CAMERA_H

#include <fisheye.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/photo.hpp"

#include <sensor_msgs/PointCloud.h>

#include <stdio.h>

class OmniCamera {

    ros::NodeHandle nh_param;

    public :

        FishEye *camera_1;
        FishEye *camera_2;

        OmniCamera();
        OmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &paramPath);

        OmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &cameraParamPath,
                   const std::string &extrinPath);

        ~OmniCamera();

        bool LoadCalibration(const std::string&);

        void DispParam();

        void LoadLUT(const std::vector<std::string> &, const std::vector<std::string> &);

        void MergeLUTWrap(cv::Size size = cv::Size(1200,400));

        void MergeLUTSph();

        void RescaleWrapLUT(cv::Size size = cv::Size(1200,400));

        void StitchImage(int INPAIN_FLAG = 0);

        void SaveImage(const std::string &filename = "panoramicImage.jpg");

        void ApplyBaseline();

        void MessRGBSph(sensor_msgs::PointCloud &);

        void PartiallyFillMess(sensor_msgs::PointCloud &);

        void Rotate90roll();

        bool IsInit();

        cv::Mat GetExtrin();

        cv::Mat GetPano();

        cv::Mat GetLUT();

        void SetExtrin(const cv::Mat &);

        void DownSample(int sampling_ratio = 1);

        int CompLUTsampSph2Im(int bandwidth = 64);

        void SampSphFct( cv::Mat&, int bandwidth = 64);

        void GetSphSampGrid(int bandwidth, cv::Mat &pts, bool ishemi = false);

        void GetHemiSphSampGrid(cv::Mat &pts,int bandwidth);

    private :

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



