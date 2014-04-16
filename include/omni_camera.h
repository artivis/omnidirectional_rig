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

//        OmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &cameraParamPath,
//                   const std::string &extrinPath, const std::vector<std::string> &LUTpath,
//                   const std::vector<std::string> &LUTtype = std::vector<std::string>);

        ~OmniCamera();

        //void InitCamera(int cameraNum, const std::string &topicName, const std::string &paramPath);

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

        int GetRGBSphSamp();

        void SetExtrin(const cv::Mat &);

        void SetRGBSphSamp(int);

    private :

        cv::Mat _extrin;

        bool _init;

        cv::Size _panoSize;

        cv::Mat _pano;

        cv::Mat _LUTsphere;
        cv::Mat _LUT_wrap_im;

        int _RGBSphSamp;

};



#endif // OMNI_CAMERA_H



