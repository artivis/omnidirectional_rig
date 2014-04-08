#ifndef OMNI_CAMERA_H
#define OMNI_CAMERA_H

//#include <vector>
//#include <string>
//#include <opencv2/core/core.hpp>

//#include <ros/ros.h>

//#include <image_handler.h>

#include <fisheye.h>

//using namespace cv;
//using namespace std;


class OmniCamera {

    ros::NodeHandle nh_param;

    public :

        OmniCamera();
        OmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &paramPath);
        OmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &cameraParamPath, const std::string &extrinPath);

        void InitCamera(int cameraNum, const std::string &topicName, const std::string &paramPath);

        bool LoadCalibration(const std::string&);

        void DispParam();

        bool IsInit();

        FishEye *camera_1;
        FishEye *camera_2;

    private :

        cv::Mat baseline;

        bool _init;

};



#endif // OMNI_CAMERA_H



