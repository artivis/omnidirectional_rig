#include "image_handler.h"
#include "omni_camera.h"
#include "fisheye.h"

#include <opencv2/core/core.hpp>

#include <iostream>
#include <string>


int main(int argc, char** argv){

    ros::init(argc,argv, "demo_omni_oculus");

    const std::string cloudPtTopic = "/cloud_sphere";
    ros::NodeHandle nh;

    ros::Publisher pub_CloudSph = nh.advertise<sensor_msgs::PointCloud>(cloudPtTopic,0);

    std::vector<std::string> path_yamls_cam;
    std::vector<std::string> topics_name;
    std::string maskCamera_1;
    std::string maskCamera_2;
    std::string extrinParam;

    path_yamls_cam.push_back("etc/calib/Pal_intrinsicParam_cam1.yaml");
    path_yamls_cam.push_back("etc/calib/Pal_intrinsicParam_cam2.yaml");

    maskCamera_1  = "etc/images/cam1/Img_mask1.jpg";
    maskCamera_2  = "etc/images/cam2/Img_mask2.jpg";

    topics_name.push_back("/left/image_raw");
    topics_name.push_back("/right/image_raw");

    extrinParam = "etc/calib/Pal_extrinsicParam.yaml";

    OmniCamera omniSys(topics_name,path_yamls_cam,extrinParam);

    omniSys.DispParam();

    omniSys.camera_1->LoadMask(maskCamera_1);
    omniSys.camera_2->LoadMask(maskCamera_2);

    omniSys.DownSample(4);

    omniSys.DispParam();

    sensor_msgs::PointCloud ptsCld;

    double time;

    omniSys.PartiallyFillMess(ptsCld);

    do
    {
        omniSys.camera_1->ReadFrame();
        omniSys.camera_2->ReadFrame();

        time = (double)cv::getTickCount();

        omniSys.MessRGBSph(ptsCld);

        std::cout << "time to comp sphere : "<<((double)cv::getTickCount() - time) / cv::getTickFrequency()<<std::endl<<std::endl;

        time = (double)cv::getTickCount();

        pub_CloudSph.publish(ptsCld);

        std::cout << "time to publish sphere : "<<((double)cv::getTickCount() - time) / cv::getTickFrequency()<<std::endl<<std::endl;

        ros::spinOnce();

    }while(true);

    return 0;
}
