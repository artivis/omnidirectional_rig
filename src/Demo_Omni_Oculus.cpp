#include "image_handler.h"
#include "omni_camera.h"
#include "fisheye.h"

#include <opencv2/core/core.hpp>

#include <iostream>
#include <string>


int main(int argc, char** argv){

    if (argc != 2) return -1;
    if (argv == NULL) return -1;

    int sampling_ratio = atof(argv[1]);
    std::string conf_path = argv[2]; //TODO check nb arg & val

    ros::init(argc,argv, "demo_omni_oculus");

    ros::NodeHandle nh;

    std::vector<std::string> path_yamls_cam;
    std::vector<std::string> topics_name;
    std::string maskCamera_1;
    std::string maskCamera_2;
    std::string extrinParam;

    const std::string cloudPtTopic = "/cloud_sphere";

    ros::Publisher pub_CloudSph = nh.advertise<sensor_msgs::PointCloud>(cloudPtTopic,0);

    path_yamls_cam.push_back(AddPath("etc/calib/Pal_intrinsicParam_cam1.yaml",conf_path));
    path_yamls_cam.push_back(AddPath("etc/calib/Pal_intrinsicParam_cam2.yaml",conf_path));

    extrinParam = AddPath("etc/calib/Pal_extrinsicParam.yaml",conf_path);

    maskCamera_1  = AddPath("etc/images/cam1/Img_mask1.jpg",conf_path);
    maskCamera_2  = AddPath("etc/images/cam2/Img_mask2.jpg",conf_path);

    topics_name.push_back("/left/image_raw");
    topics_name.push_back("/right/image_raw");

    OmniCamera omniSys(topics_name,path_yamls_cam,extrinParam);

    omniSys.DispParam();

    omniSys.camera_1->LoadMask(maskCamera_1);
    omniSys.camera_2->LoadMask(maskCamera_2);

    omniSys.DownSample(sampling_ratio);

    omniSys.DispParam();

    sensor_msgs::PointCloud ptsCld;

    double time;
    char exit;

    omniSys.PartiallyFillMess(ptsCld);

    do
    {
        omniSys.camera_1->ReadFrame();
        omniSys.camera_2->ReadFrame();

        time = (double)cv::getTickCount();

        omniSys.MessRGBSph(ptsCld);

        //std::cout << "time to comp sphere : "<<((double)cv::getTickCount() - time) / cv::getTickFrequency()<<std::endl<<std::endl;

        time = (double)cv::getTickCount();

        pub_CloudSph.publish(ptsCld);

        //std::cout << "time to publish sphere : "<<((double)cv::getTickCount() - time) / cv::getTickFrequency()<<std::endl<<std::endl;

        ros::spinOnce();

        exit = cv::waitKey(10);

        if(exit == 27) break;

    }while(1);

    return 0;
}
