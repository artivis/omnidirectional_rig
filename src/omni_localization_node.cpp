#include "image_handler.h"

#include "omni_camera.h"

#include "fisheye.h"

#include <opencv2/core/core.hpp>

#include <iostream>
#include <string>


int main(int argc, char** argv){

    ros::init(argc,argv, "omni_localization");

    cv::Mat cur_im_1;
    cv::Mat cur_im_2;

    std::vector<std::string> path_yamls_cam;
    std::vector<std::string> topics_name;
    std::string extrinParam;
    std::string LUTcam1;
    std::string LUTcam2;

    path_yamls_cam.push_back("etc/intrinsicParam_cam1.yaml");
    path_yamls_cam.push_back("etc/intrinsicParam_cam1.yaml");

    topics_name.push_back("/whatever");
    topics_name.push_back("/whatever");

    extrinParam = "etc/extrinsicParam.yaml";

    LUTcam1 = "etc/LUT_cam1.yaml";
    LUTcam2 = "etc/LUT_cam2.yaml";

    OmniCamera omniSys(topics_name,path_yamls_cam);

    omniSys.camera_1->DispParam();
    omniSys.camera_2->DispParam();

    omniSys.camera_1->readImage("etc/images/cam1/cal_seq3_cam1_1.bmp",cur_im_1);
    omniSys.camera_2->readImage("etc/images/cam2/cal_seq3_cam2_1.bmp",cur_im_2);

//    omniSys.camera_1->LoadLUT(LUTcam1,"sphere");





    cv::namedWindow("imshow_cam1",cv::WINDOW_NORMAL);
    cv::imshow("imshow_cam1",cur_im_1);

    cv::namedWindow("imshow_cam2",cv::WINDOW_NORMAL);
    cv::imshow("imshow_cam2",cur_im_2);


    cv::waitKey(0);

    return 0;

}
