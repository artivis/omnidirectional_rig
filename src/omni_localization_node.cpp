#include "image_handler.h"

#include "omni_camera.h"

#include "fisheye.h"

#include <opencv2/core/core.hpp>

#include <iostream>
#include <string>


int main(int argc, char** argv){

    ros::init(argc,argv, "omni_localization");

    std::vector<std::string> path_yamls_cam;
    std::vector<std::string> topics_name;
    std::string maskCamera_1;
    std::string maskCamera_2;
    std::string extrinParam;
    std::string LUTcam1;
    std::string LUTcam2;

    path_yamls_cam.push_back("etc/intrinsicParam_cam1.yaml");
    path_yamls_cam.push_back("etc/intrinsicParam_cam1.yaml");

    maskCamera_1  = "etc/images/cam1/Img_mask1.jpg";
    maskCamera_2  = "etc/images/cam2/Img_mask2.jpg";

    topics_name.push_back("/whatever");
    topics_name.push_back("/whatever");

    extrinParam = "etc/extrinsicParam.yaml";

    LUTcam1 = "etc/LUT_txt_cam1.txt";
//    LUTcam1 = "etc/qqa.txt";
    LUTcam2 = "etc/LUT_short_cam2.txt";

    OmniCamera omniSys(topics_name,path_yamls_cam,extrinParam);

    omniSys.DispParam();

    omniSys.camera_1->readImage("etc/images/cam1/cal_seq3_cam1_1.bmp");
    omniSys.camera_2->readImage("etc/images/cam2/cal_seq3_cam2_1.bmp");

    omniSys.camera_1->LoadMask(maskCamera_1);
    omniSys.camera_2->LoadMask(maskCamera_2);

    omniSys.camera_1->LoadLUT(LUTcam1,"Sphere");

    std::cout<<omniSys.camera_1->GetLUT().size();

    std::cout<<omniSys.camera_1->GetLUT();

    cv::namedWindow("imshow_cam1",cv::WINDOW_NORMAL);
    cv::imshow("imshow_cam1",omniSys.camera_1->getImage());

    cv::namedWindow("imshow_cam2",cv::WINDOW_NORMAL);
    cv::imshow("imshow_cam2",omniSys.camera_2->getImage());


    cv::waitKey(0);

    cv::namedWindow("imshow_cam1",cv::WINDOW_NORMAL);
    cv::imshow("imshow_cam1",omniSys.camera_1->GetMask()*255);

    cv::namedWindow("imshow_cam2",cv::WINDOW_NORMAL);
    cv::imshow("imshow_cam2",omniSys.camera_2->GetMask()*255);

    cv::waitKey(0);

    return 0;

}
