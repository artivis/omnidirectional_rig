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
    std::vector<std::string> LUTs_file;
    std::vector<std::string> LUTs_type;
    std::string im_cam1;
    std::string im_cam2;
    std::string maskCamera_1;
    std::string maskCamera_2;
    std::string extrinParam;

    path_yamls_cam.push_back("etc/calib/intrinsicParam_cam1.yaml");
    path_yamls_cam.push_back("etc/calib/intrinsicParam_cam1.yaml");

    im_cam1 = "etc/images/cam1/cal_seq3_cam1_1.bmp";
    im_cam2 = "etc/images/cam2/cal_seq3_cam2_1.bmp";

    maskCamera_1  = "etc/images/cam1/Img_mask1.jpg";
    maskCamera_2  = "etc/images/cam2/Img_mask2.jpg";

    topics_name.push_back("/whatever");
    topics_name.push_back("/whatever");

    extrinParam = "etc/calib/extrinsicParam.yaml";

    LUTs_file.push_back("etc/calib/LUT_sph_cam1.txt");
    LUTs_file.push_back("etc/calib/LUT_sph_cam2.txt");

    LUTs_type.push_back("PlCa");
    LUTs_type.push_back("PlCa");

    OmniCamera omniSys(topics_name,path_yamls_cam,extrinParam);

    omniSys.DispParam();

    omniSys.camera_1->readImage(im_cam1);
    omniSys.camera_2->readImage(im_cam2);

    omniSys.camera_1->LoadMask(maskCamera_1);
    omniSys.camera_2->LoadMask(maskCamera_2);

    omniSys.LoadLUT(LUTs_file,LUTs_type);

    cv::namedWindow("imshow_cam1",cv::WINDOW_NORMAL);
//    cv::imshow("imshow_cam1",omniSys.camera_1->getImage());

//    cv::namedWindow("imshow_cam2",cv::WINDOW_NORMAL);
//    cv::imshow("imshow_cam2",omniSys.camera_2->getImage());

//    cv::waitKey(0);

//    cv::imshow("imshow_cam1",omniSys.camera_1->GetMask()*255);
//    cv::imshow("imshow_cam2",omniSys.camera_2->GetMask()*255);

//    cv::waitKey(0);

    omniSys.MergeLUT();

    omniSys.StitchImage(1);

    cv::imshow("imshow_cam1",omniSys.GetPano());

    cv::waitKey(0);

    return 0;

}
