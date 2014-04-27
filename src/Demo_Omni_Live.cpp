#include "image_handler.h"
#include "omni_camera.h"
#include "fisheye.h"

#include <opencv2/core/core.hpp>

#include <iostream>
#include <string>


int main(int argc, char** argv){

    ros::init(argc,argv, "omnicamera_live");

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

    if (!omniSys.IsInit())
    {
        std::cout << "Omni System Not Init !"<<std::endl;
        return 2;
    }

    omniSys.DispParam();

    omniSys.camera_1->LoadMask(maskCamera_1);
    omniSys.camera_2->LoadMask(maskCamera_2);

    omniSys.SetPanoSize(400,1000);

    omniSys.MergeLUTWrap();

    do
    {
        omniSys.camera_1->ReadFrame();
        omniSys.camera_2->ReadFrame();

        omniSys.StitchImage(1);

        cv::imshow("OmniLive",omniSys.GetPano());

        cv::waitKey(100);

        ros::spinOnce();

    }while(cv::waitKey(10) != 'q');

    return 0;
}
