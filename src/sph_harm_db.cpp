#include "image_handler.h"
#include "omni_camera.h"
#include "fisheye.h"
#include "sssoft.h"

#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <string>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sph_harm_db");

    if (argc == 4 && std::string(argv[1]) != "-h" && std::string(argv[1]) != "--help")
    {
        OmniCamera omniSys("");

        if (!omniSys.IsInit())
        {
            std::cout << "Omni System Not Init !"<<std::endl;
            return 2;
        }

        omniSys.SetPanoSize(400,400);

        cv::Mat sampSphFunc;
        int bwIn = 16;

        std::vector<std::string> list1;
        std::vector<std::string> list2;

        SOFTWRAPP::harmCoeff harmcoeff;

        getListOfFilesInFolder(std::string(argv[1]),std::string(argv[3]),list1);
        getListOfFilesInFolder(std::string(argv[2]),std::string(argv[3]),list2);

        for(int i = 0 ; i < list1.size(); i++)
        {
            omniSys.camera_1->readImage(list1[i]);
            omniSys.camera_2->readImage(list2[i]);

            omniSys.MergeLUTWrap();

            omniSys.StitchImage();

            SOFTWRAPP::SampSphFct(bwIn,omniSys.GetPano(),sampSphFunc);

            SOFTWRAPP::WrapSphHarm(bw,sampSphFunc,harmcoeff);


        }



    }
    else
    {
      ROS_ERROR("Usage:\n - sph_harm_db [ROS args...] <cam1 images directory> <cam2 images directory> <image extension> <output path>\n");
    }

    return 0;
}
