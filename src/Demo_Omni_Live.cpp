#include "image_handler.h"
#include "omni_camera.h"
#include "fisheye.h"

#include <opencv2/core/core.hpp>

#include <iostream>
#include <string>


int main(int argc, char** argv){

    ros::init(argc,argv, "omnicamera_live");

    OmniCamera omniSys("");

    if (!omniSys.IsInit())
    {
        std::cout << "Omni System Not Init !"<<std::endl;
        return 2;
    }

    omniSys.DispParam();

    omniSys.SetPanoSize(400,1000);

    omniSys.MergeLUTWrap(true);



//    // Test with images from disk
//    double time;
//    std::string imCam1 = "etc/images/right_frame0000.jpg";
//    std::string imCam2 = "etc/images/left_frame0000.jpg";
//    omniSys.camera_1->readImage(imCam1);
//    omniSys.camera_2->readImage(imCam2);

//    time = (double)cv::getTickCount();
//    omniSys.StitchImage();
//    std::cout << "time to stitch : "<<((double)cv::getTickCount() - time) / cv::getTickFrequency()<<std::endl<<std::endl;

//    cv::namedWindow("pano");
//    cv::imshow("pano",omniSys.GetPano());
//    cv::waitKey(0);
//    cv::destroyWindow("pano");
//    return 25;

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
