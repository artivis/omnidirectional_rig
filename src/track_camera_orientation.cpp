#include "image_handler.h"
#include "omni_camera.h"
#include "fisheye.h"
#include "sssoft.h"

#include <opencv2/core/core.hpp>

#include <iostream>
#include <string>


int main(int argc, char** argv){

    ros::init(argc,argv, "track_camera_orientation");

    double time;

    cv::Vec3f rotation;

//    std::string im_cam1;
//    std::string im_cam2;

//    im_cam2 = "/home/student/JeremieDeray/rosbag/rotary_table/left/frame0000.jpg";
//    im_cam1 = "/home/student/JeremieDeray/rosbag/rotary_table/right/frame0000.jpg";

    OmniCamera omniSys("");

    if (!omniSys.IsInit())
    {
        std::cout << "Omni System Not Init !"<<std::endl;
        return 2;
    }

    omniSys.DispParam();

//    omniSys.camera_1->readImage(im_cam1);
//    omniSys.camera_2->readImage(im_cam2);

    omniSys.SetPanoSize(400,400);

    omniSys.MergeLUTWrap();

//    omniSys.StitchImage();

    cv::Mat sampSphFunc1, sampSphFunc2;

    int bwIn = 64;

//    sampSphFunc1 = omniSys.GetPano();

//    SOFTWRAPP::SampSphFct(bwIn,omniSys.GetPano(),sampSphFunc1);

//    time = (double)cv::getTickCount();

//    im_cam2 = "/home/student/JeremieDeray/rosbag/left/frame0048.jpg";
//    im_cam1 = "/home/student/JeremieDeray/rosbag/right/frame0048.jpg";

//    omniSys.camera_1->readImage(im_cam1);
//    omniSys.camera_2->readImage(im_cam2);

//    omniSys.StitchImage();

//    sampSphFunc2 = omniSys.GetPano();

//    SOFTWRAPP::SampSphFct(bwIn,omniSys.GetPano(),sampSphFunc2);

//    cv::imshow("im1",sampSphFunc1*255);
//    cv::imshow("im2",sampSphFunc2*255);

//    cv::waitKey(0);

//    return 10;



//    SOFTWRAPP::WrapSphCorr2(bwIn,sampSphFunc1,sampSphFunc2,rotation);

//    SOFTWRAPP::DispRotEst(rotation);

//    std::cout << "time to correlate : "<<((double)cv::getTickCount() - time) / cv::getTickFrequency()<<std::endl<<std::endl;

//    return -2;


     cv::Mat rotZYZ;

     omniSys.ReadFrame();

     omniSys.StitchImage();

    SOFTWRAPP::SampSphFct(bwIn,omniSys.GetPano(),sampSphFunc1);



    do
    {
        time = (double)cv::getTickCount();

        omniSys.ReadFrame();

        omniSys.StitchImage();

        SOFTWRAPP::SampSphFct(bwIn,omniSys.GetPano(),sampSphFunc2);

        SOFTWRAPP::WrapSphCorr2(bwIn,sampSphFunc1,sampSphFunc2,rotation);

        SOFTWRAPP::DispRotEst(rotation);

        std::cout << "time to estimate : "<<((double)cv::getTickCount() - time) / cv::getTickFrequency()<<std::endl<<std::endl;

        rotZYZ = GetZYZRotationMat(rotation[0],rotation[1],rotation[2]);

        std::cout << "ZYZ Rotation : "<< rotZYZ <<std::endl<<std::endl;

        sampSphFunc2.copyTo(sampSphFunc1);

        ros::spinOnce();


    }while(cv::waitKey(10) != 'q');

    return 0;
}
