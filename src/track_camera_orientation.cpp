#include "image_handler.h"
#include "poly_omni.h"
#include "omni_camera.h"
#include "sssoft.h"

#include <opencv2/core/core.hpp>

#include <iostream>
#include <string>


int main(int argc, char** argv){

    ros::init(argc,argv, "track_camera_orientation");

    double time;

    cv::Vec3f rotation;

    std::string im_cam1;
    std::string im_cam2;

    im_cam2 = "/home/student/JeremieDeray/rosbag/rotary_table/left/frame0000.jpg";
    im_cam1 = "/home/student/JeremieDeray/rosbag/rotary_table/right/frame0000.jpg";

    OmniCameraRig omniSys;

    if (!omniSys.IsInit())
    {
        std::cout << "Omni System Not Init !"<<std::endl;
        return 2;
    }

    omniSys.DispParam();

//    omniSys.camera_1->readImage(im_cam1);
//    omniSys.camera_2->readImage(im_cam2);

//    omniSys.SetPanoSize(400,400);

//    omniSys.MergeLUTWrap();

//    omniSys.StitchImage();

    cv::Mat sampSphFunc1, sampSphFunc2;

    int bwIn = 16;

//    sampSphFunc1 = omniSys.GetPano();

//    cv::imshow("tt",cv::imread("/home/student/JeremieDeray/rosbag/rotation/pano/100Img_-3.37156_-3.57351_0_0_0.598362_0.801226.jpg")); cv::waitKey(0); return 4;

//    SOFTWRAPP::SampSphFct(bwIn,cv::imread("/home/student/JeremieDeray/rosbag/rotation/pano/110Img_-3.16887_-3.31932_0_0_0.108815_0.994062.jpg") ,sampSphFunc1);

//    time = (double)cv::getTickCount();

//    im_cam2 = "/home/student/JeremieDeray/rosbag/rotary_table/left/frame0020.jpg";
//    im_cam1 = "/home/student/JeremieDeray/rosbag/rotary_table/right/frame0020.jpg";

//    omniSys.camera_1->readImage(im_cam1);
//    omniSys.camera_2->readImage(im_cam2);

//    omniSys.StitchImage();

//    sampSphFunc2 = omniSys.GetPano();

//    SOFTWRAPP::SampSphFct(bwIn,cv::imread("/home/student/JeremieDeray/rosbag/rotation/pano/100Img_-3.37156_-3.57351_0_0_0.598362_0.801226.jpg"),sampSphFunc2);

////    cv::imshow("im1",sampSphFunc1*255);
////    cv::imshow("im2",sampSphFunc2*255);

////    cv::waitKey(0);

////    return 10;

//    SOFTWRAPP::WrapSphCorr2(bwIn,sampSphFunc1,sampSphFunc2,rotation);

//    SOFTWRAPP::DispRotEst(rotation);

//    std::cout << "time to correlate : "<<((double)cv::getTickCount() - time) / cv::getTickFrequency()<<std::endl<<std::endl;

//    return -2;


    bwIn = 16;

    cv::Mat img_ref;
     double tmp_ang = 0.0,angle_sum = 0.0;

     double REFINEMENT_TRHESHOLD = Deg2Rad(45.0/2.0);

//     std::string path_to_img = "/home/student/JeremieDeray/rosbag/run2/images/omni/pano/";
     std::string path_to_img = "/home/student/rot_est_run2_5fps/";

     std::set<std::string> image_names;

     image_names = loadFilesName(path_to_img);

     std::set<std::string>::iterator it = image_names.begin();

     cv::Mat myimage = cv::imread(*it);

     image_names.erase(it);

     SOFTWRAPP::SampSphFct(bwIn,myimage,sampSphFunc1);

     sampSphFunc1.copyTo(img_ref);

     BOOST_FOREACH(std::string img, image_names)
     {
        myimage = cv::imread(img);

        SOFTWRAPP::SampSphFct(bwIn,myimage,sampSphFunc2);

        time = (double)cv::getTickCount();

        SOFTWRAPP::WrapSphCorr2(bwIn,sampSphFunc1,sampSphFunc2,rotation);

        std::cout << "%time to estimate : "<<((double)cv::getTickCount() - time) / cv::getTickFrequency()<<std::endl<<std::endl;

        SOFTWRAPP::DispRotEst(rotation);

//        tmp_ang = rotation[0] + rotation[2];

//        if(tmp_ang > mypi)
//        {
//            tmp_ang -= 2*mypi;
//        }else if(tmp_ang < -mypi){

//            tmp_ang += 2*mypi;
//        }

//        angle_sum += std::abs(tmp_ang);

//        if(angle_sum > REFINEMENT_TRHESHOLD)
//        {
//            SOFTWRAPP::WrapSphCorr2(bwIn,img_ref,sampSphFunc2,rotation);
//            std::cout<<"REFINED=[REFINED; "<<rotation[0]+rotation[2]<<"];"<<std::endl<<std::endl;
//            sampSphFunc2.copyTo(img_ref);
//            angle_sum = 0.0;
//        }

        sampSphFunc2.copyTo(sampSphFunc1);

     }

     return 6;

//     omniSys.ReadFrame();

//     omniSys.StitchImage();

//    SOFTWRAPP::SampSphFct(bwIn,omniSys.GetPano(),sampSphFunc1);


    do
    {
//        time = (double)cv::getTickCount();

//        omniSys.ReadFrame();

//        omniSys.StitchImage();

//        SOFTWRAPP::SampSphFct(bwIn,omniSys.GetPano(),sampSphFunc2);

//        SOFTWRAPP::WrapSphCorr2(bwIn,sampSphFunc1,sampSphFunc2,rotation);

//        SOFTWRAPP::DispRotEst(rotation);

//        std::cout << "time to estimate : "<<((double)cv::getTickCount() - time) / cv::getTickFrequency()<<std::endl<<std::endl;

//        rotZYZ = GetZYZRotationMat(rotation[0],rotation[1],rotation[2]);

//        std::cout << "ZYZ Rotation : "<< rotZYZ <<std::endl<<std::endl;

//        sampSphFunc2.copyTo(sampSphFunc1);

//        ros::spinOnce();


    }while(cv::waitKey(10) != 'q');

    return 0;
}
