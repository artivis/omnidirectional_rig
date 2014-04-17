#include "image_handler.h"

#include "omni_camera.h"

#include "fisheye.h"

#include <opencv2/core/core.hpp>

#include <iostream>
#include <string>


int main(int argc, char** argv){

    ros::init(argc,argv, "omni_localization");


    const std::string cloudPtTopic = "/cloud_sphere";
    ros::NodeHandle nh;

    ros::Publisher pub_CloudSph = nh.advertise<sensor_msgs::PointCloud>(cloudPtTopic,1);

    std::vector<std::string> path_yamls_cam;
    std::vector<std::string> topics_name;
    std::vector<std::string> LUTs_file;
    std::vector<std::string> LUTs_type;
    std::string im_cam1;
    std::string im_cam2;
    std::string maskCamera_1;
    std::string maskCamera_2;
    std::string extrinParam;

    path_yamls_cam.push_back("etc/calib/Pal_intrinsicParam_cam1.yaml");
    path_yamls_cam.push_back("etc/calib/Pal_intrinsicParam_cam2.yaml");

    im_cam1 = "etc/images/cam1/left_frame0000.jpg";
    im_cam2 = "etc/images/cam2/right_frame0000.jpg";

    maskCamera_1  = "etc/images/cam1/Img_mask1.jpg";
    maskCamera_2  = "etc/images/cam2/Img_mask2.jpg";

    topics_name.push_back("/left/image_raw");
    topics_name.push_back("/right/image_raw");

    extrinParam = "etc/calib/Pal_extrinsicParam.yaml";

    LUTs_file.push_back("etc/calib/LUT_sph_cam1.txt");
    LUTs_file.push_back("etc/calib/LUT_sph_cam2.txt");

    LUTs_type.push_back("Sphere");
    LUTs_type.push_back("Sphere");

    OmniCamera omniSys(topics_name,path_yamls_cam,extrinParam);

    omniSys.DispParam();

    cv::namedWindow("imshow_cam1",cv::WINDOW_NORMAL);
    cv::namedWindow("imshow_cam2",cv::WINDOW_NORMAL);

//    cv::Mat ima1;
//    cv::Mat ima2;

//    int tt = cv::waitKey(50);

//    std::cout<<"key val . "<< tt <<std::endl;

//    while (true)
//    {
//        omniSys.camera_1->ReadFrame();
//        omniSys.camera_2->ReadFrame();

//        ima1 = omniSys.camera_1->getImage();
//        ima2 = omniSys.camera_2->getImage();

//        cv::imshow("imshow_cam1",ima1);
//        cv::imshow("imshow_cam2",ima2);

//        std::cout<<"caloop "<<std::endl;

//        int tt = cv::waitKey(50);

//        std::cout<<"key val . "<< tt <<std::endl;

//        if (cv::waitKey(50) == 131143) break; //q key
//    }

//    omniSys.camera_1->readImage(im_cam1);
//    omniSys.camera_2->readImage(im_cam2);

    omniSys.camera_1->LoadMask(maskCamera_1);
    omniSys.camera_2->LoadMask(maskCamera_2);

//    omniSys.LoadLUT(LUTs_file,LUTs_type);

//    cv::imshow("imshow_cam1",omniSys.camera_1->getImage());
//    cv::imshow("imshow_cam2",omniSys.camera_2->getImage());

//    cv::waitKey(0);

//    cv::imshow("imshow_cam1",omniSys.camera_1->GetMask()*255);
//    cv::imshow("imshow_cam2",omniSys.camera_2->GetMask()*255);

//    cv::waitKey(0);

//    omniSys.MergeLUT();

//    omniSys.StitchImage(1);

    sensor_msgs::PointCloud ptsCld;

    double time;

       omniSys.PartiallyFillMess(ptsCld);

    std::cout<<" mess : "<<ptsCld.points.size()<<std::endl;

    do
    {
        std::cout << "before frame : "<<std::endl;

        omniSys.camera_1->ReadFrame();
        omniSys.camera_2->ReadFrame();

        std::cout << "after frame : "<<std::endl;

        time = (double)cv::getTickCount();

        omniSys.MessRGBSph(ptsCld);

        std::cout << "time to comp sphere : "<<((double)cv::getTickCount() - time) / cv::getTickFrequency()<<std::endl<<std::endl;

        time = (double)cv::getTickCount();

        pub_CloudSph.publish(ptsCld);

        std::cout << "time to publish sphere : "<<((double)cv::getTickCount() - time) / cv::getTickFrequency()<<std::endl<<std::endl;

        ros::spinOnce();

    }while(true);

//    cv::imshow("imshow_cam1",omniSys.GetPano());


//    cv::waitKey(0);

    return 0;

}
