#include "image_handler.h"

#include "omni_camera.h"

#include "fisheye.h"

#include <opencv2/core/core.hpp>

#include <iostream>
#include <string>


int main(int argc, char** argv){

    ros::init(argc,argv, "omni_localization");

//    const std::string cloudPtTopic = "/cloud_sphere";
//    ros::NodeHandle nh;

    //ros::Publisher pub_CloudSph = nh.advertise<sensor_msgs::PointCloud>(cloudPtTopic,1);


    OmniCamera omniSys("");


    std::string im_cam1;
    std::string im_cam2;


    im_cam1 = "etc/images/cam1/left_frame0000.jpg";
    im_cam2 = "etc/images/cam2/right_frame0000.jpg";


    pal::slam::FeatureVector featurevector_cam1;
    pal::slam::FeatureVector featurevector_cam2;
    pal::slam::FeatureExtractor featureExtractor;


    omniSys.DispParam();



    do
    {
        omniSys.ReadFrame();
        omniSys.camera_1->readImage(im_cam1);
        omniSys.camera_2->readImage(im_cam2);

        featurevector_cam1 = featureExtractor.processImage(omniSys.camera_1->getImage());
        featurevector_cam2 = featureExtractor.processImage(omniSys.camera_2->getImage());






        ros::spinOnce();

    }while(true);

    return 0;

}
