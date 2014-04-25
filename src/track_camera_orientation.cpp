#include "image_handler.h"
#include "omni_camera.h"
#include "fisheye.h"
#include "sssoft.h"

#include <opencv2/core/core.hpp>

#include <iostream>
#include <string>


int main(int argc, char** argv){

    ros::init(argc,argv, "track_camera_orientation");

//    const std::string cloudPtTopic = "/cloud_sphere";
//    ros::NodeHandle nh;

//    ros::Publisher pub_CloudSph = nh.advertise<sensor_msgs::PointCloud>(cloudPtTopic,0);

    std::vector<std::string> path_yamls_cam;
    std::vector<std::string> topics_name;
    std::string im_cam1;
    std::string im_cam2;
    std::string maskCamera_1;
    std::string maskCamera_2;
    std::string extrinParam;

    path_yamls_cam.push_back("etc/calib/Pal_intrinsicParam_cam1.yaml");
    path_yamls_cam.push_back("etc/calib/Pal_intrinsicParam_cam2.yaml");

    im_cam1 = "etc/images/left_frame0000.jpg";
    im_cam2 = "etc/images/right_frame0000.jpg";

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

    omniSys.camera_1->readImage(im_cam1);
    omniSys.camera_2->readImage(im_cam2);

    omniSys.camera_1->LoadMask(maskCamera_1);
    omniSys.camera_2->LoadMask(maskCamera_2);

    double time;

    cv::Mat sampSphFunc1, sampSphFunc2;

    int bwIn = 16, bwOut = 8;

    std::vector< std::vector< std::complex<double> > > sphHarm1, sphHarm2;

    omniSys.SampSphFct(sampSphFunc1,bwIn);

    MatInfo(sampSphFunc1,"sampSphFunc1",true);

    std::cout << "START !!!!!!!!!!!!!!!!!!!!!"<<std::endl;

    SOFTWRAPP::SphericalHarmonics(bwIn,sampSphFunc1,sphHarm1);

    SOFTWRAPP::DispSphHarm(sphHarm1);

    std::cout << "MIDDLE !!!!!!!!!!!!!!!!!!!!!"<<std::endl;

    sampSphFunc1.release(); sphHarm1.clear();

    omniSys.SampSphFct(sampSphFunc1,bwIn);

    MatInfo(sampSphFunc1,"sampSphFunc1",true);

    SOFTWRAPP::SphericalHarmonics(bwIn,sampSphFunc1,sphHarm1);

    SOFTWRAPP::DispSphHarm(sphHarm1);

    std::cout << "END !!!!!!!!!!!!!!!!!!!!!"<<std::endl;

    im_cam1 = "etc/images/left_frame0000.jpg";
    im_cam2 = "etc/images/right_frame0000.jpg";

    omniSys.camera_1->readImage(im_cam1);
    omniSys.camera_2->readImage(im_cam2);

    omniSys.SampSphFct(sampSphFunc2,bwIn);

    MatInfo(sampSphFunc1,"sampSphFunc2",true);

    std::cout << "START !!!!!!!!!!!!!!!!!!!!!"<<std::endl;

    SOFTWRAPP::SphericalHarmonics(bwIn,sampSphFunc2,sphHarm2);

    SOFTWRAPP::DispSphHarm(sphHarm2);

    std::cout << "END !!!!!!!!!!!!!!!!!!!!!"<<std::endl;

    cv::Vec3f rotation;

    SOFTWRAPP::CorrSO3(bwIn,bwOut,sphHarm1,sphHarm1,rotation);

    SOFTWRAPP::DispRotEst(rotation);

    return -2;






    do
    {
        time = (double)cv::getTickCount();

        omniSys.camera_1->ReadFrame();
        omniSys.camera_2->ReadFrame();

        std::cout << "time to publish sphere : "<<((double)cv::getTickCount() - time) / cv::getTickFrequency()<<std::endl<<std::endl;

        ros::spinOnce();

    }while(cv::waitKey(10) != 'q');

    return 0;
}
