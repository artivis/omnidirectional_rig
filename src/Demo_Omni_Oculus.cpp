#include "image_handler.h"
#include "omni_camera.h"
#include "fisheye.h"

#include <opencv2/core/core.hpp>

#include <iostream>
#include <string>



#include <boost/range/combine.hpp>
#include <boost/tuple/tuple.hpp>
//#include <boost/foreach.hpp>
//#include <boost/filesystem.hpp>

//std::set<std::string> loadFilesName(const std::string &dir)
//{
//    std::set<std::string> result;

//    boost::filesystem::path directory(dir);
//    if (!boost::filesystem::is_directory(dir))
//    {
//        ROS_ERROR_STREAM("ERROR! Directory not found: " << dir);
//    }

//    boost::filesystem::directory_iterator end_iter;
//    for (boost::filesystem::directory_iterator it(dir); it != end_iter; ++it)
//    {
//        boost::filesystem::path file = it->path().filename();;

//        if (boost::filesystem::is_regular_file(it->status()) && file.filename().string()[0] != '.')
//        {
//            result.insert(file.string());
//        }
//    }
//    return result;
//}




int main(int argc, char** argv){

    //if (argc != 3) return -1;
    //if (argv == NULL) return -1;

    int sampling_ratio = atof(argv[1]);
    std::string conf_path = argv[2]; //TODO check nb arg & val

    ros::init(argc,argv, "demo_omni_oculus");

    ros::NodeHandle nh;

    std::vector<std::string> path_yamls_cam;
    std::vector<std::string> topics_name;
    std::string maskCamera_1;
    std::string maskCamera_2;
    std::string extrinParam;

    const std::string cloudPtTopic = "/cloud_sphere";

    ros::Publisher pub_CloudSph = nh.advertise<sensor_msgs::PointCloud>(cloudPtTopic,0);

    path_yamls_cam.push_back(AddPath("etc/calib/proto2/Pal_intrinsicParam_cam1.yaml",conf_path));
    path_yamls_cam.push_back(AddPath("etc/calib/proto2/Pal_intrinsicParam_cam2.yaml",conf_path));

    extrinParam = AddPath("etc/calib/proto2/Pal_extrinsicParam.yaml",conf_path);

    maskCamera_1  = AddPath("etc/images/cam1/Img_mask1.jpg",conf_path);
    maskCamera_2  = AddPath("etc/images/cam2/Img_mask2.jpg",conf_path);

    topics_name.push_back("/left/image_raw");
    topics_name.push_back("/right/image_raw");

    OmniCamera omniSys(topics_name,path_yamls_cam,extrinParam);

    omniSys.DispParam();

    omniSys.camera_1->LoadMask(maskCamera_1);
    omniSys.camera_2->LoadMask(maskCamera_2);

    omniSys.DownSample(sampling_ratio);

    omniSys.DispParam();

    sensor_msgs::PointCloud ptsCld;

    double time;
    char exit;

    omniSys.PartiallyFillMess(ptsCld);

    std::set<std::string> imagesR = loadFilesName("/home/student/JeremieDeray/rosbag/runvideo/images/1/right/");
    std::set<std::string> imagesL = loadFilesName("/home/student/JeremieDeray/rosbag/runvideo/images/1/left/");

//    do
//    {
//        omniSys.camera_1->ReadFrame();
//        omniSys.camera_2->ReadFrame();

//        time = (double)cv::getTickCount();

//        omniSys.MessRGBSph(ptsCld);

//        //std::cout << "time to comp sphere : "<<((double)cv::getTickCount() - time) / cv::getTickFrequency()<<std::endl<<std::endl;

//        time = (double)cv::getTickCount();

//        pub_CloudSph.publish(ptsCld);

//        //std::cout << "time to publish sphere : "<<((double)cv::getTickCount() - time) / cv::getTickFrequency()<<std::endl<<std::endl;

//        ros::spinOnce();

//        exit = cv::waitKey(10);

//        if(exit == 27) break;

//    }while(1);


    std::cout << "ABOUT TO BOOST " << std::endl;
    std::cout << "FILESR "<< imagesR.size() << std::endl;
    std::cout << "FILESL "<< imagesL.size() << std::endl;
    std::string file1,file2;

    BOOST_FOREACH(boost::tie(file1,file2), boost::combine(imagesR,imagesL))
    {
        std::cout << "BOOSTING " << std::endl;

        omniSys.camera_1->readImage(file1);
        omniSys.camera_2->readImage(file2);

        omniSys.MessRGBSph(ptsCld);

        pub_CloudSph.publish(ptsCld);

        std::cout << "PUBLISHED " << std::endl;

        ros::spinOnce();

        exit = cv::waitKey(4000);
    }



    return 0;
}
