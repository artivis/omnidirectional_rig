#include "image_handler.h"
#include "poly_omni.h"

#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <string>


int main(int argc, char** argv){

    ros::init(argc,argv, "omnicamera_live");
    ros::NodeHandle nh;

    std::vector<std::string> topics_name;
    std::string topic;
    nh.param("/spherical_vision/topic_left",topic,std::string("/left/image_raw"));
    topics_name.push_back(topic);
    nh.param("/spherical_vision/topic_right",topic,std::string("/right/image_raw"));
    topics_name.push_back(topic);

    OmniCameraRig omniSys;

    if (!omniSys.IsInit())
    {
        std::cout << "Omni System Not Init !"<<std::endl;
        return 2;
    }

    const std::string panoTopic = "/omni_camera/panorama";

    image_transport::ImageTransport imTrans(nh);
    sensor_msgs::Image img_msg;
    image_transport::Publisher pub = imTrans.advertise(panoTopic, 1);
    cv_bridge::CvImage cv_img;

    cv_img.encoding = "bgr8";


    omniSys.DispParam();

    omniSys.SetPanoSize(400,1000);

    omniSys.MergeLUTWrap();

    double time;

    SyncImageHandler syncImageHandler(topics_name[0],topics_name[1]);


//     //Test with images from disk
//    std::string imCam1 = "/home/student/JeremieDeray/rosbag/run3/images/omni/left/025Img_10.8304_-2.4232_0_0_-0.112161_0.99369.jpg";
//    std::string imCam2 = "/home/student/JeremieDeray/rosbag/run3/images/omni/right/025Img_10.8304_-2.4232_0_0_-0.112161_0.99369.jpg";
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

    std::vector< cv::Mat > images;

    do
    {
        syncImageHandler.waitUntilImages(images);

        omniSys.camera_1->setImage(images[0]);
        omniSys.camera_2->setImage(images[1]);

        time = (double)cv::getTickCount();

        omniSys.StitchImage();

        //cv_img.header.stamp = ros::Time::now();

        cv_img.image = omniSys.GetPano();

        cv_img.toImageMsg(img_msg);

        pub.publish(img_msg);

        img_msg.header.stamp = ros::Time::now();

        std::cout << "time to stitch : "<<((double)cv::getTickCount() - time) / cv::getTickFrequency()<<std::endl<<std::endl;

        ros::spinOnce();

    }while(cv::waitKey(10) != 'q');

    return 0;
}
