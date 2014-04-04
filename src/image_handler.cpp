#include <ros/ros.h>

#include "pal_visual_localization/image_handler.h"

namespace pal {
    namespace slam {

        ImageHandler::ImageHandler(const std::string &topicName):_imageReceived(false)
        {
            ROS_DEBUG("Building image handler");
            _nh.param("images_folder",folder,std::string("etc/images/"));

            _nh.setCallbackQueue(&_cbqueue);
            _subscriber = _nh.subscribe(topicName, 1, &ImageHandler::topicCallback, this);
        }

        ImageHandler::~ImageHandler()
        {
        }

        void ImageHandler::topicCallback(const sensor_msgs::ImageConstPtr& received_image)
        {
            _imageReceived = true;
            _image = received_image;
        }

        bool ImageHandler::imageReceived()
        {
            return _imageReceived;
        }

        sensor_msgs::ImageConstPtr  ImageHandler::getImage()
        {
            sensor_msgs::ImageConstPtr empty;
            if (!_imageReceived)
            {
                ROS_ERROR("Called getImage() without any image received");
                return empty;
            }
            return _image;
        }

        sensor_msgs::ImageConstPtr  ImageHandler::waitUntilImageReceived()
        {
            _imageReceived=false;
            ROS_DEBUG("waiting until image received");
            ros::Rate rate(100.0);
            while (!_imageReceived  && ros::ok())
            {
                _cbqueue.callOne();
                rate.sleep();
            }
            return getImage();
        }
        void ImageHandler::saveImage(std::string name, cv::Mat& image)
        {
            /**
             * @brief Save cv::Mat images in disk
             * @param directory: directory to save the images
             * @param image: image to be saved
             */

            std::string dir = folder + name;
            ROS_DEBUG_STREAM("Saving image in " << dir.c_str());
            const char *dirdef;
            dirdef = dir.c_str();
            cv::imwrite(dirdef, image);
        }

        void ImageHandler::saveImage(std::string name, IplImage* image)
        {
            /**
             * @brief Save IplImage* image in disk
             * @param directory: directory to save the images
             * @param image: image to be saved
             */

            std::string dir = folder + name;
            ROS_DEBUG_STREAM("Saving image in "<< dir.c_str());
            std::ostringstream oss;
            oss << dir.c_str();
            cvSaveImage(oss.str().c_str(), image);
        }

        void ImageHandler::readImage(std::string file, cv::Mat& img)
        {
            /**
             * @brief Read an image from disk
             * @param file: image file name to read
             * @param img: cv::Mat variable to load the image
             */

            ROS_DEBUG_STREAM("Reading image " << file.c_str());
            const char *dirdef;
            dirdef = file.c_str();
            img = cv::imread(dirdef);

        }

    }
}
