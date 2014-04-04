#ifndef PAL_VISUAL_LOCALIZATION_IMAGE_HANDLER_H_
#define PAL_VISUAL_LOCALIZATION_IMAGE_HANDLER_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace pal {
    namespace slam {

        class ImageHandler
        {

        public:

            ImageHandler(const std::string &topicName);
            ~ImageHandler();
            void topicCallback(const sensor_msgs::ImageConstPtr& received_image);
            bool imageReceived();
            sensor_msgs::ImageConstPtr getImage();
            sensor_msgs::ImageConstPtr waitUntilImageReceived();

            void saveImage(std::string name, cv::Mat& image);
            void saveImage(std::string name, IplImage* image);
            void readImage(std::string file, cv::Mat& img);

        private:

            ros::NodeHandle _nh;
            ros::CallbackQueue _cbqueue;
            ros::Subscriber _subscriber;
            bool _imageReceived;
            sensor_msgs::ImageConstPtr _image;
            std::string folder;
        };
    }
}

#endif  // PAL_VISUAL_LOCALIZATION_IMAGE_HANDLER_H_
