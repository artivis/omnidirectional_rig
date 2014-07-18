#include <ros/ros.h>

#include "image_handler.h"
#include <cv_bridge/cv_bridge.h>
#include <boost/foreach.hpp>


        SingleImageHandler::SingleImageHandler(const std::string &topicName) :
        _imageReceived(false),
        _it(_nh)
        {
            ROS_DEBUG("Building image handler");

            _nh.setCallbackQueue(&_cbqueue);
            _subscriber = _it.subscribe(topicName, 1, &SingleImageHandler::topicCallback, this);
        }

        SingleImageHandler::~SingleImageHandler()
        {
        }

        void SingleImageHandler::topicCallback(const sensor_msgs::ImageConstPtr& received_image)
        {
            _imageReceived = true;
            _image = received_image;
        }

        bool SingleImageHandler::isImageReceived()
        {
            return _imageReceived;
        }

        void SingleImageHandler::getImage(sensor_msgs::ImageConstPtr &image)
        {
            if (!_imageReceived)
            {
                ROS_ERROR("Called getImage() without any image received");
                image = sensor_msgs::ImageConstPtr();
            }
            image = _image;
        }

        void SingleImageHandler::waitUntilImageReceived(sensor_msgs::ImageConstPtr &image)
        {
            _imageReceived=false;
            ROS_DEBUG("waiting until image received");
            while (!_imageReceived  && ros::ok())
            {
                _cbqueue.callOne();
                ros::Duration(0.01).sleep();
            }
            getImage(image);
        }

        void SingleImageHandler::getImage(cv::Mat &image)
        {
            if (!_imageReceived)
            {
                ROS_ERROR("Called getImage() without any image received");
                image =cv::Mat();
            }

            cv_bridge::CvImagePtr cvPtr;
            cvPtr = cv_bridge::toCvCopy(_image,"8UC3");

            image = cvPtr->image;
        }

        void SingleImageHandler::waitUntilImageReceived(cv::Mat &image)
        {
            _imageReceived=false;
            ROS_DEBUG("waiting until image received");
            while (!_imageReceived  && ros::ok())
            {
                _cbqueue.callOne();
                ros::Duration(0.01).sleep();
            }
            return getImage(image);
        }

        void SingleImageHandler::saveImage(const std::string &name, cv::Mat& image)
        {
            /**
             * @brief Save cv::Mat images in disk
             * @param name: directory/name.extension to save the image
             * @param image: image to be saved
             */

            ROS_DEBUG_STREAM("Saving image in " << name);
            cv::imwrite(name, image);
        }

        void SingleImageHandler::saveImage(const std::string &name, IplImage* image)
        {
            /**
             * @brief Save IplImage* image in disk
             * @param name: directory/name.extension to save the image
             * @param image: image to be saved
             */

            std::string dir = name;
            ROS_DEBUG_STREAM("Saving image in "<< dir.c_str());
            std::ostringstream oss;
            oss << dir.c_str();
            cvSaveImage(oss.str().c_str(), image);
        }

        void SingleImageHandler::readImage(const std::string &file, cv::Mat& img)
        {
            /**
             * @brief Read an image from disk
             * @param file: image file name to read
             * @param img: cv::Mat variable to load the image
             */

            ROS_DEBUG_STREAM("Reading image " << file);
            img = cv::imread(file);

        }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



        SyncImageHandler::SyncImageHandler(const std::string &topic1, const std::string &topic2)
        :_imageReceived(false)
        {
            this->init(topic1,topic2);
        }

        SyncImageHandler::~SyncImageHandler()
        {

        }

        void SyncImageHandler::init(const std::string &topic1, const std::string &topic2)
        {
            _imageTransport.reset( new image_transport::ImageTransport( _nh));

            std::string transHint;
            _nh.param("_transport_img",transHint,std::string("raw"));

            image_transport::TransportHints transportHint(transHint);

            _imgSub1.reset(new image_transport::SubscriberFilter(*_imageTransport, topic1, 1, transportHint) );
            _imgSub2.reset(new image_transport::SubscriberFilter(*_imageTransport, topic2, 1, transportHint) );

            _approxSynchronizer.reset( new message_filters::Synchronizer< ApproxSync >(ApproxSync(10),
                                                                                       *_imgSub1, *_imgSub2) );

            _approxSynchronizer->registerCallback(boost::bind(&SyncImageHandler::stereoCallback, this, _1, _2));

            this->_images.resize(2);
            this->_imageReceived = false;
        }

        void SyncImageHandler::stereoCallback(const sensor_msgs::ImageConstPtr& msgImg1,
                                              const sensor_msgs::ImageConstPtr& msgImg2)
        {
            _images[0] = msgImg1;
            _images[1] = msgImg2;

            this->_imageReceived = true;
        }

        void SyncImageHandler::waitUntilImages(std::vector<sensor_msgs::ImageConstPtr > &images)
        {
            this->_imageReceived = false;
            ROS_INFO_STREAM("Waiting until images received");

            while (!this->_imageReceived  && ros::ok())
            {
                ros::Duration(0.01).sleep();
            }
            getImages(images);
        }

        void SyncImageHandler::getImages(std::vector<sensor_msgs::ImageConstPtr > &images)
        {
            if (!this->_imageReceived)
            {
                ROS_ERROR("Called getImage() without any image received");
                images = std::vector< sensor_msgs::ImageConstPtr >();
            }
            images = this->_images;
        }

        void SyncImageHandler::waitUntilImages(std::vector<cv::Mat > &images)
        {
            this->_imageReceived = false;
            ROS_INFO_STREAM("Waiting until images received");

            while (!this->_imageReceived  && ros::ok())
            {
                ros::Duration(0.01).sleep();
            }
            getImages(images);
        }

        void SyncImageHandler::getImages(std::vector<cv::Mat > &images)
        {
            if (!this->_imageReceived)
            {
                ROS_ERROR("Called getImage() without any image received");
                images = std::vector<cv::Mat >();
            }

            cv_bridge::CvImagePtr cvPtr;

            BOOST_FOREACH(const sensor_msgs::ImageConstPtr &img, this->_images)
            {
                cvPtr = cv_bridge::toCvCopy(img,"8UC3");
                images.push_back(cvPtr->image);
            }
        }
