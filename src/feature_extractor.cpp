#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "feature_extractor.h"

namespace pal {
  namespace slam {

    FeatureExtractor::FeatureExtractor()
    {
    }

    FeatureExtractor::~FeatureExtractor()
    {
      // Free all resources allocated by libsiftfast (eg. cached kernels).
      DestroyAllResources();
    }

    cv::Mat FeatureExtractor::getImageFromMessage(const sensor_msgs::ImageConstPtr& sensor_image)
    {
      cv_bridge::CvImagePtr image;
      try
      {
        image = cv_bridge::toCvCopy(sensor_image, sensor_msgs::image_encodings::MONO8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
        return cv::Mat();
      }

      // TODO: Do we need the .clone() here?
      return image->image.clone();
    }

    FeatureVector FeatureExtractor::processFile(const std::string& filename)
    {
      ROS_INFO_STREAM("Processing file: " << filename);
      cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
      return processImage(image);
    }

    FeatureVector FeatureExtractor::processImage(const cv::Mat& image)
    {
      ROS_INFO_STREAM("Extracting features from " << image.rows
                                                  << " x " << image.cols
                                                  << " image...");
      Keypoint keypoints = extractKeypoints(image);

      FeatureVector features;
      Keypoint p = keypoints;
      while (p != NULL)
      {
        Feature f(p->descrip);
        features.push_back(f);
        p = p->next;
      }
      FreeKeypoints(keypoints);

      ROS_INFO_STREAM("Done! " << features.size() << " features found.");
      return features;
    }

    Keypoint FeatureExtractor::extractKeypoints(const cv::Mat& image)
    {
      Image sift_image = CreateImage(image.rows, image.cols);
      for (int i = 0; i < image.rows; ++i)
      {
        uint8_t* pSrc = (uint8_t*) image.data + image.step * i;
        float* pDst = sift_image->pixels + i * sift_image->stride;
        for (int j = 0; j < image.cols; ++j)
          pDst[j] = (float) pSrc[j] * (1.0f / 255.0f);
      }

      Keypoint keypoints;
      keypoints = GetKeypoints(sift_image);
      DestroyAllImages();
      return keypoints;
    }

  }
}
