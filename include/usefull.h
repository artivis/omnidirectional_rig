#ifndef USEFULL_H
#define USEFULL_H

#include <vector>
#include <iostream>
#include <math.h>
#include <opencv2/core/core.hpp>

template <class NumType>
cv::Mat Vector2Mat(std::vector< NumType > vect){

    cv::Mat matrix = cv::Mat::zeros(1,vect.size(), cv::DataType<NumType>::type);

    for (int r=0; r<vect.size(); r++)
    {
         matrix.at<NumType>(0,r) = vect[r];
    }

    return matrix;
}


void Cart2Sph(const cv::Mat&, cv::Mat&, int rad_flag = 0);

#endif // USEFULL_H
