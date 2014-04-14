#ifndef USEFULL_H
#define USEFULL_H

#include <vector>
#include <iostream>
#include <math.h>
#include <opencv2/core/core.hpp>



const float pi = 3.141592653589793238462643383279;


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

void Sph2Cart(const cv::Mat&, cv::Mat &);

void GetSphSampGrid(int bandwidth, cv::Mat &pts, bool ishemi = false);

void GetHemiSphSampGrid(int bandwidth, cv::Mat &pts);

void MeshGrid(const cv::Mat&, const cv::Mat&, cv::Mat&, cv::Mat&);

#endif // USEFULL_H
