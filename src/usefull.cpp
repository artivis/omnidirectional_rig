#include "usefull.h"



template <class NumType>
cv::Mat Vector2Mat(std::vector< NumType > vect){

    cv::Mat matrix = cv::Mat::zeros(1,vect.size(), cv::DataType<NumType>::type);

    for (int r=0; r<vect.size(); r++)
    {
         matrix.at<NumType>(0,r) = vect[r];
    }

    return matrix;
}
