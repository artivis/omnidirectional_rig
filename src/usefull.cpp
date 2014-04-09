#include "usefull.h"



void Cart2Sph(const cv::Mat& cart_coor, cv::Mat& sph_coor, int rad_flag )
{

    if (!cart_coor.rows==3)
    {
        return;
    }

    int loop = std::max(cart_coor.rows,cart_coor.cols);

    double x,y,z;

    sph_coor = cv::Mat::zeros(cart_coor.rows,cart_coor.cols,cart_coor.type());

    if(rad_flag == 1){
        for (int i=0;i<loop;i++)
        {
            x = cart_coor.at<double>(0,i);
            y = cart_coor.at<double>(1,i);
            z = cart_coor.at<double>(2,i);

            sph_coor.at<double>(0,i) = atan2(y,x); //azimmuth

            sph_coor.at<double>(0,i) = atan2(z,sqrt( x*x + y*y )); //elevation

            sph_coor.at<double>(0,i) = sqrt( x*x + y*y + z*z ); //radius
        }
    }else{
        for (int i=0;i<loop;i++)
        {
            x = cart_coor.at<double>(0,i);
            y = cart_coor.at<double>(1,i);

            sph_coor.at<double>(0,i) = atan2(y,x); //azimmuth

            sph_coor.at<double>(0,i) = atan2(z,sqrt( x*x + y*y )); //elevation
        }
    }
}
