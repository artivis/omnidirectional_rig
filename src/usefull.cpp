#include "usefull.h"



void Cart2Sph(const cv::Mat& cart_coor, cv::Mat& sph_coor, int rad_flag )
{

    if (!cart_coor.rows==3)
    {
        return;
    }

    int loop = std::max(cart_coor.rows,cart_coor.cols);

    double x,y,z;

    sph_coor = cv::Mat::ones(cart_coor.rows,cart_coor.cols,cart_coor.type());

    if(rad_flag == 1){
        for (int i=0;i<loop;i++)
        {
            x = cart_coor.at<double>(0,i);
            y = cart_coor.at<double>(1,i);
            z = cart_coor.at<double>(2,i);

            sph_coor.at<double>(0,i) = atan2(y,x); //azimmuth

            sph_coor.at<double>(1,i) = atan2(z,sqrt( x*x + y*y )); //elevation

            sph_coor.at<double>(2,i) = sqrt( x*x + y*y + z*z ); //radius
        }
    }else{
        for (int i=0;i<loop;i++)
        {
            x = cart_coor.at<double>(0,i);
            y = cart_coor.at<double>(1,i);

            sph_coor.at<double>(0,i) = atan2(y,x); //azimmuth

            sph_coor.at<double>(1,i) = atan2(z,sqrt( x*x + y*y )); //elevation
        }
    }
}




void GetSphSampGrid(int bandwidth, cv::Mat &pts, bool ishemi){

    cv::Mat theta = cv::Mat::zeros(1,bandwidth*bandwidth-1,CV_32FC1);
    cv::Mat phi = cv::Mat::zeros(1,bandwidth*bandwidth-1,CV_32FC1);

    cv::Mat theta_grid, phi_grid;

    cv::MatIterator_<float> it_theta = theta.begin<float>(), it_theta_end = theta.end<float>(),
            it_phi = phi.begin<float>();

    int i = 0;

    int hemi = (ishemi) ? 2 : 1;

    for (;it_theta!=it_theta_end;it_theta++)
    {
        *it_theta = mypi*(2*i+1) / (4*bandwidth);

        *it_phi = mypi*i / (hemi*bandwidth);

        it_phi++;
        i++;
    }

    MeshGrid(theta,phi,theta_grid,phi_grid);

    cv::Mat tmp = theta_grid.t();

    tmp.reshape(1,1).copyTo(theta_grid);

    tmp = phi_grid.t();

    tmp.reshape(1,1).copyTo(phi_grid);

    cv::vconcat(theta_grid,phi_grid,pts);

}


void GetHemiSphSampGrid(int bandwidth, cv::Mat &pts){

    GetSphSampGrid(bandwidth, pts, true);

}


void MeshGrid(const cv::Mat &X_val, const cv::Mat &Y_val, cv::Mat &X_grid, cv::Mat &Y_grid)
{
    cv::repeat(X_val,Y_val.total(),1,X_grid);
    cv::repeat(Y_val.reshape(1,1).t(),1,X_val.total(),Y_grid);
}


void Sph2Cart(const cv::Mat &sph_pts, cv::Mat &cart_pts)
{

    const float *ptr_theta = sph_pts.ptr<float>(0);
    const float *ptr_phi = sph_pts.ptr<float>(1);

    cart_pts = cv::Mat::zeros(3,sph_pts.cols,CV_32FC1);

    float *ptr_x = cart_pts.ptr<float>(0);
    float *ptr_y = cart_pts.ptr<float>(1);
    float *ptr_z = cart_pts.ptr<float>(2);

    if (sph_pts.rows == 2)
    {
        for (int i=0; i<sph_pts.cols; i++)
        {

            *ptr_x = sin(*ptr_theta) * cos(*ptr_phi);

            *ptr_y = sin(*ptr_theta) * sin(*ptr_phi);

            *ptr_z = cos(*ptr_theta);

            ptr_x++;
            ptr_y++;
            ptr_z++;

            ptr_theta++;
            ptr_phi++;
        }
    }else if (sph_pts.rows == 3){

        const float *ptr_rad = sph_pts.ptr<float>(2);

        for (int i=0; i<sph_pts.cols; i++)
        {
            *ptr_x = *ptr_rad * sin(*ptr_theta) * cos(*ptr_phi);

            *ptr_y = *ptr_rad * sin(*ptr_theta) * sin(*ptr_phi);

            *ptr_z = *ptr_rad * cos(*ptr_theta);

            ptr_x++;
            ptr_y++;
            ptr_z++;

            ptr_theta++;
            ptr_phi++;
            ptr_rad++;

        }
    }else{
        return;
    }
}



