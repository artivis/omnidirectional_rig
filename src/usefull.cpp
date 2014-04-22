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




//void GetSphSampGrid(int bandwidth, cv::Mat &pts, bool ishemi){

//    cv::Mat theta = cv::Mat::zeros(1,bandwidth*bandwidth-1,CV_32FC1);
//    cv::Mat phi = cv::Mat::zeros(1,bandwidth*bandwidth-1,CV_32FC1);

//    cv::Mat theta_grid, phi_grid;

//    cv::MatIterator_<float> it_theta = theta.begin<float>(), it_theta_end = theta.end<float>(),
//            it_phi = phi.begin<float>();

//    int i = 0;

//    int hemi = (ishemi) ? 2 : 1;

//    for (;it_theta!=it_theta_end;it_theta++)
//    {
//        *it_theta = mypi*(2*i+1) / (4*bandwidth);

//        *it_phi = mypi*i / (hemi*bandwidth);

//        it_phi++;
//        i++;
//    }

//    MeshGrid(theta,phi,theta_grid,phi_grid);

//    cv::Mat tmp = theta_grid.t();

//    tmp.reshape(1,1).copyTo(theta_grid);

//    tmp = phi_grid.t();

//    tmp.reshape(1,1).copyTo(phi_grid);

//    cv::vconcat(theta_grid,phi_grid,pts);

//}


//void GetHemiSphSampGrid(cv::Mat &pts,int bandwidth){

//    GetSphSampGrid(bandwidth, pts, true);

//}


void MeshGrid(const cv::Mat &X_val, const cv::Mat &Y_val, cv::Mat &X_grid, cv::Mat &Y_grid)
{
    cv::repeat(X_val,Y_val.total(),1,X_grid);
    cv::repeat(Y_val.reshape(1,1).t(),1,X_val.total(),Y_grid);
}


void Sph2Cart(const cv::Mat &sph_pts, cv::Mat &cart_pts)
{
    cv::Mat cart_pts_tmp = cv::Mat::zeros(3,sph_pts.cols,sph_pts.type());

    if (sph_pts.rows == 2)
    {
        for (int i=0; i<sph_pts.cols; i++)
        {
            cart_pts_tmp.at<float>(0,i) = sin(sph_pts.at<float>(0,i)) * cos(sph_pts.at<float>(1,i));
            cart_pts_tmp.at<float>(1,i) = sin(sph_pts.at<float>(0,i)) * sin(sph_pts.at<float>(1,i));
            cart_pts_tmp.at<float>(2,i) = cos(sph_pts.at<float>(0,i));
        }

    }else if (sph_pts.rows == 3){

        for (int i=0; i<sph_pts.cols; i++)
        {
            cart_pts_tmp.at<float>(0,i) = sph_pts.at<float>(2,i) * sin(sph_pts.at<float>(0,i)) * cos(sph_pts.at<float>(1,i));
            cart_pts_tmp.at<float>(1,i) = sph_pts.at<float>(2,i) * sin(sph_pts.at<float>(0,i)) * sin(sph_pts.at<float>(1,i));
            cart_pts_tmp.at<float>(2,i) = sph_pts.at<float>(2,i) * cos(sph_pts.at<float>(0,i));
        }
    }else{
        return;
    }

    cart_pts_tmp.convertTo(cart_pts,sph_pts.type());
}

std::string AddPath(const std::string &obj, const std::string &root)
{
    std::stringstream ss;
            ss << root << "/" << obj;
            return ss.str();
}


void RotateCloudPoint(cv::Mat &ClPts, double roll, double pitch, double yaw, bool rad)
{
    cv::Mat rotationMat;
    cv::Mat tmp;

    rotationMat = GetRotationMat(roll,pitch,yaw,rad);

    rotationMat.convertTo(rotationMat,CV_32F);

    tmp = rotationMat * ClPts;

    tmp.convertTo(ClPts,ClPts.type());
}

cv::Mat GetRotationMat(double roll, double pitch, double yaw, bool rad)
{
    cv::Mat rotYaw = cv::Mat::eye(3,3,CV_64F);
    cv::Mat rotPitch = cv::Mat::eye(3,3,CV_64F);
    cv::Mat rotRoll = cv::Mat::eye(3,3,CV_64F);

    if (!rad)
    {
        yaw = Deg2Rad(yaw);
        pitch = Deg2Rad(pitch);
        roll = Deg2Rad(roll);
    }

    rotPitch.at<double>(0,0) =  cos( pitch ); //pitch
    rotPitch.at<double>(0,2) =  sin( pitch );
    rotPitch.at<double>(2,0) = -sin( pitch );
    rotPitch.at<double>(2,2) =  cos( pitch );


    rotYaw.at<double>(0,0) =  cos( yaw ); //yaw
    rotYaw.at<double>(0,1) = -sin( yaw );
    rotYaw.at<double>(1,0) =  sin( yaw );
    rotYaw.at<double>(1,1) =  cos( yaw );


    rotRoll.at<double>(1,1) =  cos( roll ); //roll
    rotRoll.at<double>(1,2) = -sin( roll );
    rotRoll.at<double>(2,1) =  sin( roll );
    rotRoll.at<double>(2,2) =  cos( roll );

    return rotYaw * rotPitch * rotRoll;
}

double Deg2Rad(double angle)
{
    return angle * (mypi/180.0);
}







