#include <omni_camera.h>


OmniCamera::OmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &cameraParamPath)
{

    this->camera_1 = new FishEye(topicsName.at(0),cameraParamPath.at(0));

    this->camera_2 = new FishEye(topicsName.at(1),cameraParamPath.at(1));

    this->_panoSize = cv::Size(1200,400);

    this->_init = false;

    this->_isSampled = false;

    this->_sampling_ratio = 1;
}

OmniCamera::OmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &cameraParamPath, const std::string &extrinPath)
{
    this->camera_1 = new FishEye(topicsName.at(0),cameraParamPath.at(0));

    this->camera_2 = new FishEye(topicsName.at(1),cameraParamPath.at(1));

    this->LoadCalibration(extrinPath);

    this->_panoSize = cv::Size(1200,400);

    this->_init = true;

    this->_isSampled = false;

    this->_sampling_ratio = 1;
}

OmniCamera::~OmniCamera(){

    delete this->camera_1;
    delete this->camera_2;
}

bool OmniCamera::IsInit(){
    return this->_init;
}

bool OmniCamera::LoadCalibration(const std::string& paramPath){

    cv::FileStorage fs(paramPath,cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        std::cout<<"Failed to open "<<paramPath<< std::endl;
        return false;
    }

    fs["extrinsicParam"] >> this->_extrin;

    fs.release();

    this->_extrin.convertTo(this->_extrin,CV_32FC1);

    return true;
}


void OmniCamera::DispParam(){

    if(this->camera_1->IsInit()) {
        this->camera_1->DispParam();
    }

    if(this->camera_2->IsInit()) {
        this->camera_2->DispParam();
    }

    if(this->IsInit()) {
        std::cout<<"system extrinsic parameters : \n" << this->GetExtrin()<<std::endl<<std::endl;
    }
}


cv::Mat OmniCamera::GetExtrin(){
    return this->_extrin;
}

cv::Mat OmniCamera::GetPano(){
    return this->_pano;
}

cv::Mat OmniCamera::GetLUT(){
    return this->_LUT_wrap_im;
}

void OmniCamera::SetExtrin(const cv::Mat &extrin){
    this->_extrin = extrin;

}

void OmniCamera::LoadLUT(const std::vector<std::string> &LUTfiles, const std::vector<std::string> &LUTtype)
{
    if (!this->IsInit())
    {
        std::cout<<"Init camera first"<<std::endl;
        return;
    }

    this->camera_1->LoadLUT(LUTfiles[0],LUTtype[0]);
    this->camera_2->LoadLUT(LUTfiles[1],LUTtype[1]);
}

void OmniCamera::MergeLUTWrap(cv::Size size)
{

    if(this->camera_1->_LUT_wrap_im.empty() || this->camera_2->_LUT_wrap_im.empty()) return;

    cv::Mat tmp;

    cv::hconcat(this->camera_1->_LUT_wrap_im,this->camera_2->_LUT_wrap_im,tmp);

    this->camera_1->ReleaseLut();
    this->camera_2->ReleaseLut();

    double min,max;

    for (int i = 0; i<2; i++)
    {
        cv::minMaxLoc(tmp.row(i),&min,&max);

        tmp.row(i).convertTo(this->_LUT_wrap_im.row(i), CV_16UC1
                             ,(double)(size.width/(max-min)),(double)(- (min * (size.width/(max-min)))));
    }
}

void OmniCamera::MergeLUTSph()
{
    if(this->camera_1->_LUTsphere.empty() || this->camera_2->_LUTsphere.empty()) return;

    cv::Mat tmp;

    cv::hconcat(this->camera_1->_LUTsphere,this->camera_2->_LUTsphere,tmp);

    this->camera_1->ReleaseLut();
    this->camera_2->ReleaseLut();

    tmp.convertTo(this->_LUTsphere,CV_32FC1);
}

void OmniCamera::RescaleWrapLUT(cv::Size size)
{
    double min,max;

    cv::minMaxLoc(this->_LUT_wrap_im.row(0),&min,&max);

    this->_LUT_wrap_im.row(0).convertTo(this->_LUT_wrap_im.row(0), CV_16UC1
                         ,(double)(size.width/(max-min)),(double)(- (min * (size.width/(max-min)))));

    cv::minMaxLoc(this->_LUT_wrap_im.row(1),&min,&max);

    this->_LUT_wrap_im.row(1).convertTo(this->_LUT_wrap_im.row(1),CV_16UC1,
                         size.height/(max-min),- (min * (size.height/(max-min))));

    this->_pano = cv::Mat::zeros(size.height,size.width,this->_pano.type());
}

void OmniCamera::StitchImage(int INPAIN_FLAG)
{
    if (!this->IsInit() || this->camera_1->_Frame.empty() || this->camera_2->_Frame.empty())
    {
        return;
    }

    if (this->camera_1->_Mask.empty()) this->camera_1->_Mask = cv::Mat::ones(this->camera_1->_cameraParam.imSize.rows,this->camera_1->_cameraParam.imSize.cols,CV_8UC1);
    if (this->camera_2->_Mask.empty()) this->camera_2->_Mask = cv::Mat::ones(this->camera_2->_cameraParam.imSize.rows,this->camera_2->_cameraParam.imSize.cols,CV_8UC1);

    this->_pano = cv::Mat::zeros(this->_panoSize, 16);

    cv::Mat mask_inpaint = cv::Mat::zeros(this->_panoSize, 0);
    mask_inpaint += 255;

    int row_ind = 0;
    int col_ind = 0;

    int pix_im1 = this->camera_1->_cameraParam.imSize.cols * this->camera_1->_cameraParam.imSize.rows;

    cv::Mat im_val = this->camera_1->_Frame;

    cv::Mat im_mask = this->camera_1->_Mask;

    const cv::Vec3b *ptr_pix;
    ptr_pix = im_val.ptr<cv::Vec3b>(row_ind) + col_ind;

//    cv::Vec3b *ptr_pano;

    const uchar *ptr_mask;
    ptr_mask = im_mask.ptr<uchar>(row_ind) + col_ind;

    for (int i = 0; i < this->_LUT_wrap_im.cols; i++)
    {
        if(*ptr_mask > 0)
        {

//            ptr_pano = &this->_pano.at<cv::Vec3b>(this->_LUT_wrap_im.at<unsigned short>(1,i),this->_LUT_wrap_im.at<unsigned short>(0,i));

//            if (cv::sum(*ptr_pano)[0] > 0)
//            {
//                *ptr_pano = (*ptr_pano + *ptr_pix)/2;

//            }else{

                this->_pano.at<cv::Vec3b>(this->_LUT_wrap_im.at<unsigned short>(1,i),this->_LUT_wrap_im.at<unsigned short>(0,i)) = *ptr_pix;
//            }


            if (INPAIN_FLAG) mask_inpaint.at<uchar>(this->_LUT_wrap_im.at<unsigned short>(1,i),this->_LUT_wrap_im.at<unsigned short>(0,i)) = 0;

        }

        row_ind++;

        if (row_ind == im_val.rows)
        {
            row_ind = 0;
            col_ind++;
        }

        if (i == pix_im1-1)
        {
            im_val = this->camera_2->_Frame;
            im_mask = this->camera_2->_Mask;
            row_ind = 0;
            col_ind = 0;
        }

        ptr_pix = im_val.ptr<cv::Vec3b>(row_ind) + col_ind;
        ptr_mask = im_mask.ptr<uchar>(row_ind) + col_ind;
    }

    cv::inpaint(this->_pano,mask_inpaint,this->_pano,5,cv::INPAINT_TELEA);

}


void OmniCamera::SaveImage(const std::string &filename)
{
    cv::imwrite(filename,this->_pano);
}


void OmniCamera::ApplyBaseline()
{
    if(!this->IsInit() || this->camera_2->_LUTsphere.empty()) return;

    cv::Mat tmp = this->_extrin(cv::Rect(0,0,3,3)) * this->camera_2->_LUTsphere;

    tmp.convertTo(this->camera_2->_LUTsphere,CV_32F);
}

void OmniCamera::Rotate90roll()
{
    cv::Mat Rot90roll = cv::Mat::eye(3,3,CV_32FC1);

    float angle = -90.0;

    Rot90roll.at<float>(1,1) =  cos( angle * (mypi/180.0) ); //roll
    Rot90roll.at<float>(1,2) = -sin( angle * (mypi/180.0) );
    Rot90roll.at<float>(2,1) =  sin( angle * (mypi/180.0) );
    Rot90roll.at<float>(2,2) =  cos( angle * (mypi/180.0) );

    cv::Mat tmp = Rot90roll * this->_LUTsphere;

    tmp.convertTo(this->_LUTsphere,CV_32F); // in theorz could be removed
}

void OmniCamera::PartiallyFillMess(sensor_msgs::PointCloud &PointCloud){

    if (!this->IsInit()) return;

    if (this->_LUTsphere.empty())
    {

        if (this->camera_1->_LUTsphere.empty())
        {
            this->camera_1->Im2Sph(this->camera_1->_cameraParam.imSize.rows ,this->camera_1->_cameraParam.imSize.cols);
        }

        if (this->camera_2->_LUTsphere.empty())
        {
            this->camera_2->Im2Sph(this->camera_2->_cameraParam.imSize.rows,this->camera_2->_cameraParam.imSize.cols);
        }

        this->ApplyBaseline();

        this->MergeLUTSph();

        this->Rotate90roll(); //change later by a proper tf
    }

    if (this->_LUTsphere.empty()) return;

    PointCloud.header.frame_id = "base_link";

    PointCloud.points.resize(this->_LUTsphere.cols);

    PointCloud.channels.resize(3);

    PointCloud.channels[0].name = "r";
    PointCloud.channels[1].name = "g";
    PointCloud.channels[2].name = "b";

    PointCloud.channels[0].values.resize(this->_LUTsphere.cols);
    PointCloud.channels[1].values.resize(this->_LUTsphere.cols);
    PointCloud.channels[2].values.resize(this->_LUTsphere.cols);

    int row_ind = 0;
    int col_ind = 0;

    int pix_im1 = this->camera_1->_cameraParam.imSize.cols * this->camera_1->_cameraParam.imSize.rows;

    cv::Mat mask = this->camera_1->_Mask;

    const uchar *ptr_mask;
    ptr_mask = mask.ptr<uchar>(row_ind) + col_ind;

    for (int i = 0; i<this->_LUTsphere.cols; i++)
    {
        if (*ptr_mask > 0)
        {
            PointCloud.points[i].x = this->_LUTsphere.at<float>(0,i) ;
            PointCloud.points[i].y = this->_LUTsphere.at<float>(1,i) ;
            PointCloud.points[i].z = this->_LUTsphere.at<float>(2,i) ;
        }

        row_ind++;

        if (row_ind == this->camera_1->_cameraParam.imSize.rows)
        {
            row_ind = 0;
            col_ind++;
        }

        if (i == pix_im1-1)
        {
            mask = this->camera_2->_Mask;
            row_ind = 0;
            col_ind = 0;
        }

        ptr_mask = mask.ptr<uchar>(row_ind) + col_ind;
    }
}

void OmniCamera::MessRGBSph(sensor_msgs::PointCloud &PointCloud)
{
    if (!this->IsInit()) return;

    if (this->_LUTsphere.empty()) return;

    ros::Time timeStamp = ros::Time::now();

    PointCloud.header.stamp = timeStamp;

    int row_ind = 0;
    int col_ind = 0;

    int pix_im1 = this->camera_1->_cameraParam.imSize.cols * this->camera_1->_cameraParam.imSize.rows;

    cv::Mat im_val;

    this->camera_1->_Frame.convertTo(im_val,CV_32FC3);
    cv::Mat mask = this->camera_1->_Mask;

    const cv::Vec3f *ptr_pix;
    ptr_pix = im_val.ptr<cv::Vec3f>(row_ind) + col_ind;

    const uchar *ptr_mask;
    ptr_mask = mask.ptr<uchar>(row_ind) + col_ind;

    for (int i = 0; i<(this->_LUTsphere.cols); i++)
    {

        if (*ptr_mask > 0)
        {
            PointCloud.channels[0].values[i] = ((*ptr_pix)[2])/255.0; //r
            PointCloud.channels[1].values[i] = ((*ptr_pix)[1])/255.0; //g
            PointCloud.channels[2].values[i] = ((*ptr_pix)[0])/255.0; //b
        }

        row_ind++;

        if (row_ind == this->camera_1->_cameraParam.imSize.rows)
        {
            row_ind = 0;
            col_ind++;
        }

        if (i == (pix_im1-1))
        {
            this->camera_2->_Frame.convertTo(im_val,CV_32FC3);
            mask = this->camera_2->_Mask;
            row_ind = 0;
            col_ind = 0;
        }

        ptr_pix = im_val.ptr<cv::Vec3f>(row_ind) + col_ind;
        ptr_mask = mask.ptr<uchar>(row_ind) + col_ind;
    }
}


void OmniCamera::DownSample(int sampling_ratio)
{
    this->camera_1->DownSample(sampling_ratio);
    this->camera_2->DownSample(sampling_ratio);

    this->ApplyBaseline();

    this->MergeLUTSph();

    this->_isSampled = true;
    this->_sampling_ratio = sampling_ratio;
}

void OmniCamera::GetSphSampGrid(int bandwidth, cv::Mat &pts, bool ishemi){

    cv::Mat theta = cv::Mat::zeros(1,2*bandwidth,CV_32FC1);
    cv::Mat phi = cv::Mat::zeros(1,2*bandwidth,CV_32FC1);

    cv::Mat theta_grid, phi_grid;

    cv::MatIterator_<float> it_theta = theta.begin<float>(), it_theta_end = theta.end<float>(),
            it_phi = phi.begin<float>();

    int i = 0;

    int hemi = (ishemi) ? 1 : 2;

    for (;it_theta!=it_theta_end;it_theta++)
    {
        *it_theta = mypi*(hemi*i+1) / (4*bandwidth);

        *it_phi = mypi*i / (bandwidth);

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


void OmniCamera::GetHemiSphSampGrid(cv::Mat &pts,int bandwidth){

    GetSphSampGrid(bandwidth, pts, true);

}


void OmniCamera::SphHarSample(int bandwidth)
{
    if (!this->IsInit()) return;
    if (this->camera_1->_Frame.empty() || this->camera_2->_Frame.empty()) return;

    cv::Mat sphGrid;
    cv::Mat imGrid;
    cv::Mat subMatSph;
    cv::Mat subMatIm;

    this->GetSphSampGrid(bandwidth,sphGrid);

    std::cout << "sph grid size : "<<sphGrid.rows<<" "<<sphGrid.cols<<std::endl;
    std::cout << "sph grid : "<<sphGrid<<std::endl;

    cv::MatIterator_<float> mat_it = sphGrid.begin<float>();

    int ind = 0;

    do
    {
        mat_it++;
        ind++;
    }while(*mat_it < (mypi/2));

    mat_it = sphGrid.end<float>();

    imGrid = cv::Mat::zeros(2,sphGrid.cols,CV_32S);

    cv::Mat tmp;

    Sph2Cart(sphGrid,tmp);
    tmp.copyTo(sphGrid);
    tmp.release();

    std::cout << "sph grid size : "<<sphGrid.rows<<" "<<sphGrid.cols<<std::endl;
    std::cout << "sph grid : "<<sphGrid<<std::endl;

    subMatSph = sphGrid(cv::Rect(0,0,ind,3));
    subMatIm = imGrid(cv::Rect(0,0,ind,2));

    this->camera_1->Sph2Im(subMatSph,subMatIm);

    subMatSph = sphGrid(cv::Rect(ind,0,sphGrid.cols-ind,3));
    subMatIm = imGrid(cv::Rect(ind,0,sphGrid.cols-ind,2));

    RotateCloudPoint(subMatSph,0,180,0);

    this->camera_2->Sph2Im(subMatSph,subMatIm);

    imGrid.convertTo(this->_LUTsph_im,CV_32F);
}
