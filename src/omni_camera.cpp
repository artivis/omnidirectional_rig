#include <omni_camera.h>

OmniCamera::OmniCamera(const std::string &packPath)
{

    std::vector<std::string> path_yamls_cam;
    std::vector<std::string> topics_name;
    std::string maskCamera_1;
    std::string maskCamera_2;
    std::string extrinParam;

    path_yamls_cam.push_back(("etc/calib/Pal_intrinsicParam_cam1.yaml"));
    path_yamls_cam.push_back(("etc/calib/Pal_intrinsicParam_cam2.yaml"));

    maskCamera_1  = ("etc/images/cam1/Img_mask1.jpg");
    maskCamera_2  = ("etc/images/cam2/Img_mask2.jpg");

    topics_name.push_back(("/right/image_raw"));
    topics_name.push_back(("/left/image_raw"));

    extrinParam = ("etc/calib/Pal_extrinsicParam.yaml");

    this->camera_1 = new FishEye(topics_name.at(0),path_yamls_cam.at(0));

    this->camera_2 = new FishEye(topics_name.at(1),path_yamls_cam.at(1));

    this->LoadCalibration(extrinParam);

    this->camera_1->LoadMask(maskCamera_1);
    this->camera_2->LoadMask(maskCamera_2);

    this->_panoSize = cv::Size(1200,400);

    this->_isSampled = false;

    this->_sampling_ratio = 1;

    this->_ind_LUTsph = 0;

    this->_init = this->camera_1->IsInit() && this->camera_2->IsInit() && true;

}


OmniCamera::OmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &cameraParamPath)
{

    this->camera_1 = new FishEye(topicsName.at(0),cameraParamPath.at(0));

    this->camera_2 = new FishEye(topicsName.at(1),cameraParamPath.at(1));

    this->_panoSize = cv::Size(1200,400);

    cv::Mat extrin = cv::Mat::zeros(3,4,CV_32F);

    extrin(cv::Rect(0,0,3,3)) = GetRotationMat(0,180,0);

    extrin.at<float>(0,3) = 0.;
    extrin.at<float>(1,3) = 0.;
    extrin.at<float>(2,3) = 0.;

    extrin.copyTo(this->_extrin,CV_32F);

    this->_isSampled = false;

    this->_sampling_ratio = 1;

    this->_ind_LUTsph = 0;

    this->_init = this->camera_1->IsInit() && this->camera_2->IsInit() && true;
}

OmniCamera::OmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &cameraParamPath, const std::string &extrinPath)
{
    this->camera_1 = new FishEye(topicsName.at(0),cameraParamPath.at(0));

    this->camera_2 = new FishEye(topicsName.at(1),cameraParamPath.at(1));

    this->LoadCalibration(extrinPath);

    this->_panoSize = cv::Size(1200,400);

    this->_isSampled = false;

    this->_sampling_ratio = 1;

    this->_ind_LUTsph = 0;

    this->_init = this->camera_1->IsInit() && this->camera_2->IsInit() && true;
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

void OmniCamera::ReadFrame()
{
    this->camera_1->ReadFrame();
    this->camera_2->ReadFrame();
}


void OmniCamera::DispParam(){

    if(this->camera_1->IsInit()) {
        this->camera_1->DispParam();
    }else{
        std::cout<<"Camera 1 not initialized !!!"<<std::endl;
    }

    if(this->camera_2->IsInit()) {
        this->camera_2->DispParam();
    }else{
        std::cout<<"Camera 2 not initialized !!!"<<std::endl;
    }

    if(this->IsInit()) {
        std::cout<<"system extrinsic parameters : \n" << this->GetExtrin() <<std::endl<<std::endl;
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

void OmniCamera::SetPanoSize(cv::Size &panoSize)
{
    this->_panoSize = panoSize;
}

void OmniCamera::SetPanoSize(int rows, int cols)
{
    this->_panoSize = cv::Size(cols,rows);
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

void OmniCamera::MergeLUTWrap(bool heal)
{
    if(this->camera_1->_LUTsphere.empty() || this->camera_2->_LUTsphere.empty())
    {
        this->camera_1->Im2Sph(this->camera_1->_cameraParam.imSize.rows,this->camera_1->_cameraParam.imSize.cols);
        this->camera_2->Im2Sph(this->camera_2->_cameraParam.imSize.rows,this->camera_2->_cameraParam.imSize.cols);

        this->ApplyBaseline();

        this->MergeLUTSph();
    }
    if (this->_LUT_wrap_im.empty())
    {
        if(heal)
        {
            this->Sph2HealPano();
        }else{
            this->Sph2Pano();
        }
    }
}

void OmniCamera::MergeLUTHeal()
{
    if(this->camera_1->_LUTsphere.empty() || this->camera_2->_LUTsphere.empty())
    {
        this->camera_1->Im2Sph(this->camera_1->_cameraParam.imSize.rows,this->camera_1->_cameraParam.imSize.cols);
        this->camera_2->Im2Sph(this->camera_2->_cameraParam.imSize.rows,this->camera_2->_cameraParam.imSize.cols);

        this->ApplyBaseline();

        this->MergeLUTSph();
    }
    if (this->_LUT_wrap_im.empty())
    {
        this->Sph2Pano();
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

void OmniCamera::StitchImage(bool INPAIN_FLAG)
{
    if (!this->IsInit() || this->camera_1->_Frame.empty() || this->camera_2->_Frame.empty() || this->_LUT_wrap_im.empty())
    {
        return;
    }

    if (this->camera_1->_Mask.empty()) this->camera_1->_Mask = cv::Mat::ones(this->camera_1->_cameraParam.imSize.rows,this->camera_1->_cameraParam.imSize.cols,CV_8UC1);
    if (this->camera_2->_Mask.empty()) this->camera_2->_Mask = cv::Mat::ones(this->camera_2->_cameraParam.imSize.rows,this->camera_2->_cameraParam.imSize.cols,CV_8UC1);

    this->_pano = cv::Mat::zeros(this->_panoSize, 16);

    cv::Mat mask_inpaint = cv::Mat::zeros(this->_panoSize, 0);
    mask_inpaint += 255;

    int pix_im1 = this->camera_1->_cameraParam.imSize.cols * this->camera_1->_cameraParam.imSize.rows;

    cv::Mat im_val = this->camera_1->_Frame;

    cv::Mat im_mask = this->camera_1->_Mask;

    int _cross_ind_row = 0;
    int _cross_ind_col = 0;

    for (int i = 0; i < this->_LUT_wrap_im.cols; i++)
    {
        if(im_mask.at<uchar>(_cross_ind_row,_cross_ind_col) > 0)
        {

            this->_pano.at<cv::Vec3b>(this->_LUT_wrap_im.at<unsigned short>(0,i),this->_LUT_wrap_im.at<unsigned short>(1,i)) =
                    im_val.at<cv::Vec3b>(_cross_ind_row,_cross_ind_col);


            if (INPAIN_FLAG) mask_inpaint.at<uchar>(this->_LUT_wrap_im.at<unsigned short>(0,i),this->_LUT_wrap_im.at<unsigned short>(1,i)) = 0;
        }
        _cross_ind_row++;

        if (_cross_ind_row == im_val.rows)
        {
            _cross_ind_row = 0;
            _cross_ind_col++;
        }

        if (i == pix_im1-1)
        {
            im_val = this->camera_2->_Frame;
            im_mask = this->camera_2->_Mask;
            _cross_ind_row = 0;
            _cross_ind_col = 0;
        }
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

//    cv::Mat tt = GetRotationMat(0,180,0);
//    tt.convertTo(tt,CV_32F);

    cv::Mat tmp = this->_extrin(cv::Rect(0,0,3,3)) * this->camera_2->_LUTsphere; //

    tmp.convertTo(this->camera_2->_LUTsphere,CV_32F);
}

void OmniCamera::Rotate90roll()
{
    cv::Mat Rot90roll = GetRotationMat(-90,0,90);

    Rot90roll.convertTo(Rot90roll,this->_LUTsphere.type());

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


void OmniCamera::SampSphFct(cv::Mat &sampFct, int bandwidth)
{
    if (!this->IsInit()) return;
    if (this->_pano.empty()) return;

    cv::Mat tmp;
    double min, max;
    cv::Size gridSize;

    gridSize.height = bandwidth*2;
    gridSize.width = bandwidth*2;

    cv::cvtColor(this->_pano,tmp,CV_BGR2GRAY);

    tmp.convertTo(tmp ,CV_64F);

    cv::minMaxLoc(tmp,&min,&max);

    tmp -= min;
    tmp /= max;

    tmp /= cv::norm(tmp);

    cv::resize(tmp,sampFct,gridSize);
}


void OmniCamera::Sph2Pano()
{
    if (!this->IsInit()) return;
    if (this->_LUTsphere.empty()) return;

    double min,max;

    cv::Mat tmp;

    this->Rotate90roll();

    Cart2Sph(this->_LUTsphere, tmp);

    this->_LUT_wrap_im = cv::Mat::zeros(tmp.size(),CV_16UC1);

    cv::minMaxLoc(tmp.row(0),&min,&max);

    tmp.row(0).convertTo(this->_LUT_wrap_im.row(0), CV_16UC1
                         ,(double)((this->_panoSize.height-1)/(max-min)),(double)(- (min * ((this->_panoSize.height-1)/(max-min)))));

    cv::minMaxLoc(tmp.row(1),&min,&max);

    tmp.row(1).convertTo(this->_LUT_wrap_im.row(1), CV_16UC1
                         ,(double)((this->_panoSize.width-1)/(max-min)),(double)(- (min * ((this->_panoSize.width-1)/(max-min)))));
}

void OmniCamera::Sph2HealPano()
{
    if (!this->IsInit()) return;
    if (this->_LUTsphere.empty()) return;

    double min,max;

    cv::Mat tmp, tmp2;

    this->Rotate90roll();

    Cart2Sph(this->_LUTsphere, tmp);

    Sph2Heal(tmp,tmp2);

    tmp.release();

    this->_LUT_wrap_im = cv::Mat::zeros(tmp2.size(),CV_16UC1);

    cv::minMaxLoc(tmp2.row(0),&min,&max);

    tmp2.row(0).convertTo(this->_LUT_wrap_im.row(0), CV_16UC1
                          ,(double)((this->_panoSize.height-1)/(max-min)),(double)(- (min * ((this->_panoSize.height-1)/(max-min)))));

    cv::minMaxLoc(tmp2.row(1),&min,&max);

    tmp2.row(1).convertTo(this->_LUT_wrap_im.row(1), CV_16UC1
                          ,(double)((this->_panoSize.width-1)/(max-min)),(double)(- (min * ((this->_panoSize.width-1)/(max-min)))));

}
