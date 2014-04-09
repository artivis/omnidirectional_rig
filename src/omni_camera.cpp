#include <omni_camera.h>


OmniCamera::OmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &cameraParamPath)
{

    this->camera_1 = new FishEye(topicsName.at(0),cameraParamPath.at(0));

    this->camera_2 = new FishEye(topicsName.at(1),cameraParamPath.at(1));

    this->_init = false;

    _panoSize = cv::Size(1200,400);

}

OmniCamera::OmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &cameraParamPath, const std::string &extrinPath)
{

    this->camera_1 = new FishEye(topicsName.at(0),cameraParamPath.at(0));

    this->camera_2 = new FishEye(topicsName.at(1),cameraParamPath.at(1));

    this->LoadCalibration(extrinPath);

    this->_init = true;

}

//OmniCamera::OmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &cameraParamPath,
//           const std::string &extrinPath, const std::vector<std::string> &LUTpath)
//{

//    this->camera_1 = new FishEye(topicsName.at(0),cameraParamPath.at(0));

//    this->camera_2 = new FishEye(topicsName.at(1),cameraParamPath.at(1));

//    this->LoadCalibration(extrinPath);

//    this->LoadLUT(LUTpath);

//    this->_init = true;
//}

OmniCamera::~OmniCamera(){

    delete this->camera_1;
    delete this->camera_2;
}

//void OmniCamera::InitCamera(int cameraNum, const std::string &topicName, const std::string &paramPath){

//    switch (cameraNum){

//        case 1:
//            if (this->camera_1->IsInit())
//            {
//                this->camera_1->~FishEye();
//            }

//            this->camera_1 = new FishEye(topicName, paramPath);

//        case 2:
//            if (this->camera_2->IsInit())
//            {
//                this->camera_2->~FishEye();
//            }

//            this->camera_2 = new FishEye(topicName, paramPath);
//    }

//}

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

void OmniCamera::MergeLUT(cv::Size size)
{
    cv::Mat tmp;

    cv::hconcat(this->camera_1->_LUT_wrap_im,this->camera_2->_LUT_wrap_im,tmp);

//    cv::hconcat(this->camera_1->GetLUT("Sphere"),this->camera_2->GetLUT("Sphere"),this->_LUT_wrap_im);

    this->camera_1->ReleaseLut();
    this->camera_2->ReleaseLut();

    this->_LUT_wrap_im = cv::Mat::zeros(tmp.rows,tmp.cols,CV_8SC1);

    double min,max;

    cv::minMaxLoc(tmp.row(0),&min,&max);

    std::cout<<"minmax tmp row 0 : "<<min<<" "<<max<<std::endl;

    tmp.row(0).convertTo(this->_LUT_wrap_im.row(0), CV_8SC1,size.height/(max-min),-size.height/min);

    cv::minMaxLoc(tmp.row(1),&min,&max);

    std::cout<<"minmax tmp row 1 : "<<min<<" "<<max<<std::endl;

    tmp.row(1).convertTo(this->_LUT_wrap_im.row(1),CV_8SC1,size.width/(max-min),-size.width/min);

    cv::minMaxLoc(this->_LUT_wrap_im.row(0),&min,&max);

    std::cout<<"minmax LUT row 0 : "<<min<<" "<<max<<std::endl;

    cv::minMaxLoc(this->_LUT_wrap_im.row(1),&min,&max);

    std::cout<<"minmax LUT row 1 : "<<min<<" "<<max<<std::endl;
}

void OmniCamera::RescaleWrapLUT(cv::Size size)
{
//    double min,max;
//    cv::Mat tmp;

//    tmp = this->_LUT_wrap_im.row(0);

//    cv::minMaxLoc(tmp,&min,&max);

//    std::cout<< " within rescale : before conv"<<std::endl;cv::waitKey(100);

//    this->_LUT_wrap_im.row(0).convertTo(this->_LUT_wrap_im.row(0),CV_8SC1,size.width / (max-min),-size.width / min);

//    this->_LUT_wrap_im.row(0) = this->_LUT_wrap_im.row(0) + abs(*min);
//    this->_LUT_wrap_im.row(0) = this->_LUT_wrap_im.row(0) / abs(*max);

//    cv::minMaxIdx(this->_LUT_wrap_im.row(1),&min,&max);

//    this->_LUT_wrap_im.row(1).convertTo(this->_LUT_wrap_im.row(1),CV_8SC1,size.width / (max-min),-size.width / min);

//    this->_LUT_wrap_im.row(1) = this->_LUT_wrap_im.row(1) + abs(*min);
//    this->_LUT_wrap_im.row(1) = this->_LUT_wrap_im.row(1) / abs(*max);

//    this->_LUT_wrap_im.row(0) = (int)(_LUT_wrap_im.row(0) * size.height);
//    this->_LUT_wrap_im.row(1) = (int)(_LUT_wrap_im.row(1) * size.width);
}

void OmniCamera::StitchImage(int INPAIN_FLAG)
{
    if (!this->IsInit() || this->camera_1->_Frame.empty() || this->camera_2->_Frame.empty())
    {
        return;
    }

    double pix_cam1 = this->camera_1->GetImageSize().at(0) * this->camera_1->GetImageSize().at(1);

    this->RescaleWrapLUT();

    std::cout<< " within stitch : after rescale "<<std::endl;

    this->_pano = cv::Mat::zeros(this->_panoSize, CV_32FC3);

    cv::Mat mask_inpaint = cv::Mat::zeros(this->_panoSize, CV_8U);

    cv::MatIterator_<cv::Vec3d> it_cam;

    cv::MatIterator_<double> it_mask;

    it_cam = this->camera_1->_Frame.begin<cv::Vec3d>();

    it_mask = this->camera_1->_Mask.begin<double>();

    for (int i = 0; i < this->_LUT_wrap_im.cols;i++)
    {
        if(*it_mask == 1)
        {
            this->_pano.at<cv::Vec3d>(this->_LUT_wrap_im.at<double>(1,i),this->_LUT_wrap_im.at<double>(0,i))[0] = (*it_cam)[0];

            this->_pano.at<cv::Vec3d>(this->_LUT_wrap_im.at<double>(1,i),this->_LUT_wrap_im.at<double>(0,i))[1] = (*it_cam)[1];

            this->_pano.at<cv::Vec3d>(this->_LUT_wrap_im.at<double>(1,i),this->_LUT_wrap_im.at<double>(0,i))[2] = (*it_cam)[2];
        }else if(INPAIN_FLAG){
            mask_inpaint.at<double>(this->_LUT_wrap_im.at<double>(1,i),this->_LUT_wrap_im.at<double>(0,i)) = 1;
        }

        it_cam++;
        it_mask++;

        if (i == pix_cam1)
        {
            it_cam = this->camera_2->_Frame.begin<cv::Vec3d>();
            it_mask = this->camera_2->_Mask.begin<double>();
        }
    }
}
