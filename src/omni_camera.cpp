#include <omni_camera.h>


OmniCamera::OmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &cameraParamPath)
{

    this->camera_1 = new FishEye(topicsName.at(0),cameraParamPath.at(0));

    this->camera_2 = new FishEye(topicsName.at(1),cameraParamPath.at(1));

    _panoSize = cv::Size(1200,400);

    this->_init = false;
}

OmniCamera::OmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &cameraParamPath, const std::string &extrinPath)
{

    this->camera_1 = new FishEye(topicsName.at(0),cameraParamPath.at(0));

    this->camera_2 = new FishEye(topicsName.at(1),cameraParamPath.at(1));

    this->LoadCalibration(extrinPath);

    _panoSize = cv::Size(1200,400);

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

//    cv::hconcat(this->camera_1->_LUT_wrap_im,this->camera_2->_LUT_wrap_im,tmp);

    this->camera_1->_LUT_wrap_im.copyTo(this->_LUT_wrap_im);
    this->_LUT_wrap_im -= 1;

//    cv::hconcat(this->camera_1->GetLUT("Sphere"),this->camera_2->GetLUT("Sphere"),this->_LUT_wrap_im);

    this->camera_1->ReleaseLut();
    this->camera_2->ReleaseLut();

//    this->_LUT_wrap_im = cv::Mat::zeros(tmp.rows,tmp.cols,CV_16UC1);

//    double min,max;

//    cv::minMaxLoc(tmp.row(0),&min,&max);

//    std::cout<<"minmax tmp row 0 : "<<min<<" "<<max<<std::endl;

//    tmp.row(0).convertTo(this->_LUT_wrap_im.row(0), CV_16UC1
//                         ,(double)(size.width/(max-min)),(double)(- (min * (size.width/(max-min)))));

//    cv::minMaxLoc(tmp.row(1),&min,&max);

//    std::cout<<"minmax tmp row 1 : "<<min<<" "<<max<<std::endl;

//    tmp.row(1).convertTo(this->_LUT_wrap_im.row(1),CV_16UC1,
//                         size.height/(max-min),- (min * (size.height/(max-min))));

//    cv::minMaxLoc(this->_LUT_wrap_im.row(0),&min,&max);

//    std::cout<<"minmax LUT row 0 : "<<min<<" "<<max<<std::endl;

//    cv::minMaxLoc(this->_LUT_wrap_im.row(1),&min,&max);

//    std::cout<<"minmax LUT row 1 : "<<min<<" "<<max<<std::endl;
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

    this->_pano = cv::Mat::zeros(this->_panoSize, CV_8UC3);

    cv::Mat mask_inpaint = cv::Mat::zeros(this->_panoSize, CV_8U);

//    cv::MatIterator_<cv::Vec3b> it_cam;
//    cv::MatIterator_<cv::Vec3b> it_end;

//    cv::MatIterator_<double> it_mask;

    int row_ind = 0;
    int col_ind = 0;

    int pix_paint = 0;

    int pix_im1 = this->camera_1->_cameraParam.imSize.cols * this->camera_1->_cameraParam.imSize.rows;

    std::cout<<"pix_im1 : "<<pix_im1<<std::endl;

//    it_cam = this->camera_1->_Frame.col(col_ind).begin<cv::Vec3b>();

//    it_end = this->camera_1->_Frame.col(col_ind).end<cv::Vec3b>();

//    it_mask = this->camera_1->_Mask.begin<double>();

    cv::Mat im_val = this->camera_1->_Frame;

    cv::Mat im_mask = this->camera_1->_Mask;

    std::cout<<"im_val type : "<<im_val.type()<<std::endl;
    std::cout<<"mask type : "<<im_mask.type()<<std::endl;
    std::cout<<"lut type : "<<this->_LUTsphere.type()<<std::endl;

    const cv::Vec3b *ptr_pix = im_val.ptr<cv::Vec3b>(row_ind) + col_ind;

    const uchar *ptr_mask = im_mask.ptr<uchar>(row_ind) + col_ind;

    std::cout<<"within stitch : bef loop"<<std::endl;

    std::cout<<"within stitch : im_val "<< im_val.at<cv::Vec3b>(0,0) <<std::endl;
    std::cout<<"within stitch : b "<< *ptr_pix <<std::endl;
//    std::cout<<"within stitch : b "<< ptr_pix <<std::endl;


    cv::Vec3b tmpp;


    for (int i = 0; i < this->_LUT_wrap_im.cols; i++)
    {

//        std::cout<<"rows : "<<row_ind<<" cols : "<<col_ind<<std::endl;

        if(*ptr_mask > 0)
        {

            tmpp = *ptr_pix;

//            std::cout<<"within stitch : loop TMPP "<< tmpp <<std::endl;
//            std::cout<<"within stitch : loop TMPP "<< tmpp[0] <<std::endl;

//            std::cout<<"within stitch : loop ptr "<< *ptr_pix <<std::endl;



            this->_pano.at<cv::Vec3b>(this->_LUT_wrap_im.at<unsigned short>(1,i),this->_LUT_wrap_im.at<unsigned short>(0,i))[0] = tmpp[0];
            this->_pano.at<cv::Vec3b>(this->_LUT_wrap_im.at<unsigned short>(1,i),this->_LUT_wrap_im.at<unsigned short>(0,i))[1] = tmpp[1];
            this->_pano.at<cv::Vec3b>(this->_LUT_wrap_im.at<unsigned short>(1,i),this->_LUT_wrap_im.at<unsigned short>(0,i))[2] = tmpp[2];


//            this->_pano.at<cv::Vec3b>(this->_LUT_wrap_im.at<unsigned short>(1,i),this->_LUT_wrap_im.at<unsigned short>(0,i))[0] = ptr_pix[0];

//            this->_pano.at<cv::Vec3b>(this->_LUT_wrap_im.at<unsigned short>(1,i),this->_LUT_wrap_im.at<unsigned short>(0,i))[1] = ptr_pix[1];

//            this->_pano.at<cv::Vec3b>(this->_LUT_wrap_im.at<unsigned short>(1,i),this->_LUT_wrap_im.at<unsigned short>(0,i))[2] = ptr_pix[2];

//            std::cout<<"within stitch : paint "<<std::endl;

            pix_paint++;

        }else if(INPAIN_FLAG){

            mask_inpaint.at<uchar>(this->_LUT_wrap_im.at<unsigned short>(1,i),this->_LUT_wrap_im.at<unsigned short>(0,i)) = 1;
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
            std::cout<<"within stitch : load ima2 "<<std::endl;
            row_ind = 0;
            col_ind = 0;
        }

        ptr_pix = im_val.ptr<cv::Vec3b>(row_ind) + col_ind;
        ptr_mask = im_mask.ptr<uchar>(row_ind) + col_ind;


//        it_cam++;
//        it_mask++;

//        if (it_cam == it_end)
//        {
//            //std::cout<<"within stitch : end row : "<<i<<std::endl;
//            it_cam = this->camera_1->_Frame.col(col_ind += 1).begin<cv::Vec3b>();
//            it_end = this->camera_1->_Frame.col(col_ind += 1).end<cv::Vec3b>();
//            it_mask = this->camera_1->_Mask.col(col_ind += 1).begin<double>();
//        }

//        if (i == pix_im1)
//        {
//            col_ind = 0;

//            it_cam = this->camera_2->_Frame.col(col_ind).begin<cv::Vec3b>();
//            it_end = this->camera_2->_Frame.col(col_ind).end<cv::Vec3b>();
//            it_mask = this->camera_1->_Mask.col(col_ind).begin<double>();
//        }
    }

    std::cout<<"within stitch : end : pix_paint"<<pix_paint<<std::endl;
}
