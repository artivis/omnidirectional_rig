#include "fisheye.h"


FishEye::FishEye(const std::string &topicsName, const std::string &paramPath)
    : ImageHandler(topicsName)
{

    this->_init = this->_loadParam(paramPath);

}

FishEye::FishEye(const std::string &topicsName, const std::string &paramPath, const std::string &LUTSphPath)
    : ImageHandler(topicsName)
{

    this->_init = this->_loadParam(paramPath);

    this->LoadLUT(LUTSphPath,"sphere");

}


FishEye::~FishEye(){


}


bool FishEye::_loadParam(const std::string &paramPath)
{

    cv::FileStorage fs(paramPath,cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        std::cout<<"Failed to open "<<paramPath<< std::endl;
        return false;
    }

    fs["type"] >> this->_cameraParam.cameraType;
    fs["xi"] >> this->_cameraParam.xi;
    fs["K"] >> this->_cameraParam.intrinParam;

    cv::FileNode fn = fs["image_size"];

    fn["rows"] >> this->_cameraParam.imSize.rows;
    fn["cols"] >> this->_cameraParam.imSize.cols;

    fs.release();

    return true;
}

std::string FishEye::GetType(){
    return this->_cameraParam.cameraType;
}

double FishEye::GetXi(){
    return this->_cameraParam.xi;
}

cv::Mat FishEye::GetIntrinsic(){
    return this->_cameraParam.intrinParam;
}

//imageSize FishEye::Get_imageSize(){
//    return this->_cameraParam.imSize;
//}

std::vector<int> FishEye::GetImageSize(){

    std::vector<int> tmp;

    tmp.push_back(this->_cameraParam.imSize.rows);
    tmp.push_back(this->_cameraParam.imSize.cols);

    return tmp;
}

cv::Mat FishEye::GetLUT(){
    return this->_LUTsphere;
}

cv::Mat FishEye::GetLUT(const std::string &LUT ){

    std::string LUTsphere =  "Sphere";
    std::string LUTheal = "Healpix";
    std::string LUTplatte = "PlCa";

    if (LUT.compare(LUTsphere) == 0 && !this->_LUTsphere.empty())
    {

        return this->_LUTsphere;


    }else if(LUT.compare(LUTheal) == 0 && !this->_LUT_wrap_im.empty()){

        return this->_LUT_wrap_im;

    }else if(LUT.compare(LUTplatte) == 0 && !this->_LUT_wrap_im.empty()){

        return this->_LUT_wrap_im;

    }else{

        std::cout<<"Wrong LUT choice, please choose a correct option :\n"<<
                   "1 : 'Sphere' for points lying on the S2 sphere\n"<<
                   "2 : 'Healpix' for the Healpix unwrapped points\n"<<
                   "3 : 'PlCa' for Platte Carree unwrapped points"<<std::endl;

        return cv::Mat::zeros(1,1,CV_32F);
    }


}

cv::Mat FishEye::GetMask(){
    return this->_Mask;
}

cv::Mat FishEye::getImage(){
    return this->_Frame;
}

bool FishEye::IsInit(){
    return this->_init;
}

void FishEye::SetType(const std::string& type){
    this->_cameraParam.cameraType = type;
}

void FishEye::SetXi(double xi){
    this->_cameraParam.xi = xi;
}

void FishEye::SetIntrinsic(const cv::Mat &intrin){
    this->_cameraParam.intrinParam = intrin;
}

void FishEye::SetImageSize(const imageSize &imSize){
    this->_cameraParam.imSize = imSize;
}

void FishEye::SetImageSize(int rows, int cols){
    this->_cameraParam.imSize.rows = rows;
    this->_cameraParam.imSize.cols = cols;
}

void FishEye::ReleaseLut(){
    if (!this->_LUT_wrap_im.empty()) this->_LUT_wrap_im.release();
    if (!this->_LUTsphere.empty()) this->_LUTsphere.release();
}

void FishEye::DispParam()
{

    std::cout << "camera type : " << this->_cameraParam.cameraType << std::endl;

    std::cout << "camera xi   : " << this->_cameraParam.xi << std::endl;

    std::cout << "camera intrinsic parameters : \n " << this->_cameraParam.intrinParam << std::endl;

    std::cout << "image size :\n   rows : " << this->_cameraParam.imSize.rows
              << "\n   cols : " << this->_cameraParam.imSize.cols << std::endl<< std::endl;

}

bool FishEye::LoadLUT(const std::string& filename, const std::string &LUT)
{

    std::ifstream fs;

    fs.open ((char*)filename.c_str());

    if ( !fs.is_open() )
    {
        std::cout<<"Failed to open "<<filename<< std::endl;
        return false;
    }

    std::string LUTsphere =  "Sphere";
    std::string LUTheal = "Healpix";
    std::string LUTplatte = "PlCa";

    std::vector<float> tmp_vec;
    std::string num;

    if (LUT.compare(LUTsphere) == 0)
    {

        while(std::getline(fs,num,','))
        {

            tmp_vec.push_back(atof(num.c_str()));

        }

        this->_LUTsphere = Vector2Mat<float>(tmp_vec);

        this->_LUTsphere = this->_LUTsphere.reshape(0,tmp_vec.size()/3);

        this->_LUTsphere = this->_LUTsphere.t();


    }else if(LUT.compare(LUTheal)  == 0 ){

        while(std::getline(fs,num,','))
        {

            tmp_vec.push_back(atof(num.c_str()));

        }

        this->_LUT_wrap_im = Vector2Mat<float>(tmp_vec);

        this->_LUT_wrap_im = this->_LUT_wrap_im.reshape(0,tmp_vec.size()/2);

        this->_LUT_wrap_im = this->_LUT_wrap_im.t();

    }else if(LUT.compare(LUTplatte)  == 0 ){

        while(std::getline(fs,num,','))
        {

            tmp_vec.push_back(atof(num.c_str()));

        }

        this->_LUT_wrap_im = Vector2Mat<float>(tmp_vec);

        this->_LUT_wrap_im = this->_LUT_wrap_im.reshape(0,tmp_vec.size()/2);

        this->_LUT_wrap_im = this->_LUT_wrap_im.t();

    }else{

        std::cout<<"Error while loading LUT, please choose a correct option :\n"<<
                   "1 : 'Sphere' for points lying on the S2 sphere\n"<<
                   "2 : 'Healpix' for the Healpix unwrapped points\n"<<
                   "3 : 'PlCa' for Platte Carree unwrapped points"<<std::endl;

        return false;
    }

//    fs.release();

    fs.close();

    return true;
}



void FishEye::LoadMask(const std::string& maskFile){

    this->_Mask = cv::imread(maskFile,CV_LOAD_IMAGE_GRAYSCALE);

    cv::threshold(this->_Mask,this->_Mask,240,1,cv::THRESH_BINARY);
}

void FishEye::readImage(std::string file){

    ImageHandler::readImage(file,this->_Frame);

}


//void FishEye::CompLUT(const std::string&){

//    if (!this->_init)
//    {
//        std::cout<<"Please Load camera parameter prior to compute LUT"<<std::endl;
//        return;
//    }

//    if (LUT.compare(LUTsphere) == 0)
//    {
//        std::cout<<"here0";
//        fs["LUT_sph_pts"] >> this->_LUTsphere;

//    }else if(LUT.compare(LUTheal)  == 0 ){

//        fs["LUT_Healpix_pts"] >> this->_LUT_wrap_im; std::cout<<"here1";

//    }else if(LUT.compare(LUTplatte)  == 0 ){

//        fs["LUT_PlCa_pts"] >> this->_LUT_wrap_im; std::cout<<"here2";

//    }else{

//        std::cout<<"Error while loading LUT, please choose a correct option :\n"<<
//                   "1 : Sphere for points lying on the S2 sphere\n"<<
//                   "2 : Healpix for the Healpix unwrapped points\n"<<
//                   "3 : PlCa for Platte Carree unwrapped points"<<std::endl;

//        return false;
//    }

//}


//void FishEye::compSphericalCoor(){




//}





