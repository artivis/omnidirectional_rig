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

std::string FishEye::Get_type(){
    return this->_cameraParam.cameraType;
}

double FishEye::Get_xi(){
    return this->_cameraParam.xi;
}

cv::Mat FishEye::Get_intrinsic(){
    return this->_cameraParam.intrinParam;
}

//imageSize FishEye::Get_imageSize(){
//    return this->_cameraParam.imSize;
//}

std::vector<int> FishEye::Get_imageSize(){

    std::vector<int> tmp;

    tmp.push_back(this->_cameraParam.imSize.rows);
    tmp.push_back(this->_cameraParam.imSize.cols);

    return tmp;
}

bool FishEye::isInit(){
    return this->_init;
}

void FishEye::Set_type(const std::string& type){
    this->_cameraParam.cameraType = type;
}

void FishEye::Set_xi(double xi){
    this->_cameraParam.xi = xi;
}

void FishEye::Set_intrinsic(const cv::Mat &intrin){
    this->_cameraParam.intrinParam = intrin;
}

void FishEye::Set_imageSize(const imageSize &imSize){
    this->_cameraParam.imSize = imSize;
}

void FishEye::Set_imageSize(int rows, int cols){
    this->_cameraParam.imSize.rows = rows;
    this->_cameraParam.imSize.cols = cols;
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
{std::cout<<"here0";
    cv::FileStorage fs(filename,cv::FileStorage::READ);std::cout<<"here0";

    if (!fs.isOpened())
    {
        std::cout<<"Failed to open "<<filename<< std::endl;
        return false;
    }

    std::string LUTsphere =  "Sphere";
    std::string LUTheal = "Healpix";
    std::string LUTplatte = "PlatteCarree";

    if (LUT.compare(LUTsphere) == 0)
    {
std::cout<<"here0";
        fs["LUT_sph_pts"] >> this->_LUTsphere;

    }else if(LUT.compare(LUTheal)  == 0 ){

        fs["LUT_Healpix_pts"] >> this->_LUT_wrap_im; std::cout<<"here1";

    }else if(LUT.compare(LUTplatte)  == 0 ){

        fs["LUT_PlCa_pts"] >> this->_LUT_wrap_im; std::cout<<"here2";

    }else{

        std::cout<<"Error while loading LUT, please choose a correct option :\n"<<
                   "1 : Sphere for points lying on the S2 sphere\n"<<
                   "2 : Healpix for the Healpix unwrapped points\n"<<
                   "3 : PlCa for Platte Carree unwrapped points"<<std::endl;

        return false;
    }

    fs.release();

    return true;

}





