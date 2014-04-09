#ifndef FISHEYE_H
#define FISHEYE_H

#include <vector>
#include <string>
#include <iostream>
#include <fstream>


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "image_handler.h"
#include "usefull.h"

struct imageSize{

    int rows;
    int cols;

};

struct CameraParam{

    std::string cameraType;

    double xi;

    cv::Mat intrinParam;

    imageSize imSize;

};


class FishEye : public pal::slam::ImageHandler
{
public:

    FishEye();
    FishEye(const std::string &topicsName, const std::string &paramPath);

    FishEye(const std::string &topicsName,
            const std::string &paramPath,
            const std::string &LUTSphPath);

    ~FishEye();

//    friend class OmniCamera;

    std::string GetType();
    double GetXi();
    cv::Mat GetIntrinsic();
//    imageSize Get_imageSize();
    std::vector<int> GetImageSize();
    cv::Mat GetLUT();
    cv::Mat GetLUT(const std::string &);
    cv::Mat GetMask();
    cv::Mat getImage();

    bool IsInit();

    void SetType(const std::string&);
    void SetXi(double);
    void SetIntrinsic(const cv::Mat&);
    void SetImageSize(const imageSize&);
    void SetImageSize(int rows, int cols);

    void ReleaseLut();

    void DispParam();

    void LoadMask(const std::string&);

    bool LoadLUT(const std::string&,const std::string&);
//    void CompLUT(const std::string&);

    void readImage(std::string file);



//private :


    CameraParam _cameraParam;

    bool _init;

    cv::Mat _LUTsphere;
    cv::Mat _LUT_wrap_im;

    cv::Mat _Mask;
    cv::Mat _Frame;

    bool _loadParam(const std::string &paramPath);

//    void _compSphericalCoor();

};



#endif // FISHEYE_H
