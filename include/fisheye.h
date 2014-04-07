#ifndef FISHEYE_H
#define FISHEYE_H

#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include "yaml-cpp/yaml.h"

#include <image_handler.h>

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

    std::string Get_type();
    double Get_xi();
    cv::Mat Get_intrinsic();
//    imageSize Get_imageSize();
    std::vector<int> Get_imageSize();

    bool isInit();

    void Set_type(const std::string&);
    void Set_xi(double);
    void Set_intrinsic(const cv::Mat&);
    void Set_imageSize(const imageSize&);
    void Set_imageSize(int rows, int cols);

    void DispParam();

    bool LoadLUT(const std::string&,const std::string&);

private :

    bool _loadParam(const std::string &paramPath);

    CameraParam _cameraParam;

    bool _init;

    cv::Mat _LUTsphere;

    cv::Mat _LUT_wrap_im;
};



#endif // FISHEYE_H
