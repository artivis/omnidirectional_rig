#include <omni_camera.h>


OmniCamera::OmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &paramPath)
{

    this->camera_1 = new FishEye(topicsName.at(0),paramPath.at(0));

    this->camera_2 = new FishEye(topicsName.at(1),paramPath.at(1));

    this->_init = false;

}

OmniCamera::OmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &paramPath, const std::string &extrinPath)
{

    this->camera_1 = new FishEye(topicsName.at(0),paramPath.at(0));

    this->camera_2 = new FishEye(topicsName.at(1),paramPath.at(1));

    this->LoadCalibration(extrinPath);

    this->_init = true;

}

void OmniCamera::InitCamera(int cameraNum, const std::string &topicName, const std::string &paramPath){

    switch (cameraNum){

        case 1:
            if (this->camera_1->IsInit())
            {
                this->camera_1->~FishEye();
            }

            this->camera_1 = new FishEye(topicName, paramPath);

        case 2:
            if (this->camera_2->IsInit())
            {
                this->camera_2->~FishEye();
            }

            this->camera_2 = new FishEye(topicName, paramPath);
    }

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

    fs["extrinsicParam"] >> this->baseline;

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
        std::cout<<"system extrinsic parameters : \n" << this->baseline<<std::endl<<std::endl;
    }
}
