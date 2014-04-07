#include <omni_camera.h>


OmniCamera::OmniCamera(const std::vector<std::string> &topicsName, const std::vector<std::string> &paramPath)
{

    this->camera_1 = new FishEye(topicsName.at(0),paramPath.at(0));

    this->camera_2 = new FishEye(topicsName.at(1),paramPath.at(1));

}

void OmniCamera::initCamera(int cameraNum, const std::string &topicName, const std::string &paramPath){

    switch (cameraNum){

        case 1:
            if (this->camera_1->isInit())
            {
                this->camera_1->~FishEye();
            }

            this->camera_1 = new FishEye(topicName, paramPath);

        case 2:
            if (this->camera_2->isInit())
            {
                this->camera_2->~FishEye();
            }

            this->camera_2 = new FishEye(topicName, paramPath);
    }

}

