#pragma once

#include <iostream>
#include <vector>
#include "keyframe.h"
#include "opencv2/opencv.hpp"

namespace Data
{
    class LocalMap
    {
        private:
            std::vector<std::shared_ptr<Data::KeyFrame>> mvLocalMap;
            cv::Point3d mLocalPose;

        public:
            LocalMap() = default;

            void setLocalMap(std::shared_ptr<Data::KeyFrame> keyFrame);
            std::vector<std::shared_ptr<Data::KeyFrame>> getLocalMap();

            cv::Point3d getLocalPose();
            void resetLocalMap();
    };
}
