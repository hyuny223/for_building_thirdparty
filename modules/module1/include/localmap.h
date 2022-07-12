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
            std::vector<Data::KeyFrame> mvLocalMap;
            cv::Point3d mLocalPose;

        public:
            LocalMap() = default;

            void setLocalMap(Data::KeyFrame keyFrame);
            std::vector<Data::KeyFrame> getLocalMap();

            cv::Point3d getLocalPose();
            void resetLocalMap();

    };
}
