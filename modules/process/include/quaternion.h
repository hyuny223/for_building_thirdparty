#pragma once

#include "quaternion.h"

#include "opencv2/opencv.hpp"
#include <cmath>


namespace Frontend
{
    //cv::Mat q is 1x4 matrix
    void R2Quaternion(std::shared_ptr<Data::KeyFrame> currKeyFrame);
    void Quaternion2R(cv::Mat& q, cv::Mat& R);
}
