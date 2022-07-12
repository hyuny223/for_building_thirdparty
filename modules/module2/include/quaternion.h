#pragma once

#include "quaternion.h"

#include "opencv2/opencv.hpp"
#include <cmath>

//cv::Mat q is 1x4 matrix
void R2Quaternion(cv::Mat& R, cv::Mat& q);
void Quaternion2R(cv::Mat& q, cv::Mat& R);