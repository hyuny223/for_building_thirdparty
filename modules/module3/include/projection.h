#pragma once

#include "opencv2/opencv.hpp"
#include "keyframe.h"

void doProjection(std::shared_ptr<Data::KeyFrame> keyFrame); // 3차원 좌표, rvec, tvec, intrinsic
