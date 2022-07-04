#pragma once
// cv::Point3_<_Tp>::Point3_(_Tp _x, _Tp _y, _Tp _z)
#include <iostream>
#include "opencv2/opencv.hpp"

namespace Data
{
    class FramePoint
    {
        protected:
            std::vector<cv::Point3f> mvFramePoints3d;
            std::vector<cv::KeyPoint> mvFramePoints2d;

        public:
            FramePoint() = default;

            void setFramePoint3d(std::vector<cv::Point3f> framePoints);
            std::vector<cv::Point3f> getFramePoint3d();


            void setFramePoint2d(std::vector<cv::KeyPoint> framePoints);
            std::vector<cv::KeyPoint> getFramePoint2d();

    };
}
