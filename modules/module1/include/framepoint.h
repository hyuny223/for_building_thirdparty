#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"

namespace Data
{
    class FramePoint
    {
        protected:
            std::vector<cv::Point3d> mvFramePoints3d;
            std::vector<cv::KeyPoint> mvFramePoints2d;

        public:
            FramePoint() = default;

            void setFramePoint3d(std::vector<cv::Point3d> framePoints);
            std::vector<cv::Point3d> getFramePoint3d();


            void setFramePoint2d(std::vector<cv::KeyPoint> framePoints);
            std::vector<cv::KeyPoint> getFramePoint2d();

    };
}
