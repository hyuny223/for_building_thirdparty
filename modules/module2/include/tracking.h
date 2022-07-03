#pragma once

#include "datatype.h"


namespace Frontend
{
    class Tracking
    {

        protected:
            // std::shared_ptr<Data::FramePoint> framePoint = std::make_shared<Data::FramePoint>();
            // int miFeatures;

        public:
            Tracking() = default;
            std::shared_ptr<Data::FramePoint> framePoint = std::make_shared<Data::FramePoint>();

            void detectFeatures(const cv::Mat& gray_L, const cv::Mat& gray_R, const int& nfeatures);
            void matchFeatures(const cv::Mat& gray_L, const cv::Mat& gray_R);
            void computeFundamentalMatrix();
            void computeTriangulation(const cv::Point2f& ptr_L, const cv::Point2f& ptr_R);
    };
}

