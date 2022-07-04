#pragma once

#include "framepoint.h"
#include "frame.h"


namespace Frontend
{
    class Tracking
    {

        protected:
            std::shared_ptr<Data::Frame> mpFrame_L;
            std::shared_ptr<Data::Frame> mpFrame_R;

        public:
            Tracking() = default;
            Tracking(const std::shared_ptr<Data::Frame> frame_L,
                    const std::shared_ptr<Data::Frame> frame_R);
            // ~Tracking();

            void detectFeatures(const int& nFeatures);
            void matchFeatures();
            void computeEssentialMatrix();
            void computeTriangulation(const cv::Point2f& ptr_L, const cv::Point2f& ptr_R);
    };
}
