#pragma once

#include "opencv2/opencv.hpp"
#include "frame.h"
#include "framepoint.h"
#include "keyframe.h"
#include "tracking.h"


class Similarity
{
    protected:
        std::shared_ptr<Data::Frame> mpPrev, mpCurr;

        std::vector<cv::Point2d> mvPrevGoodMatches, mvCurrGoodMatches;

    public:
        Similarity() = default;
        Similarity(const std::shared_ptr<Data::Frame> prev, std::shared_ptr<Data::Frame> curr);
        // ~Similarity();

        void findSimFeatures();
        bool computeSimilarity(const int& nFeatures);
};
