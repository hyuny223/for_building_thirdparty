#pragma once

#include "opencv2/opencv.hpp"
#include "frame.h"
#include "framepoint.h"
#include "keyframe.h"
#include "tracking.h"


class Similarity
{
    protected:
        std::shared_ptr<Data::Frame> mpPrev;
        std::shared_ptr<Data::Frame> mpCurr;

        std::vector<cv::Point2f> mvPrevGoodMatches, mvCurrGoodMatches;

    public:
        Similarity() = default;
        Similarity(const std::shared_ptr<Data::Frame> prev, std::shared_ptr<Data::Frame> curr);
        // ~Similarity();

        void findSimFeatures();
        bool computeSimilarity();
};
