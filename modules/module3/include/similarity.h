#pragma once

#include <iostream>
#include <string>

#include "opencv2/opencv.hpp"
#include "frame.h"
#include "framepoint.h"
#include "keyframe.h"
#include "tracking.h"


class Similarity
{
    protected:
        std::shared_ptr<Data::KeyFrame> mpPrev;
        std::shared_ptr<Data::Frame> mpCurr;
        std::vector<cv::Point2d> mvPrevGoodMatches, mvCurrGoodMatches;

    public:
        Similarity() = default;
        Similarity(const std::shared_ptr<Data::KeyFrame> prev, std::shared_ptr<Data::Frame> curr);
        // ~Similarity();

        void findSimFeatures(const std::string& mode);
        bool computeSimilarity(const int& nFeatures);
};
