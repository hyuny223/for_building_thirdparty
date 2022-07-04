#include <tuple>

#include "opencv2/opencv.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include "similarity.h"

Similarity::Similarity(const std::shared_ptr<Data::Frame> prev, const std::shared_ptr<Data::Frame> curr)
: mpPrev(prev), mpCurr(curr){};


void Similarity::findSimFeatures()
{
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);

    std::vector<cv::DMatch> matches;

    matcher->match(mpPrev->getDescriptors(), mpCurr->getDescriptors(), matches);

    std::sort(matches.begin(), matches.end());

    // std::vector<cv::DMatch> goodMatches(matches.begin(), matches.end());

    auto framePoints_L = mpPrev->getFramePoint2d();
    auto framePoints_R = mpCurr->getFramePoint2d();

    for (size_t i = 0; i < matches.size(); i++)
    {
        mvPrevGoodMatches.push_back(framePoints_L[matches[i].queryIdx].pt);
        mvCurrGoodMatches.push_back(framePoints_R[matches[i].trainIdx].pt);
    }
}

bool Similarity::computeSimilarity()
{
    const int&& nDiscriptorAvg = (mpPrev->getDescriptors().rows + mpCurr->getDescriptors().rows) / 2;
    const int& goodFeatures = mvPrevGoodMatches.size();

    if (goodFeatures / nDiscriptorAvg < 0.3) // 비슷한게 많이 없다면 keyframe으로!
    {
        return true;
    }
    return false;
}
