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

    std::vector<std::vector<cv::DMatch>> matches;
    matcher->knnMatch(mpPrev->getDescriptors(), mpCurr->getDescriptors(), matches, 2);

    std::vector<cv::DMatch> goodMatches;
    for (auto m : matches)
    {
        if (m[0].distance / m[1].distance < 0.6)
        {
            goodMatches.push_back(m[0]);
        }
    }

    auto framePoints_L = mpPrev->getFramePoint2d();
    auto framePoints_R = mpCurr->getFramePoint2d();

    for (int i = 0; i < goodMatches.size(); i++)
    {
        mvPrevGoodMatches.push_back(framePoints_L[goodMatches[i].queryIdx].pt);
        mvCurrGoodMatches.push_back(framePoints_R[goodMatches[i].trainIdx].pt);
    }

}

bool Similarity::computeSimilarity(const int& nFeatures)
{

    float nfFeatures = static_cast<float>(nFeatures);
    const float& nfGoodFeatures = mvPrevGoodMatches.size();

    if (nfGoodFeatures / nfFeatures< 0.15) // 비슷한게 많이 없다면 keyframe으로!
    {
        std::cout << "new KeyFrame Found!!" << std::endl;
        return true;
    }
    return false;
}
