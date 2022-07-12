#pragma once

#include "framepoint.h"
#include "frame.h"
#include "keyframe.h"
#include <spdlog/spdlog.h>

namespace Frontend
{
template <typename T>
void detectFeatures(T Frame_L, T Frame_R, const int& nFeatures)
{
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create(nFeatures);

    // std::vector<cv::KeyPoint> framePoints_L = Frame_L->getFramePoint2d();
    // std::vector<cv::KeyPoint> framePoints_R = Frame_R->getFramePoint2d();
    std::vector<cv::KeyPoint> framePoints_L;
    std::vector<cv::KeyPoint> framePoints_R;

    // cv::Mat descriptors_L = Frame_L->getDescriptors();
    // cv::Mat descriptors_R = Frame_R->getDescriptors();
    cv::Mat descriptors_L;
    cv::Mat descriptors_R;

    detector->detectAndCompute(Frame_L->getFrame(), cv::Mat(), framePoints_L, descriptors_L);
    detector->detectAndCompute(Frame_R->getFrame(), cv::Mat(), framePoints_R, descriptors_R);

    Frame_L->setFramePoint2d(framePoints_L);
    Frame_R->setFramePoint2d(framePoints_R);

    Frame_L->setDescriptors(descriptors_L);
    Frame_R->setDescriptors(descriptors_R);
};


template <typename T>
void matchFeatures(T Frame_L, T Frame_R)
{
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);

    std::vector<std::vector<cv::DMatch>> matches;
    matcher->knnMatch(Frame_L->getDescriptors(), Frame_R->getDescriptors(), matches, 2);

    std::vector<cv::DMatch> goodMatches;
    for (auto m : matches)
    {
        if (m[0].distance / m[1].distance < 0.8)
        {
            goodMatches.push_back(m[0]);
        }
    }

    auto framePoints_L = Frame_L->getFramePoint2d();
    auto framePoints_R = Frame_R->getFramePoint2d();

    std::vector<cv::Point2d> goodMatches_L = Frame_L->getGoodMatches();
    std::vector<cv::Point2d> goodMatches_R = Frame_R->getGoodMatches();

    for (size_t i = 0; i < goodMatches.size(); i++)
    {
        goodMatches_L.push_back(framePoints_L[goodMatches[i].queryIdx].pt);
        goodMatches_R.push_back(framePoints_R[goodMatches[i].trainIdx].pt);

    }

    Frame_L->setGoodMatches(goodMatches_L);
    Frame_R->setGoodMatches(goodMatches_R);
    

    
    cv::Mat dst;
    cv::drawMatches(Frame_L->getFrame(), Frame_L->getFramePoint2d(), Frame_R->getFrame(), Frame_R->getFramePoint2d(), matches, dst);

    cv::imshow("dst",dst);
    // cv::waitKey(1);
    cv::waitKey(1e3/20);
};


template<typename T>
cv::Mat computeTransformMat(T& R, T& t, T& transform)
{
    cv::hconcat(R, t, transform); // [R t]를 만들어 주는 과정

    double _b[] = {0,0,0,1};
    cv::Mat b(1,4,CV_64FC1,_b);

    cv::vconcat(transform, b, transform); // [[R t], [0 1]]를 만들어주는 과정
    return transform;
}


template<typename T>
void splitTransformMat(T& transform, T& R, T& t)
{
    cv::Rect roiR(0,0,3,3);
    cv::Rect roit(3,0,1,3);

    R = transform(roiR);
    t = transform(roit);
}

template <typename T>
void computeEssentialMatrix(T Frame_L, T Frame_R)
{
    double k[] = {718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1};
    cv::Mat K(3,3, CV_64FC1, k);

    auto goodMatches_L = Frame_L->getGoodMatches();
    auto goodMatches_R = Frame_R->getGoodMatches();

    Frame_L->setEssentialMat(cv::findEssentialMat(goodMatches_L, goodMatches_R, K));
    cv::Mat essentialMatrix = Frame_L->getEssentialMat();
    // cv::Mat rotationMatrix = Frame_L->getRotationMat(); // double
    // cv::Mat translationMatrix = Frame_L->getTranslationMat(); // double
    cv::Mat rotationMatrix; // double
    cv::Mat translationMatrix; // double

    cv::recoverPose(essentialMatrix, goodMatches_L, goodMatches_R, K, rotationMatrix, translationMatrix);
    spdlog::error("recoverPose complect");

    cv::Mat transformMatrix;
    computeTransformMat(rotationMatrix, translationMatrix, transformMatrix);
    spdlog::error("computeTransformMat complect");

    Frame_L->setRotationMat(rotationMatrix);
    spdlog::error("setRotationMat complect");
    Frame_L->setTranslationMat(translationMatrix); // tvec은 distance가 1인 유닛 벡터이다. 스케일이 정해지지 않음.
    spdlog::error("setTranslationMat complect");
    Frame_L->setTransformMat(transformMatrix);
    spdlog::error("setTransformMat complect");
}

template <typename T>
void computeTriangulation(T Frame_L, T Frame_R)
{
    double k[] = {718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1};
    cv::Mat K(3,3, CV_64FC1, k);

    std::vector<cv::Point3d> framePoint3d_L = Frame_L->getFramePoint3d();
    cv::Mat translationMatrix = Frame_L->getTranslationMat();

    cv::Point3d p;

    cv::Mat& t = translationMatrix;
    double baseline = sqrt(std::pow(t.ptr<double>(0)[0],2) + std::pow(t.ptr<double>(0)[1],2) + std::pow(t.ptr<double>(0)[2],2));

    for(int i = 0; i < Frame_L->getGoodMatches().size(); ++i)
    {
        auto ptr_L = Frame_L->getGoodMatches()[i];
        auto ptr_R = Frame_R->getGoodMatches()[i];

        p.z = std::abs(baseline * Frame_L->getScale() / (ptr_R.x - ptr_L.x)); // depth of point

        p.x = p.z / K.ptr<double>(0)[0] * (ptr_L.x - K.ptr<double>(0)[2]); // x of point
        p.y = p.z / K.ptr<double>(1)[1] * (ptr_L.y - K.ptr<double>(1)[2]); // y of point

        framePoint3d_L.push_back(p);
    }
    Frame_L->setFramePoint3d(framePoint3d_L);
}

template<typename T>
void computeWorldPosition(T prevKeyFrame, T currKeyFrame)
{

    // currw2c = prevw2c * relative T
    auto prevw2c = prevKeyFrame->getw2c();
    auto relativeTransform = prevKeyFrame->getTransformMat();

    auto currw2c = prevw2c * relativeTransform.t();

    currKeyFrame->setw2c(currw2c);
}
}
