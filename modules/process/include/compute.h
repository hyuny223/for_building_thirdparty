#pragma once

#include <iostream>
#include <string>
#include "framepoint.h"
#include "frame.h"
#include "keyframe.h"
#include <spdlog/spdlog.h>
#include <cmath>

namespace Frontend
{
template <typename T>
void detectFeatures(T Frame_L, T Frame_R, const std::string& mode="AKAZE", const int& nFeatures = 0)
{
    cv::Ptr<cv::Feature2D> detector;
    if(mode == "SIFT")
    {
        detector = cv::SIFT::create();

    }
    else if(mode == "ORB")
    {
        detector = cv::ORB::create(nFeatures);

    }
    else
    {
        detector = cv::AKAZE::create();
    }

    std::vector<cv::KeyPoint> framePoints_L;
    std::vector<cv::KeyPoint> framePoints_R;

    cv::Mat descriptors_L;
    cv::Mat descriptors_R;

    detector->detectAndCompute(Frame_L->getFrame(), cv::Mat(), framePoints_L, descriptors_L);
    detector->detectAndCompute(Frame_R->getFrame(), cv::Mat(), framePoints_R, descriptors_R);
    spdlog::info("- detectAndCompute complete");

    Frame_L->setFramePoint2d(framePoints_L);
    Frame_R->setFramePoint2d(framePoints_R);
    spdlog::info("- setFramePoint2d complete");

    Frame_L->setDescriptors(descriptors_L);
    Frame_R->setDescriptors(descriptors_R);
    spdlog::info("- setDescriptors complete");
};


template <typename T>
void matchFeatures(T Frame_L, T Frame_R, const std::string& mode)
{
    cv::Ptr<cv::DescriptorMatcher> matcher;
    if (mode == "SITF" || mode == "ORB")
    {
        matcher = cv::BFMatcher::create(cv::NORM_L2);
    }
    else
    {
        matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    }

    std::vector<std::vector<cv::DMatch>> matches;
    matcher->knnMatch(Frame_L->getDescriptors(), Frame_R->getDescriptors(), matches, 2);
    spdlog::info("- knnMatch complete");

    std::vector<cv::DMatch> goodMatches;
    for (auto m : matches)
    {
        if (m[0].distance / m[1].distance < 0.5)
        {
            goodMatches.push_back(m[0]);
        }
    }

    auto framePoints_L = Frame_L->getFramePoint2d();
    auto framePoints_R = Frame_R->getFramePoint2d();

    std::vector<cv::Point2d> goodMatches_L = {};
    std::vector<cv::Point2d> goodMatches_R = {};


    for (size_t i = 0; i < goodMatches.size(); i++)
    {
        goodMatches_L.push_back(framePoints_L[goodMatches[i].queryIdx].pt);
        goodMatches_R.push_back(framePoints_R[goodMatches[i].trainIdx].pt);
    }

    Frame_L->setGoodMatches(goodMatches_L);
    Frame_R->setGoodMatches(goodMatches_R);
    spdlog::info("- setGoodMatches complete");

    // cv::Mat dst;
    // cv::drawMatches(Frame_L->getFrame(), framePoints_L, Frame_R->getFrame(), framePoints_R, goodMatches, dst);
    // spdlog::info("- drawMatches complete");

    // cv::imshow("dst",dst);
    // // cv::waitKey(0);
    // // spdlog::warn("- waitKey(0)");
    // cv::waitKey(1e3/20);
    // // spdlog::warn("- waitKey(1e3/20)");

};

template<typename T>
void computeTransformMat(T& R, T& t, T& transform)
{
    cv::hconcat(R, t, transform); // [R t]를 만들어 주는 과정

    double _b[] = {0,0,0,1};
    cv::Mat b(1,4,CV_64FC1,_b);

    cv::vconcat(transform, b, transform); // [[R t], [0 1]]를 만들어주는 과정
}

template <typename T>
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


    Frame_R->setEssentialMat(cv::findEssentialMat(goodMatches_L, goodMatches_R, K));
    cv::Mat essentialMatrix = Frame_R->getEssentialMat();
    spdlog::info("- setEssentialMat complete");

    cv::Mat rotationMatrix;
    cv::Mat translationMatrix;

    cv::recoverPose(essentialMatrix, goodMatches_L, goodMatches_R, K, rotationMatrix, translationMatrix);
    spdlog::info("- recoverPose complete");

    cv::Mat transformMatrix;
    computeTransformMat(rotationMatrix, translationMatrix, transformMatrix);

    spdlog::info("- computeTransformMat complete");

    Frame_R->setRotationMat(rotationMatrix);
    spdlog::info("- setRotationMat complete");
    Frame_R->setTranslationMat(translationMatrix); // tvec은 distance가 1인 유닛 벡터이다. 스케일이 정해지지 않음.
    spdlog::info("- setTranslationMat complete");
    Frame_R->setTransformMat(transformMatrix);
    spdlog::info("- setTransformMat complete");
}

template <typename T>
void computeTriangulation(T Frame_L, T Frame_R)
{
    double k[] = {718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1};
    cv::Mat K(3,3, CV_64FC1, k);

    std::vector<cv::Point3d> framePoint3d_L = {};
    cv::Mat translationMatrix = Frame_R->getTranslationMat();

    cv::Point3d p;

    cv::Mat& t = translationMatrix;

    std::vector<cv::Point2d> ptrVec_L = Frame_L->getGoodMatches();
    std::vector<cv::Point2d> ptrVec_R = Frame_R->getGoodMatches();

    // double baseline = sqrt(std::pow(t.ptr<double>(0)[0],2) + std::pow(t.ptr<double>(1)[0],2) + std::pow(t.ptr<double>(2)[0],2));
    double baseline = 0.54;

    for(int i = 0; i < ptrVec_L.size() ; ++i)
    {
        auto ptr_L = ptrVec_L[i];
        auto ptr_R = ptrVec_R[i];

        auto delta_x = -(ptr_R.x - ptr_L.x);

        if(delta_x == 0)
        {
            ptrVec_L.erase(ptrVec_L.begin() + i);
            ptrVec_R.erase(ptrVec_R.begin() + i);
            --i;
            continue;
        }
        else
        {
            p.z = (baseline / delta_x) * K.ptr<double>(0)[0];
        }
         // depth of point
        p.x = (ptr_L.x - K.ptr<double>(0)[2]) * p.z / K.ptr<double>(0)[0] ; // x of point
        p.y = (ptr_L.y + ptr_R.y) / 2 * p.z / K.ptr<double>(0)[0]; // y of point

        framePoint3d_L.emplace_back(p);
    }
    Frame_L->setFramePoint3d(framePoint3d_L);
    Frame_L->setGoodMatches(ptrVec_L);
    Frame_R->setGoodMatches(ptrVec_R);
}

template<typename T>
void computeWorldPosition(T prevKeyFrame, T currKeyFrame)
{
    std::vector<cv::Point3d> vFramePoint3d = prevKeyFrame->getFramePoint3d();
    cv::Mat transformMat = currKeyFrame->getTransformMat();
    /* 월드포지션(카메라) */
    cv::Point3d pt = prevKeyFrame->getWorldPosition();

    double world_x = transformMat.ptr<double>(0)[0]*pt.x + transformMat.ptr<double>(0)[1]*pt.y + transformMat.ptr<double>(0)[2]*pt.z + transformMat.ptr<double>(0)[3];
    double world_y = transformMat.ptr<double>(1)[0]*pt.x + transformMat.ptr<double>(1)[1]*pt.y + transformMat.ptr<double>(1)[2]*pt.z + transformMat.ptr<double>(1)[3];
    double world_z = transformMat.ptr<double>(2)[0]*pt.x + transformMat.ptr<double>(2)[1]*pt.y + transformMat.ptr<double>(2)[2]*pt.z + transformMat.ptr<double>(2)[3];

    std::cout << "camera world position : (" << world_x << ", " << world_y << ", " << world_z << ")\n\n\n\n";

    cv::Point3d worldPosition(world_x, world_y, world_z);

    currKeyFrame->setWorldPosition(worldPosition);


    // cv::Mat w2cTransformMat = currKeyFrame->getTransformMat() * prevKeyFrame->getWorld2CamTransformMat();

    // cv::Mat R(3,3,CV_64F), t(3,1,CV_64F);
    // Frontend::splitTransformMat(w2cTransformMat,R,t);
    // currKeyFrame->setWorld2CamTransformMat(w2cTransformMat);


    // R = R.inv();

    // std::vector<cv::Point3d> vWorldPoint3d = {};
    // for(auto& c : vFramePoint3d)
    // {
    //     auto x = c.x;
    //     auto y = c.y;
    //     auto z = c.z;

    //     double world_x = R.ptr<double>(0)[0] * x + R.ptr<double>(0)[1] * y + R.ptr<double>(0)[2] * z - t.ptr<double>(0)[0];
    //     double world_y = R.ptr<double>(1)[0] * x + R.ptr<double>(1)[1] * y + R.ptr<double>(1)[2] * z - t.ptr<double>(0)[1];
    //     double world_z = R.ptr<double>(2)[0] * x + R.ptr<double>(2)[1] * y + R.ptr<double>(2)[2] * z - t.ptr<double>(0)[2];

    //     cv::Point3d p(world_x,world_y,world_z);
    //     vWorldPoint3d.push_back(p);
    // }
    // prevKeyFrame->setFramePoint3d(vWorldPoint3d);
}
}
