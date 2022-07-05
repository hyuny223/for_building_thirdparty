
/***************************************************************************************************
#include <iostream>
#include <cmath>

#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include "tracking.h"
#include "framepoint.h"
#include "frame.h"


namespace Frontend
{

    Tracking::Tracking(std::shared_ptr<Data::Frame> frame_L, std::shared_ptr<Data::Frame> frame_R)
    : mpFrame_L(frame_L), mpFrame_R(frame_R){std::cout<<"thi"<<std::endl;};


    void Tracking::detectFeatures(const int& nFeatures)
    {
        cv::Ptr<cv::Feature2D> detector = cv::ORB::create(nFeatures);

        std::vector<cv::KeyPoint> framePoints_L = mpFrame_L->getFramePoint2d();
        std::vector<cv::KeyPoint> framePoints_R = mpFrame_R->getFramePoint2d();

        cv::Mat descriptors_L = mpFrame_L->getDescriptors();
        cv::Mat descriptors_R = mpFrame_R->getDescriptors();

        // std::cout << descriptors_L.type() << std::endl;

        detector->detectAndCompute(mpFrame_L->getFrame(), cv::Mat(), framePoints_L, descriptors_L);
        detector->detectAndCompute(mpFrame_R->getFrame(), cv::Mat(), framePoints_R, descriptors_R);

        mpFrame_L->setFramePoint2d(framePoints_L);
        mpFrame_R->setFramePoint2d(framePoints_R);

        mpFrame_L->setDescriptors(descriptors_L);
        mpFrame_R->setDescriptors(descriptors_R);

    }


    void Tracking::matchFeatures()
    {
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
#if 0
        std::vector<cv::DMatch> matches;
        matcher->match(mpFrame_L->getDescriptors(), mpFrame_R->getDescriptors(), matches);

        std::sort(matches.begin(), matches.end());
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + threshold);
#elif 1

        std::vector<std::vector<cv::DMatch>> matches;
        matcher->knnMatch(mpFrame_L->getDescriptors(), mpFrame_R->getDescriptors(), matches, 2);

        std::vector<cv::DMatch> goodMatches;
        for (auto m : matches)
        {
            if (m[0].distance / m[1].distance < 0.6)
            {
                goodMatches.push_back(m[0]);
            }
        }
#endif
        auto framePoints_L = mpFrame_L->getFramePoint2d();
        auto framePoints_R = mpFrame_R->getFramePoint2d();

        std::vector<cv::Point2d> goodMatches_L = mpFrame_L->getGoodMatches();
        std::vector<cv::Point2d> goodMatches_R = mpFrame_R->getGoodMatches();

        for (size_t i = 0; i < goodMatches.size(); i++)
        {
            goodMatches_L.push_back(framePoints_L[goodMatches[i].queryIdx].pt);
            goodMatches_R.push_back(framePoints_R[goodMatches[i].trainIdx].pt);
        }

        mpFrame_L->setGoodMatches(goodMatches_L);
        mpFrame_R->setGoodMatches(goodMatches_R);

        cv::Mat dst;
        cv::drawMatches(mpFrame_L->getFrame(), mpFrame_L->getFramePoint2d(), mpFrame_R->getFrame(), mpFrame_R->getFramePoint2d(), goodMatches, dst);
        cv::imshow("dst", dst);
        cv::waitKey(0);
    }


    void Tracking::computeTriangulation(const cv::Point2d& ptr_L, const cv::Point2d& ptr_R)
    {
        cv::Mat_<double> K(3,3);
        K << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1;

        std::vector<cv::Point3d> framePoint3d_L = mpFrame_L->getFramePoint3d();
        cv::Mat translationMatrix = mpFrame_L->getTranslationMatrix();

        cv::Point3d p;

        cv::Mat& t = translationMatrix;
        double baseline = sqrt(std::pow(t.ptr<double>(0)[0],2) + std::pow(t.ptr<double>(0)[1],2) + std::pow(t.ptr<double>(0)[2],2));

        std::cout << "baseline : " << baseline << std::endl;

        p.z = std::abs(baseline * mpFrame_L->getScale() / (ptr_R.x - ptr_L.x)); // depth of point
        // p.z = std::abs(10000 / (ptr_R.x - ptr_L.x)); // depth of point

        std::cout << "p.z : " << p.z << std::endl;


        p.x = p.z / K.ptr<double>(0)[0] * (ptr_L.x - K.ptr<double>(0)[2]); // x of point
        p.y = p.z / K.ptr<double>(1)[1] * (ptr_L.y - K.ptr<double>(1)[2]); // y of point

        std::cout << "( " << p.x << ", " << p.y << ", " << p.z << " )" << std::endl;

        framePoint3d_L.push_back(p);
    }

}



template<typename T>
void computeEssentialMatrix(T frame_L, T frame_R)
{
    cv::Mat_<double> K(3,3);
    K << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1;

    auto goodMatches_L = frame_L->getGoodMatches();
    auto goodMatches_R = frame_R->getGoodMatches();

    frame_L->setEssentialMatrix(cv::findEssentialMat(goodMatches_L, goodMatches_R, K));
    cv::Mat essentialMatrix = frame_L->getEssentialMatrix();

    cv::Mat rotationMatrix = frame_L->getRotationMatrix(); // double
    cv::Mat translationMatrix = frame_L->getTranslationMatrix(); // double

    cv::recoverPose(essentialMatrix, goodMatches_L, goodMatches_R, K, rotationMatrix, translationMatrix);

    frame_L->setRotationMatrix(rotationMatrix);
    frame_L->setTranslationMatrix(translationMatrix); // tvec은 distance가 1인 유닛 벡터이다. 스케일이 정해지지 않음.
}

***************************************************************************************************/
