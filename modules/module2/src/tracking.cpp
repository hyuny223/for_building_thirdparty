#include <iostream>
#include "tracking.h"
#include "datatype.h"
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>


namespace Frontend
{

    void Tracking::detectFeatures(const cv::Mat& gray_L, const cv::Mat& gray_R, const int& nfeatures)
    {
        cv::Ptr<cv::Feature2D> detector = cv::ORB::create(nfeatures);

        std::vector<cv::KeyPoint> framePoints_L = framePoint->getFramePoint2d_L();
        std::vector<cv::KeyPoint> framePoints_R = framePoint->getFramePoint2d_R();

        cv::Mat descriptors_L = framePoint->getDescriptors_L();
        cv::Mat descriptors_R = framePoint->getDescriptors_R();

        detector->detectAndCompute(gray_L, cv::Mat(), framePoints_L, descriptors_L);
        detector->detectAndCompute(gray_R, cv::Mat(), framePoints_R, descriptors_R);

        framePoint->setFramePoint2d_L(framePoints_L);
        framePoint->setFramePoint2d_R(framePoints_R);

        framePoint->setDescriptors_L(descriptors_L);
        framePoint->setDescriptors_R(descriptors_R);
    }


    void Tracking::matchFeatures(const cv::Mat& gray_L, const cv::Mat& gray_R)
    {
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);

        std::vector<cv::DMatch> matches = framePoint->getMatches();

        matcher->match(framePoint->getDescriptors_L(), framePoint->getDescriptors_R(), matches);
        framePoint->setMatches(matches);

        std::sort(matches.begin(), matches.end());

        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin()+10);

        auto framePoints_L = framePoint->getFramePoint2d_L();
        auto framePoints_R = framePoint->getFramePoint2d_R();

        std::vector<cv::Point2f> goodMatches_L = framePoint->getGoodMatches_L();
        std::vector<cv::Point2f> goodMatches_R = framePoint->getGoodMatches_R();

        for (size_t i = 0; i < goodMatches.size(); i++)
        {
            goodMatches_L.push_back(framePoints_L[goodMatches[i].queryIdx].pt);
            goodMatches_R.push_back(framePoints_R[goodMatches[i].trainIdx].pt);

            // std::cout << "left_x : " <<goodMatches_L[i].x << ", left_y : " << goodMatches_L[i].y << std::endl;
            // std::cout << "right_ x : " <<goodMatches_R[i].x << ", right_y : " << goodMatches_R[i].y << std::endl;
            // std::cout << '\n';
        }

        framePoint->setGoodMatches_L(goodMatches_L);
        framePoint->setGoodMatches_R(goodMatches_R);

        cv::Mat dst;
        cv::drawMatches(gray_L, framePoints_L, gray_R, framePoints_R, goodMatches, dst);

        cv::imshow("dst",dst);
        cv::waitKey(0);
    }


    void Tracking::computeFundamentalMatrix()
    {
        cv::Mat_<double> K(3,3);
        K << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1;

        framePoint->setEssentialMatrix(cv::findEssentialMat(framePoint->getGoodMatches_L(), framePoint->getGoodMatches_R(), K));
        cv::Mat essentialMatrix = framePoint->getEssentialMatrix();


        cv::Mat rotationMatrix = framePoint->getRotationMatrix();
        cv::Mat translationMatrix = framePoint->getTranslationMatrix();

        cv::recoverPose(essentialMatrix, framePoint->getGoodMatches_L(), framePoint->getGoodMatches_R(), K, rotationMatrix, translationMatrix);
        framePoint->setRotationMatrix(rotationMatrix);
        framePoint->setTranslationMatrix(translationMatrix);

    }

    void Tracking::computeTriangulation(const cv::Point2f& ptr_L, const cv::Point2f& ptr_R)
    {
        cv::Mat_<double> K(3,3);
        K << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1;

        std::vector<cv::Point3f> framePoint3d_L = framePoint->getFramePoint3d_L();
        auto translationMatrix = framePoint->getTranslationMatrix();

        cv::Point3f p;
        p.z = *translationMatrix.ptr<double>(0) / (ptr_R.x - ptr_L.x); // depth of point
        p.x = p.z / K.ptr<float>(0)[0] * (ptr_L.x - K.ptr<float>(0)[2]); // x of point
        p.y = p.z / K.ptr<float>(1)[1] * (ptr_L.y - K.ptr<float>(1)[2]); // y of point

        std::cout << "( " << p.x << ", " << p.y << ", " << p.z << " )" << std::endl;

        framePoint3d_L.push_back(p);
    }
}
