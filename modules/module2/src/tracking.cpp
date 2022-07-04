#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include "tracking.h"
#include "framepoint.h"
#include "frame.h"

int threshold = 20;


namespace Frontend
{
    Tracking::Tracking(const std::shared_ptr<Data::Frame> frame_L,
                        const std::shared_ptr<Data::Frame> frame_R)
    : mpFrame_L(frame_L), mpFrame_R(frame_R){};

    void Tracking::detectFeatures(const int& nFeatures)
    {
        cv::Ptr<cv::Feature2D> detector = cv::ORB::create(nFeatures);

        std::vector<cv::KeyPoint> framePoints_L = mpFrame_L->getFramePoint2d();
        std::vector<cv::KeyPoint> framePoints_R = mpFrame_R->getFramePoint2d();

        cv::Mat descriptors_L = mpFrame_L->getDescriptors();
        cv::Mat descriptors_R = mpFrame_R->getDescriptors();

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

        std::vector<cv::DMatch> matches;

        matcher->match(mpFrame_L->getDescriptors(), mpFrame_R->getDescriptors(), matches);

        std::sort(matches.begin(), matches.end());

        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + threshold);

        auto framePoints_L = mpFrame_L->getFramePoint2d();
        auto framePoints_R = mpFrame_R->getFramePoint2d();

        std::vector<cv::Point2f> goodMatches_L = mpFrame_L->getGoodMatches();
        std::vector<cv::Point2f> goodMatches_R = mpFrame_R->getGoodMatches();

        for (size_t i = 0; i < goodMatches.size(); i++)
        {
            goodMatches_L.push_back(framePoints_L[goodMatches[i].queryIdx].pt);
            goodMatches_R.push_back(framePoints_R[goodMatches[i].trainIdx].pt);

        //     // std::cout << "left_x : " <<goodMatches_L[i].x << ", left_y : " << goodMatches_L[i].y << std::endl;
        //     // std::cout << "right_ x : " <<goodMatches_R[i].x << ", right_y : " << goodMatches_R[i].y << std::endl;
        //     // std::cout << '\n';
        }

        mpFrame_L->setGoodMatches(goodMatches_L);
        mpFrame_R->setGoodMatches(goodMatches_R);

        // cv::Mat dst;
        // cv::drawMatches(mpFrame_L->getFrame(), framePoints_L, mpFrame_R->getFrame(), framePoints_R, goodMatches, dst);
        // cv::imshow("dst", dst);
        // cv::waitKey(0);
    }


    void Tracking::computeEssentialMatrix()
    {
        cv::Mat_<double> K(3,3);
        K << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1;

        mpFrame_L->setEssentialMatrix(cv::findEssentialMat(mpFrame_L->getGoodMatches(), mpFrame_R->getGoodMatches(), K));
        cv::Mat essentialMatrix = mpFrame_L->getEssentialMatrix();

        cv::Mat rotationMatrix = mpFrame_L->getRotationMatrix();
        cv::Mat translationMatrix = mpFrame_L->getTranslationMatrix();

        cv::recoverPose(essentialMatrix, mpFrame_L->getGoodMatches(), mpFrame_R->getGoodMatches(), K, rotationMatrix, translationMatrix);
        mpFrame_L->setRotationMatrix(rotationMatrix);
        mpFrame_L->setTranslationMatrix(translationMatrix);
    }


    void Tracking::computeTriangulation(const cv::Point2f& ptr_L, const cv::Point2f& ptr_R)
    {
        cv::Mat_<double> K(3,3);
        K << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1;

        std::vector<cv::Point3f> framePoint3d_L = mpFrame_L->getFramePoint3d();
        auto translationMatrix = mpFrame_L->getTranslationMatrix();

        std::cout << translationMatrix.size() << std::endl;
        std::cout << translationMatrix.at<float>(0) << std::endl;
        std::cout << translationMatrix.at<float>(1) << std::endl;
        std::cout << translationMatrix.at<float>(2) << std::endl;
        std::cout << '\n';

        // cv::Point3f p;
        // p.z = translationMatrix.at<double>(0) / (ptr_R.x - ptr_L.x); // depth of point
        // p.x = p.z / K.ptr<float>(0)[0] * (ptr_L.x - K.ptr<float>(0)[2]); // x of point
        // p.y = p.z / K.ptr<float>(1)[1] * (ptr_L.y - K.ptr<float>(1)[2]); // y of point

        // std::cout << "( " << p.x << ", " << p.y << ", " << p.z << " )" << std::endl;

        // framePoint3d_L.push_back(p);
    }
}
