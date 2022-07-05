#pragma once

#include "framepoint.h"
#include "frame.h"
#include "keyframe.h"

namespace Frontend
{
    template <typename T>
    class Tracking
    {

        protected:
            T mpFrame_L;
            T mpFrame_R;

        public:
            Tracking() = default;
            Tracking(T frame_L, T frame_R)
            : mpFrame_L(frame_L), mpFrame_R(frame_R){};
            // ~Tracking();

            void detectFeatures(const int& nFeatures)
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
            };


            void matchFeatures()
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

            };


            void computeEssentialMatrix()
            {
                cv::Mat_<double> K(3,3);
                K << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1;

                auto goodMatches_L = mpFrame_L->getGoodMatches();
                auto goodMatches_R = mpFrame_R->getGoodMatches();

                mpFrame_L->setEssentialMatrix(cv::findEssentialMat(goodMatches_L, goodMatches_R, K));
                cv::Mat essentialMatrix = mpFrame_L->getEssentialMatrix();

                cv::Mat rotationMatrix = mpFrame_L->getRotationMatrix(); // double
                cv::Mat translationMatrix = mpFrame_L->getTranslationMatrix(); // double

                cv::recoverPose(essentialMatrix, goodMatches_L, goodMatches_R, K, rotationMatrix, translationMatrix);

                mpFrame_L->setRotationMatrix(rotationMatrix);
                mpFrame_L->setTranslationMatrix(translationMatrix); // tvec은 distance가 1인 유닛 벡터이다. 스케일이 정해지지 않음.
            }


            void computeTriangulation(const cv::Point2d& ptr_L, const cv::Point2d& ptr_R)
            {
                cv::Mat_<double> K(3,3);
                K << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1;

                std::vector<cv::Point3d> framePoint3d_L = mpFrame_L->getFramePoint3d();
                cv::Mat translationMatrix = mpFrame_L->getTranslationMatrix();

                cv::Point3d p;

                cv::Mat& t = translationMatrix;
                double baseline = sqrt(std::pow(t.ptr<double>(0)[0],2) + std::pow(t.ptr<double>(0)[1],2) + std::pow(t.ptr<double>(0)[2],2));

                p.z = std::abs(baseline * mpFrame_L->getScale() / (ptr_R.x - ptr_L.x)); // depth of point

                p.x = p.z / K.ptr<double>(0)[0] * (ptr_L.x - K.ptr<double>(0)[2]); // x of point
                p.y = p.z / K.ptr<double>(1)[1] * (ptr_L.y - K.ptr<double>(1)[2]); // y of point

                framePoint3d_L.push_back(p);
                mpFrame_L->setFramePoint3d(framePoint3d_L);
            };
    };
}

