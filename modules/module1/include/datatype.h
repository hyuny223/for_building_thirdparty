#pragma once
// cv::Point3_<_Tp>::Point3_(_Tp _x, _Tp _y, _Tp _z)
#include <iostream>
#include "opencv2/opencv.hpp"

namespace Data
{

    class FramePoint
    {
        protected:
            std::vector<cv::Point3f> mvFramePoints3d_L;
            std::vector<cv::Point3f> mvFramePoints3d_R;

            std::vector<cv::KeyPoint> mvFramePoints2d_L;
            std::vector<cv::KeyPoint> mvFramePoints2d_R;

            cv::Mat mDescriptors_L;
            cv::Mat mDescriptors_R;

            std::vector<cv::DMatch> mMatches;

            std::vector<cv::Point2f> mvGoodMatches_L;
            std::vector<cv::Point2f> mvGoodMatches_R;

            cv::Mat mmFundamentalMatrix;
            cv::Mat mmEssentialMatrix;
            cv::Mat mmRoationMatrix;
            cv::Mat mmTranslationMatrix;

        public:
            FramePoint() = default;

            void setFramePoint3d_L(const std::vector<cv::Point3f>& framePoints);
            void setFramePoint3d_R(const std::vector<cv::Point3f>& framePoints);

            std::vector<cv::Point3f> getFramePoint3d_L();
            std::vector<cv::Point3f> getFramePoint3d_R();


            void setFramePoint2d_L(const std::vector<cv::KeyPoint>& framePoints);
            void setFramePoint2d_R(const std::vector<cv::KeyPoint>& framePoints);

            std::vector<cv::KeyPoint> getFramePoint2d_L();
            std::vector<cv::KeyPoint> getFramePoint2d_R();


            void setDescriptors_L(const cv::Mat& descriptors);
            void setDescriptors_R(const cv::Mat& descriptors);

            cv::Mat getDescriptors_L();
            cv::Mat getDescriptors_R();


            void setMatches(const std::vector<cv::DMatch>& match);
            std::vector<cv::DMatch> getMatches();


            void setGoodMatches_L(const std::vector<cv::Point2f>& goodMatch);
            void setGoodMatches_R(const std::vector<cv::Point2f>& goodMatch);

            std::vector<cv::Point2f> getGoodMatches_L();
            std::vector<cv::Point2f> getGoodMatches_R();

            void setFundamentalMatrix(const cv::Mat& F);
            void setEssentialMatrix(const cv::Mat& E);
            void setRotationMatrix(const cv::Mat& R);
            void setTranslationMatrix(const cv::Mat& t);

            cv::Mat getFundamentalMatrix();
            cv::Mat getEssentialMatrix();
            cv::Mat getRotationMatrix();
            cv::Mat getTranslationMatrix();

    };

}
