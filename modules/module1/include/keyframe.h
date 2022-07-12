#pragma once

#include "frame.h"

namespace Data
{
    class KeyFrame;

    class KeyFrameVec
    {
        protected:

        public:
            KeyFrameVec() = default;
            static std::vector<std::shared_ptr<Data::KeyFrame>> mvKeyFrameVec;
    };



    class KeyFrame: public Frame, public KeyFrameVec
    {
        protected:
            std::shared_ptr<Data::Frame> mpFrame;
            std::vector<cv::KeyPoint> mvReprojPoints;

            cv::Point3d mWorldPosition = {0,0,0};

            cv::Mat mw2c = cv::Mat::eye(4, 4, CV_32F);

            int miKeyFrameNum;

            cv::Mat mQuaternion;

        public:
            KeyFrame() = default;
            KeyFrame(std::shared_ptr<Data::Frame> frame);

            std::shared_ptr<Data::Frame> getKeyFrame();

            // void setWorldPosition(cv::Point3d worldPosition);
            // cv::Point3d getWorldPosition();

            void setReprojPoints(std::vector<cv::KeyPoint> reprojPoints);
            std::vector<cv::KeyPoint> getReprojPoints();


            // void setW2CRotationMat(cv::Mat rotation);
            // cv::Mat getW2CRotationMat();

            // void setW2CTranslationMat(cv::Mat translation);
            // cv::Mat getW2CTranslationMat();

            void setw2c(cv::Mat w2c);
            cv::Mat getw2c();

            int getKeyFrameNum();

            std::shared_ptr<Data::KeyFrame> getPrevKeyFrame();
            std::shared_ptr<Data::KeyFrame> getWorldKeyFrame();

            void setQuaternion(cv::Mat q);
            cv::Mat getQuaternion();

    };
}
