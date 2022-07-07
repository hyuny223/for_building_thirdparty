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

            cv::Mat mRcw, mTcw; // 절대포즈. 상대포즈는 프레임 클래스에 rotation, translation으로 존재

            int miKeyFrameNum;

        public:
            KeyFrame() = default;
            KeyFrame(std::shared_ptr<Data::Frame> frame);

            std::shared_ptr<Data::Frame> getKeyFrame();

            void setWorldPosition(cv::Point3d worldPosition);
            cv::Point3d getWorldPosition();

            void setReprojPoints(std::vector<cv::KeyPoint> reprojPoints);
            std::vector<cv::KeyPoint> getReprojPoints();


            void setW2CRotationMat(cv::Mat rotation);
            cv::Mat getW2CRotationMat();

            void setW2CTranslationMat(cv::Mat translation);
            cv::Mat getW2CTranslationMat();


            int getKeyFrameNum();

            std::shared_ptr<Data::KeyFrame> getPrevKeyFrame();
            std::shared_ptr<Data::KeyFrame> getWorldKeyFrame();
    };
}
