#pragma once

#include "frame.h"

namespace Data
{
    class KeyFrame: public Frame
    {
        protected:
            std::shared_ptr<Data::Frame> mpFrame;
            std::vector<cv::KeyPoint> mvReprojPoints = {};

            cv::Point3d mWorldPosition = {0,0,0};

            cv::Mat mWorld2CamTransform = cv::Mat::eye(4, 4, CV_32F);

            cv::Mat mQuaternion;

        public:
            KeyFrame() = default;
            KeyFrame(std::shared_ptr<Data::Frame> frame);

            std::shared_ptr<Data::Frame> getKeyFrame();

            void setReprojPoints(std::vector<cv::KeyPoint> reprojPoints);
            std::vector<cv::KeyPoint> getReprojPoints();

            int getKeyFrameNum();

            std::shared_ptr<Data::KeyFrame> getPrevKeyFrame();
            std::shared_ptr<Data::KeyFrame> getWorldKeyFrame();

            void setQuaternion(cv::Mat q);
            cv::Mat getQuaternion();

            void setWorldPosition(cv::Point3d position);
            cv::Point3d getWorldPosition();

            void setWorld2CamTransformMat(cv::Mat transform);
            cv::Mat getWorld2CamTransformMat();

    };
}
