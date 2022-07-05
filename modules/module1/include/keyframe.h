#pragma once

#include "frame.h"

namespace Data
{
    class KeyFrame: public Frame
    {
        protected:
            std::shared_ptr<Data::Frame> mpFrame;
            std::vector<cv::KeyPoint> mvReprojPoints;

            cv::Mat mmT_cw, mmT_wc; //절대포즈
            cv::Mat mmT_lr; //상대포즈

        public:
            KeyFrame() = default;
            KeyFrame(std::shared_ptr<Data::Frame> frame);
            // ~KeyFrame();

            // void setKeyFrame(std::shared_ptr<Data::Frame> frame);
            std::shared_ptr<Data::Frame> getKeyFrame();

            void setReprojPoints(std::vector<cv::KeyPoint> reprojPoints);
            std::vector<cv::KeyPoint> getReprojPoints();

            void setPose_cw(const cv::Mat& T_cw);
            cv::Mat getPose_cw();

            void setPose_wc(const cv::Mat& T_wc);
            cv::Mat getPose_wc();

            void setPose_lr(const cv::Mat& T_lr);
            cv::Mat getPose_lr();
    };
}
