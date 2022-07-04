#pragma once

#include "frame.h"

namespace Data
{
    class KeyFrame: public Frame
    {
        protected:
            std::shared_ptr<Data::Frame> mpFrame;
            cv::Mat mmT_cw, mmT_wc;

        public:
            KeyFrame() = default;
            KeyFrame(std::shared_ptr<Data::Frame>);
            // ~KeyFrame();

            void setKeyFrame(std::shared_ptr<Data::Frame> frame);
            std::shared_ptr<Data::Frame> getKeyFrame();

            void setPose_cw(const cv::Mat& T_cw);
            cv::Mat getPose_cw();

            void setPose_wc(const cv::Mat& T_wc);
            cv::Mat geePose_wc();
    };
}
