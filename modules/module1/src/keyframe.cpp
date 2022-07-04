#include "keyframe.h"

namespace Data
{

    void KeyFrame::setKeyFrame(std::shared_ptr<Data::Frame> frame)
    {
        mpFrame = frame;
    }

    std::shared_ptr<Data::Frame> KeyFrame::getKeyFrame()
    {
        return mpFrame;
    }


    void KeyFrame::setPose_cw(const cv::Mat& T_cw)
    {
        mmT_cw = T_cw;
    }

    cv::Mat KeyFrame::getPose_cw()
    {
        return mmT_cw;
    }


    void KeyFrame::setPose_wc(const cv::Mat& T_wc)
    {
        mmT_wc = T_wc;
    }

    cv::Mat KeyFrame::geePose_wc()
    {
        return mmT_wc;
    }

}
