
#include <iostream>
#include "frame.h"
#include "keyframe.h"
#include "quaternion.h"
#include "opencv2/opencv.hpp"
#include <cmath>

namespace Frontend
{
    //cv::Mat q is 1x4 matrix
    void R2Quaternion(std::shared_ptr<Data::KeyFrame> prevFrame)
    {
        cv::Mat R = prevFrame->getRotationMat();
        cv::Mat q(1,4,CV_64F);

        double roll = atan2(R.at<double>(2,1),R.at<double>(2,2));
        double pitch = atan2(-R.at<double>(2,0),sqrt(R.at<double>(2,1)*R.at<double>(2,1)+R.at<double>(2,2)*R.at<double>(2,2)));
        double yaw = atan2(R.at<double>(1,0),R.at<double>(0,0));

        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        q.at<double>(0,0) = cr * cp * cy + sr * sp * sy;
        q.at<double>(0,1) = sr * cp * cy - cr * sp * sy;
        q.at<double>(0,2) = cr * sp * cy + sr * cp * sy;
        q.at<double>(0,3) = cr * cp * sy - sr * sp * cy;

        prevFrame->setQuaternion(q);

    }

    void Quaternion2R(cv::Mat& q, cv::Mat& R)
    {

        double q0 = q.at<double>(0,0);
        double q1 = q.at<double>(0,1);
        double q2 = q.at<double>(0,2);
        double q3 = q.at<double>(0,3);

        R.at<double>(0,0) = 2*(q0*q0+q1*q1)-1.0;
        R.at<double>(0,1) = 2*(q1*q2-q0*q3);
        R.at<double>(0,2) = 2*(q1*q3+q0*q2);
        R.at<double>(1,0) = 2*(q1*q2+q0*q3);
        R.at<double>(1,1) = 2*(q0*q0+q2*q2)-1.0;
        R.at<double>(1,2) = 2*(q2*q3-q0*q1);
        R.at<double>(2,0) = 2*(q1*q3-q0*q2);
        R.at<double>(2,1) = 2*(q2*q3+q0*q1);
        R.at<double>(2,2) = 2*(q0*q0+q3*q3)-1.0;
    }
}
