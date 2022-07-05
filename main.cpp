#include <iostream>

#include "opencv2/opencv.hpp"
#include "framepoint.h"
#include "frame.h"
#include "keyframe.h"
#include "tracking.h"
#include "similarity.h"
#include "projection.h"


int main()
{
    cv::Mat image_L = cv::imread("/root/dataset/sequences/00/image_0/000092.png");
    cv::Mat image_R = cv::imread("/root/dataset/sequences/00/image_0/000096.png");

    std::shared_ptr<Data::Frame> frame_L = std::make_shared<Data::Frame>(image_L);
    std::shared_ptr<Data::Frame> frame_R = std::make_shared<Data::Frame>(image_R);

    auto tracking = std::make_shared<Frontend::Tracking<std::shared_ptr<Data::Frame>>>(frame_L, frame_R);

    int nFeatures = 500;
    tracking->detectFeatures(nFeatures);
    tracking->matchFeatures();
    tracking->computeEssentialMatrix();

    for(int i = 0; i < frame_L->getGoodMatches().size(); ++i)
    {
        tracking->computeTriangulation(frame_L->getGoodMatches()[i], frame_R->getGoodMatches()[i]);
    }

    std::shared_ptr<Similarity> sim = std::make_shared<Similarity>(frame_L, frame_R);
    sim->findSimFeatures();
    std::cout << frame_L->getFramePoint3d().size() << std::endl;


    Data::KeyFrame keyFrame;
    if (sim->computeSimilarity(nFeatures))
    {
        // keyFrame = std::make_shared<Data::KeyFrame>(frame_L);
        keyFrame = static_cast<Data::KeyFrame>(frame_L);
    }

    std::cout << keyFrame.getFramePoint3d().size() << std::endl;
    std::shared_ptr<Projection> proj = std::make_shared<Projection>();
    // proj->doProjection(keyFrame);

    return 0;
}

/*

//Require: input arrays from left and right image
KeypointWD K_L[] = .., K_R[] = ..;

//Ensure: output array of stereo keypoint pairs
pair<KeypointWD, KeypointWD> K_LR[];

//sort all input vectors in the order of the expression
sort(K_L, ((K_L[i].r < K_L[j].r) || (K_L[i].r == K_L[j].r && K_L[i].c < K_L[j].c)));
sort(K_R, ((K_R[i].r < K_R[j].r) || (K_L[i].r == K_L[j].r && K_R[i].c < K_R[j].c)));

//configuration
const float maximum_matching_distance = ..;
int idx_R = 0;

//loop over all left keypoints
for (int idx_L = 0; idx_L < K_L.size(); idx_L++)
{
    //stop condition
    if (idx_R == K_R.size()) {break;}

    //the right keypoints are on an lower row - skip left
    while (K_L[idx_L].r < K_R[idx_R].r)
    {
        idx_L++;
        if (idx_L == K_L.size()) {break;}
    }

    //the right keypoints are on an upper row - skip right
    while (K_L[idx_L].r > K_R[idx_R].r)
    {
        idx_R++;
        if (idx_R == K_R.size()) {break;}
    }

    //search bookkeeping
    int idx_RS = idx_R;
    float dist_best = maximum_matching_distance;
    int idx_best_R = 0;

    //scan epipolar line for current keypoint at idx_L
    while (K_L[idx_L].r == K_R[idx_RS].r)
    {
        //zero disparity stop condition
        if (K_R[idx_RS].c >= K_L[idx_L].c) {break;}

        //compute descriptor distance
        const float dist = hnorm(K_L[idx_L].d, K_R[idx_RS].d)
        if(dist < dist_best)
        {
            dist_best = dist;
            idx_best_R = idx_RS;
        }
        idx_RS++;
    }

    //check if something was found
    if (dist_best < maximum_matching_distance)
    {
        K_LR += pair(K_L[idx_L], K_R[idx_best_R]);
        idx_R = idx_best_R+1;
    }
}




//Require: current frame (carrying framepoints)
Frame* frame = ..;

//Require|Ensure: transform estimate camera to/from world
Transform T_c2w = .., T_w2c = ..;

//ds configuration
const bool ignore_outliers = ..;
const float kernel_maximum_error = ..;
const float close_depth = ..;
const float maximum_depth = ..;
const int number_of_iterations = ..;

for (int i = 0; i < number_of_iterations; i++)
{
    //initialize least squares components
    Matrix6 H = 0, Vector6 b = 0, Matrix4 omega = I;

    //loop over all framepoints
    for (Framepoint* fp: frame->points())
    {
        if (!fp->prev()) {continue;}

        CameraCoordinates p_c = T_w2c*fp->prev()->p_w;

        //preferably use landmark position estimate
        if (fp->landmark())
        {
            p_c = T_w2c*fp->landmark()->p_w;
            //increase weight for landmarks
            omega = ..;
        }

        //project point into left and right image
        const Keypoint k_L = frame->cam_L->project(p_c);
        const Keypoint k_R = frame->cam_R->project(p_c);

        //ds compute current reprojection error
        const Vector4 error(k_L.u-fp->k_L.u, k_L.v-fp->k_L.v,
        k_R.u-fp->k_R.u, k_R.v-fp->k_R.v);

        //compute squared error
        const float error_squared = error.transpose()*error;

        //robust kernel
        if (error_squared > kernel_maximum_error)
        {
            if (ignore_outliers) {continue;}
            omega *= kernel_maximum_error/error_squared;
        }
        else
        {
            fp->inlier = true;
        }

        //compute stereouv jacobian see Eq. (11)
        Matrix4_6 J = getJacobian(T_w2c, p_c, frame);


        //adjust omega: if the point is considered close
        if(p_c.z() < close_depth)
        {
            omega *= (close_depth-p_c.z())/close_depth;
        }
        else
        {
            omega *= (maximum_depth-p_c.z())/maximum_depth;

            //disable contribution to translational error
            J.block<3,3>(0,0) = 0;
        }
        //update H and b
        H += J.transpose()*omega*J;
        b += J.transpose()*omega*error;
    }

    //compute (Identity-damped) solution
    const Vector6 dx = solve((H+I)\(-b));
    T_w2c = v2t(dx)*T_w2c;
    T_c2w = T_w2c.inverse();
}

*/
