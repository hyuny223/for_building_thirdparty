#pragma once

#include "opencv2/opencv.hpp"
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Dense"
#include "projection.h"
#include "quaternion.h"
#include "spdlog/spdlog.h"
#include "tracking.h"
#include "ceres/rotation.h"


class Pose2dErrorTerm
{
    private:
        std::shared_ptr<Data::KeyFrame> mPrev, mCurr;
    public:
        Pose2dErrorTerm(std::shared_ptr<Data::KeyFrame> prevKeyFrame, std::shared_ptr<Data::KeyFrame> currKeyFrame)
        : mPrev(prevKeyFrame), mCurr(currKeyFrame){}
        template <typename T>
        bool operator()(const T* const r, const T* const t, T* residual) const
        {
            double k[] = {718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1};
            auto origin3d = mPrev->getFramePoint3d();
            auto origin2d = mCurr->getGoodMatches();
            T* R = new T[9];

            ceres::AngleAxisToRotationMatrix(r,R);

            // T sumOfError;
            T sumOfx;
            T sumOfy;

            for(int i=0; i < origin3d.size(); ++i)
            {
                auto origin_p = origin3d[i];
                T origin_x = T(origin_p.x);
                T origin_y = T(origin_p.y);
                T origin_z = T(origin_p.z);
                T camera_x = R[0]*origin_x + R[1]*origin_y + R[2]*origin_z + t[0];
                T camera_y = R[3]*origin_x + R[4]*origin_y + R[5]*origin_z + t[1];
                T camera_z = R[6]*origin_x + R[7]*origin_y + R[8]*origin_z + t[2];
                T projected_x = (camera_x*k[0] + camera_z*k[2]) / camera_z;
                T projected_y = (camera_y*k[4] + camera_y*k[5]) / camera_z;
                T diff_x = T(origin2d[i].x) - projected_x;
                T diff_y = T(origin2d[i].y) - projected_y;
                // T error = diff_x*diff_x + diff_y*diff_y;
                // sumOfError += error;
                sumOfx += diff_x*diff_x;
                sumOfy += diff_y*diff_y;
            }
            // residual[0] = sumOfError;
            residual[0] = sumOfx;
            residual[1] = sumOfy;
            return true;
        }
};

template <typename T>
void optimization(T prevKeyFrame, T currKeyFrame)
{
    using ceres::AutoDiffCostFunction;
    using ceres::CostFunction;
    using ceres::Problem;
    using ceres::Solver;
    using ceres::Solve;

    Problem problem;

    cv::Mat rotation = currKeyFrame->getRotationMat();
    cv::Mat translationMat = currKeyFrame->getTranslationMat();

    cv::Mat rodrigues;
    cv::Rodrigues(rotation, rodrigues);

    double* R = new double[3];
    double* t = new double[3];

    /*
    ceres의 문제점 : double* 타입만 매개변수로 넘길 수 있다. 포인터에 값들을 넘기는 과정이 필요...
    */

    R[0] = rodrigues.ptr<double>(0)[0];
    R[1] = rodrigues.ptr<double>(1)[0];
    R[2] = rodrigues.ptr<double>(2)[0];

    t[0] = translationMat.ptr<double>(0)[0];
    t[1] = translationMat.ptr<double>(1)[0];
    t[2] = translationMat.ptr<double>(2)[0];

    CostFunction* cost_function =
        new AutoDiffCostFunction<Pose2dErrorTerm, 2, 3, 3>(new Pose2dErrorTerm(prevKeyFrame, currKeyFrame));

    // problem.AddResidualBlock(cost_function, NULL , R, t);
    // problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1) , R, t);
    // problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(1.0) , R, t);
    // problem.AddResidualBlock(cost_function, new ceres::TukeyLoss(1.0) , R, t);
    // problem.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5) , R, t);
    // problem.AddResidualBlock(cost_function, new ceres::TolerantLoss(0.5) , R, t);
    // problem.AddResidualBlock(cost_function, new ceres::ArctanLoss(0.5) , R, t);
    problem.AddResidualBlock(cost_function, new ceres::ComposedLoss(new ceres::SoftLOneLoss(0.5), ceres::DO_NOT_TAKE_OWNERSHIP, new ceres::ArctanLoss(0.5), ceres::DO_NOT_TAKE_OWNERSHIP), R, t);
    // problem.AddResidualBlock(cost_function, new ceres::ComposedLoss(new ceres::CauchyLoss(2.0), ceres::DO_NOT_TAKE_OWNERSHIP, new ceres::SoftLOneLoss(2.0), ceres::DO_NOT_TAKE_OWNERSHIP), R, t);


    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 50;
    options.update_state_every_iteration = true;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.FullReport() << "\n";
    spdlog::info("- Solve complete");


    rodrigues.ptr<double>(0)[0] = R[0];
    rodrigues.ptr<double>(1)[0] = R[1];
    rodrigues.ptr<double>(2)[0] = R[2];

    translationMat.ptr<double>(0)[0] = t[0];
    translationMat.ptr<double>(1)[0] = t[1];
    translationMat.ptr<double>(2)[0] = t[2];

    cv::Mat rotationMat;
    cv::Rodrigues(rodrigues, rotationMat);

    currKeyFrame->setRotationMat(rotationMat);
    spdlog::info("- setRotationMat complete");
    currKeyFrame->setTranslationMat(translationMat);
    spdlog::info("- translation, quaternion");

    cv::Mat transformMat;
    Frontend::computeTransformMat(rotationMat, translationMat, transformMat);
    currKeyFrame->setTransformMat(transformMat);


    std::cout << summary.BriefReport() << "\n";
}



#if 0

class Pose2dErrorTerm
{
    private:
        std::shared_ptr<Data::KeyFrame> mPrev, mCurr;
    public:
        Pose2dErrorTerm(std::shared_ptr<Data::KeyFrame> prevKeyFrame, std::shared_ptr<Data::KeyFrame> currKeyFrame)
        : mPrev(prevKeyFrame), mCurr(currKeyFrame){}
        template <typename T>
        bool operator()(const T* const Q, const T* const t, T* residual) const
        {
            double k[] = {718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1};
            auto origin3d = mPrev->getFramePoint3d();
            auto origin2d = mCurr->getGoodMatches();
            T* R = new T[9];
            R[0] = 2.0 * (Q[0] * Q[0] + Q[1] * Q[1])-1.0;
            R[1]= 2.0 * (Q[1] * Q[2] - Q[0] * Q[3]);
            R[2] = 2.0 * (Q[1] * Q[3] + Q[0] * Q[2]);
            R[3] = 2.0 * (Q[1] * Q[2] + Q[0] * Q[3]);
            R[4] = 2.0 * (Q[0] * Q[0] + Q[2] * Q[2])-1.0;
            R[5] = 2.0 * (Q[2] * Q[3] - Q[0] * Q[1]);
            R[6] = 2.0 * (Q[1] * Q[3] - Q[0] * Q[2]);
            R[7] = 2.0 * (Q[2] * Q[3] + Q[0] * Q[1]);
            R[8] = 2.0 * (Q[0] * Q[0] + Q[3] * Q[3])-1.0;
            // T sumOfError;
            T sumOfx;
            T sumOfy;

            for(int i=0; i < origin3d.size(); ++i)
            {
                auto origin_p = origin3d[i];
                T origin_x = T(origin_p.x);
                T origin_y = T(origin_p.y);
                T origin_z = T(origin_p.z);
                T camera_x = R[0]*origin_x + R[1]*origin_y + R[2]*origin_z + t[0];
                T camera_y = R[3]*origin_x + R[4]*origin_y + R[5]*origin_z + t[1];
                T camera_z = R[6]*origin_x + R[7]*origin_y + R[8]*origin_z + t[2];
                T projected_x = (camera_x*k[0] + camera_z*k[2]) / camera_z;
                T projected_y = (camera_y*k[4] + camera_y*k[5]) / camera_z;
                T diff_x = T(origin2d[i].x) - projected_x;
                T diff_y = T(origin2d[i].y) - projected_y;
                // T error = diff_x*diff_x + diff_y*diff_y;
                // sumOfError += error;
                sumOfx += diff_x*diff_x;
                sumOfy += diff_y*diff_y;
            }
            // residual[0] = sumOfError;
            residual[0] = sumOfx;
            residual[1] = sumOfy;
            return true;
        }
};


template <typename T>
void optimization(T prevKeyFrame, T currKeyFrame)
{
    using ceres::AutoDiffCostFunction;
    using ceres::CostFunction;
    using ceres::Problem;
    using ceres::Solver;
    using ceres::Solve;

    Problem problem;

    cv::Mat rotation = currKeyFrame->getRotationMat();

    cv::Mat quaternion = currKeyFrame->getQuaternion();
    cv::Mat translationMat = currKeyFrame->getTranslationMat();

    double* Q = new double[4];
    double* t = new double[3];

    /*
    ceres의 문제점 : double* 타입만 매개변수로 넘길 수 있다. 포인터에 값들을 넘기는 과정이 필요...
    */
    Q[0] = quaternion.ptr<double>(0)[0];
    Q[1] = quaternion.ptr<double>(0)[1];
    Q[2] = quaternion.ptr<double>(0)[2];
    Q[3] = quaternion.ptr<double>(0)[3];

    t[0] = translationMat.ptr<double>(0)[0];
    t[1] = translationMat.ptr<double>(1)[0];
    t[2] = translationMat.ptr<double>(2)[0];

    CostFunction* cost_function =
        new AutoDiffCostFunction<Pose2dErrorTerm, 2, 3, 3>(new Pose2dErrorTerm(prevKeyFrame, currKeyFrame));

    // problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1) , Q, t);
    // problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.0001) , Q, t);
    // problem.AddResidualBlock(cost_function, new ceres::TukeyLoss(500) , Q, t);
    // problem.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.5) , Q, t);


    problem.AddResidualBlock(cost_function, NULL , rodrigues, t);


    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 50;
    options.update_state_every_iteration = true;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.FullReport() << "\n";
    spdlog::info("- Solve complete");


    /*
    최적화된 값을 다시 R,t에 넣어주는 과정
    */

    quaternion.ptr<double>(0)[0] = Q[0];
    quaternion.ptr<double>(0)[1] = Q[1];
    quaternion.ptr<double>(0)[2] = Q[2];
    quaternion.ptr<double>(0)[3] = Q[3];


    Frontend::Quaternion2R(quaternion,rotationMat);

    translationMat.ptr<double>(0)[0] = t[0];
    translationMat.ptr<double>(1)[0] = t[1];
    translationMat.ptr<double>(2)[0] = t[2];

    currKeyFrame->setRotationMat(rotationMat);
    spdlog::info("- setRotationMat complete");
    currKeyFrame->setTranslationMat(translationMat);
    currKeyFrame->setQuaternion(quaternion);
    spdlog::info("- translation, quaternion");

    cv::Mat transformMat;
    Frontend::computeTransformMat(rotationMat, translationMat, transformMat);
    currKeyFrame->setTransformMat(transformMat);


    std::cout << summary.BriefReport() << "\n";
    spdlog::info("- summary.BriefReport() :\n{}", summary.BriefReport());
}

#endif

// enum LinearSolverType {
//   // These solvers are for general rectangular systems formed from the
//   // normal equations A'A x = A'b. They are direct solvers and do not
//   // assume any special problem structure.

//   // Solve the normal equations using a dense Cholesky solver; based
//   // on Eigen.
//   DENSE_NORMAL_CHOLESKY,

//   // Solve the normal equations using a dense QR solver; based on
//   // Eigen.
//   DENSE_QR,

//   // Solve the normal equations using a sparse cholesky solver;
//   SPARSE_NORMAL_CHOLESKY,

//   // Specialized solvers, specific to problems with a generalized
//   // bi-partitite structure.

//   // Solves the reduced linear system using a dense Cholesky solver;
//   // based on Eigen.
//   DENSE_SCHUR,

//   // Solves the reduced linear system using a sparse Cholesky solver;
//   // based on CHOLMOD.
//   SPARSE_SCHUR,

//   // Solves the reduced linear system using Conjugate Gradients, based
//   // on a new Ceres implementation.  Suitable for large scale
//   // problems.
//   ITERATIVE_SCHUR,

//   // Conjugate gradients on the normal equations.
//   CGNR
// };



// class Pose2dErrorTerm
// {
//     private:
//         std::shared_ptr<Data::KeyFrame> mPrev, mCurr;

//     public:
//         Pose2dErrorTerm(std::shared_ptr<Data::KeyFrame> prevKeyFrame, std::shared_ptr<Data::KeyFrame> currKeyFrame)
//         : mPrev(prevKeyFrame), mCurr(currKeyFrame){}

//         template <typename T>
//         bool operator()(const T* const R, const T* const t, T* residual) const
//         {

//             double k[] = {718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1};

//             auto origin3d = mPrev->getFramePoint3d();
//             auto origin2d = mCurr->getGoodMatches();

//             T sumOfError;

//             for(int i=0; i < origin3d.size(); ++i)
//             {
//                 auto origin_p = origin3d[i];

//                 auto origin_x = origin_p.x;
//                 auto origin_y = origin_p.y;
//                 auto origin_z = origin_p.z;

//                 T camera_x = R[0]*origin_x + R[1]*origin_y + R[2]*origin_z + t[0];
//                 T camera_y = R[3]*origin_x + R[4]*origin_y + R[5]*origin_z + t[1];
//                 T camera_z = R[6]*origin_x + R[7]*origin_y + R[8]*origin_z + t[2];

//                 T projected_x = (camera_x*k[0] + camera_z*k[2]) / camera_z;
//                 T projected_y = (camera_y*k[4] + camera_y*k[5]) / camera_z;

//                 T diff_x = T(origin2d[i].x) - projected_x;
//                 T diff_y = T(origin2d[i].y) - projected_y;

//                 T error = diff_x*diff_x + diff_y*diff_y;

//                 sumOfError += error;
//             }

//             residual[0] = sumOfError;

//             return true;
//         }
// };

// template <typename T>
// void optimization(T prevKeyFrame, T currKeyFrame)
// {
//     using ceres::AutoDiffCostFunction;
//     using ceres::CostFunction;
//     using ceres::Problem;
//     using ceres::Solver;
//     using ceres::Solve;

//     Problem problem;

//     cv::Mat rotationMat = prevKeyFrame->getRotationMat();
//     cv::Mat translationMat = prevKeyFrame->getTranslationMat();

//     double* R = new double[9];
//     double* t = new double[3];


//     /*
//     ceres의 문제점 : double* 타입만 매개변수로 넘길 수 있다. 포인터에 값들을 넘기는 과정이 필요...
//     */
//     int n = 0;
//     for(int y = 0; y < rotationMat.rows; ++y)
//     {
//         for(int x = 0; x < rotationMat.cols; ++x)
//         {
//             R[n] = rotationMat.ptr<double>(y)[x];
//             n++;
//         }
//     }

//     n = 0;
//     for(int y = 0; y < translationMat.rows; ++y)
//     {
//         for(int x = 0; x < translationMat.cols; ++x)
//         {
//             t[n] = translationMat.ptr<double>(y)[x];
//             n++;
//         }
//     }

//     CostFunction* cost_function =
//         new AutoDiffCostFunction<Pose2dErrorTerm, 1, 9, 3>(new Pose2dErrorTerm(prevKeyFrame, currKeyFrame));

//     problem.AddResidualBlock(cost_function, nullptr, R, t);

//     Solver::Options options;
//     options.minimizer_progress_to_stdout = true;
//     Solver::Summary summary;
//     Solve(options, &problem, &summary);


//     /*
//     최적화된 값을 다시 R,t에 넣어주는 과정
//     */
//     n = 0;
//     for(int y=0; y<rotationMat.rows; ++y)
//     {
//         for(int x=0; x<rotationMat.cols; ++x)
//         {
//             rotationMat.at<double>(y,x) = R[n];
//             ++n;
//         }
//     }

//     n = 0;
//     for(int y=0; y<translationMat.rows; ++y)
//     {
//         for(int x=0; x<translationMat.cols; ++x)
//         {
//             translationMat.at<double>(y,x) = t[n];
//             ++n;
//         }
//     }

//     prevKeyFrame->setRotationMat(rotationMat);
//     prevKeyFrame->setTranslationMat(translationMat);


//     std::cout << summary.BriefReport() << "\n";

// }
