#pragma once

#include "opencv2/opencv.hpp"
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Dense"


class Pose2dErrorTerm
{
    private:
        Eigen::Vector2d mvOrigin;
    public:
        Pose2dErrorTerm(cv::KeyPoint origin)
        : mvOrigin(origin.pt.x, origin.pt.y){}

        template <typename T>
        bool operator()(const T* const estimated, T* residual)
        {
            Eigen::Vector2d mvEstimated{estimated.pt.x, estimated.pt.y};

            auto delta_x = (mvOrigin[0]-mvEstimated[0]);
            auto delta_y = (mvOrigin[1]-mvEstimated[1]);

            residual[0] = delta_x*delta_x + delta_y*delta_y;

            return true;
        }
};

template <typename T>
void optimization(T origin, T estimated)
{
    using ceres::AutoDiffCostFunction;
    using ceres::CostFunction;
    using ceres::Problem;
    using ceres::Solver;
    using ceres::Solve;

    std::cout << "before : (x,y) = (" << estimated.pt.x << estimated.pt.y << ")" << std::endl;

    Problem problem;

    CostFunction* cost_function =
        new AutoDiffCostFunction<Pose2dErrorTerm, 1, 2>(new Pose2dErrorTerm(origin));

    problem.AddResidualBlock(cost_function, NULL, estimated);

    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";

    std::cout << "after : (x,y) = (" << estimated.pt.x << estimated.pt.y << ")" << std::endl;
}
