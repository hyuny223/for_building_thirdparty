#include <iostream>
#include <vector>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"

#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

#include "framepoint.h"
#include "frame.h"
#include "keyframe.h"
#include "localmap.h"
#include "globalmap.h"
#include "tracking.h"
#include "similarity.h"
#include "projection.h"
#include "optimization.h"
#include "quaternion.h"
#include "keyframeVec.h"
#include "pangolin/pangolin.h"
#include "ceres/rotation.h"
#include "draw.h"

Data::KeyFrameVec& keyFrameVec = Data::KeyFrameVec::GetInstance();
Data::GlobalMap& globalMap = Data::GlobalMap::GetInstance();


int main()
{
    spdlog::info("proSLAM start!!");

    std::string path0("/root/dataset/sequences/00/image_0/*.png"); // ch1
    std::string path1("/root/dataset/sequences/00/image_1/*.png"); // ch1

	std::vector<std::string> str0;
	std::vector<std::string> str1;

	int index = 0;
	cv::glob(path0, str0, false);
	cv::glob(path1, str1, false);


	if (str0.size() == 0 || str1.size() == 0){
        spdlog::error("이미지가 존재하지 않습니다. 종료합니다.");
        return -1;
    }
    else{
        spdlog::info("image path : {}", path0);
        spdlog::info("image 개수 : {} \n\n\t[ start : press key (debug : d)]", str0.size());
        if(getchar() != 'd') spdlog::set_level(spdlog::level::off);
    }

    int nFeatures = 1000;

    bool firstKeyFrameflag = false;

    std::shared_ptr<Data::KeyFrame> currKeyFrame;
    std::shared_ptr<Data::LocalMap> localMap = std::make_shared<Data::LocalMap>();

    pangolin::CreateWindowAndBind("Trajectory viewer",1024,768);

	for (int cnt = 0; cnt < str0.size(); ++cnt)
	{

		cv::Mat image_1 = cv::imread(str0[cnt]);
        cv::Mat image_2 = cv::imread(str1[cnt]);
        spdlog::info("=========== frame number : {} ===========\n", cnt);


        std::shared_ptr<Data::Frame> frame_1 = std::make_shared<Data::Frame>(image_1);
        std::shared_ptr<Data::Frame> frame_2 = std::make_shared<Data::Frame>(image_2);


        spdlog::info("|   detectFeatures start   |");
        Frontend::detectFeatures(frame_1, frame_2, nFeatures); // 이미지 코너 검출
        spdlog::info("---detectFeatures complete---\n");

        spdlog::info("|   matchFeatures start   |");
        Frontend::matchFeatures(frame_1, frame_2); // 두 이미지 간 매칭점
        spdlog::info("---matchFeatures complete---\n");

        if(firstKeyFrameflag == false && frame_1->getGoodMatches().size() < 15) // 키프레임이 선정되지 않았고, 굿매치가 15개 이하라면 패스
        {
            spdlog::warn("firstKeyFrameflag fail !! [back]");
            continue;
        }

        if(firstKeyFrameflag == false && frame_1->getGoodMatches().size() >= 15) // 첫번째 키프레임 선정
        {

            currKeyFrame = std::make_shared<Data::KeyFrame>(frame_1);
            keyFrameVec.setKeyFrameVec(currKeyFrame);
            firstKeyFrameflag = true;
            localMap->setLocalMap(currKeyFrame);
            spdlog::warn("firstKeyFrame done !! [back]");
            continue;
        }

        std::shared_ptr<Data::KeyFrame> prevKeyFrame = keyFrameVec.getKeyFrameVec().back();

        std::shared_ptr<Similarity> sim = std::make_shared<Similarity>(prevKeyFrame, frame_1);
        sim->findSimFeatures();
        if (sim->computeSimilarity(nFeatures)) // 충분히 다르다고 생각하면
        {
            currKeyFrame = std::make_shared<Data::KeyFrame>(frame_1); //curr를 키프레임으로 선정.
            keyFrameVec.setKeyFrameVec(currKeyFrame);
        }
        else // 아니라면 다음 이미지로 넘어가기
        {
            spdlog::warn("NO Similarity !! [back]");
            continue;
        }

        spdlog::info("|   computeEssentialMatrix start   |");
        Frontend::computeEssentialMatrix(prevKeyFrame, currKeyFrame); // 두 이미지 간 Essential Matrix를 구하는 과정.
        spdlog::info("---computeEssentialMatrix complete---\n");    // 그러나 Mono에서는 KeyFrame에서 구하는 것이기에 의미가 없다.


        spdlog::info("|   computeTriangulation start   |");
        Frontend::computeTriangulation(prevKeyFrame, currKeyFrame); // Correspondence 간의 Triangulation을 계산
        spdlog::info("---computeTriangulation complete---\n");

        spdlog::info("|   doProjection start   |");
        doProjection(prevKeyFrame, currKeyFrame);
        spdlog::info("---doProjection complete---\n");

        spdlog::info("|   optimization start   |");
        optimization(prevKeyFrame, currKeyFrame);
        spdlog::info("---optimization complete---\n");

        Frontend::computeWorldPosition(prevKeyFrame, currKeyFrame);

        if(localMap->getLocalMap().size() == 20) // 로컬맵에 키프레임이 20개라면 global map에 저장하고 local map reset
        {
            globalMap.setGlobalMap(localMap);
            localMap->resetLocalMap();
        }
        else // 키프레임이 20장 미만이라면 local map에 keyframe 계속 저장
        {
            localMap->setLocalMap(currKeyFrame);
        }

        Frontend::R2Quaternion(currKeyFrame);

        cv::Mat t = currKeyFrame->getTranslationMat();
        cv::Mat q = currKeyFrame->getQuaternion();

        std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
        std::vector<double> timeRecords;
        double tx = t.ptr<double>(0)[0];
        double ty = t.ptr<double>(0)[1];
        double tz = t.ptr<double>(0)[2];
        double qx = q.ptr<double>(0)[0];
        double qy = q.ptr<double>(0)[1];
        double qz = q.ptr<double>(0)[2];
        double qw = q.ptr<double>(0)[3];

        Eigen::Isometry3d Twr(Eigen::Quaterniond(qw,qx,qy,qz).normalized());
        Twr.pretranslate(Eigen::Vector3d(tx, ty, tz));
        poses.push_back(Twr);

        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 800, 200, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, pangolin::AxisZ)
        );
        pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


        for(size_t i=0; (pangolin::ShouldQuit()==false)&&i<poses.size();++i)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            drawKeytFrame(poses);
            drawCoordinate(0.5);
            // bool flag = true;
            // drawLine(i, poses, flag);
        }
        pangolin::FinishFrame();

    }

    std::cout << keyFrameVec.getKeyFrameVec().size() << std::endl;
    spdlog::info(">>>>>>>>>>>>>>> proSLAM success!! <<<<<<<<<<<<<<");

    return 0;
}




// int main ()
// {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(5,1));
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
//     // Fill in the CloudIn data
//     for (auto& point : *cloud_in)
//     {
//     point.x = 1024 * rand() / (RAND_MAX + 1.0f);
//     point.y = 1024 * rand() / (RAND_MAX + 1.0f);
//     point.z = 1024 * rand() / (RAND_MAX + 1.0f);
//     }

//     std::cout << "Saved " << cloud_in->size () << " data points to input:" << std::endl;
//     std::cout << "\n\n";

//     for (auto& point : *cloud_in)
//     {
//         std::cout << point << std::endl;
//     }
//     std::cout << "\n\n";

//     *cloud_out = *cloud_in;

//     for (auto& point : *cloud_out)
//     {
//         point.x += 0.7f;
//     }

//     for (auto& point : *cloud_out)
//     {
//         std::cout << point << std::endl;
//     }
//     std::cout << "\n\n";

//     pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

//     icp.setInputSource(cloud_in);
//     icp.setInputTarget(cloud_out);

//     pcl::PointCloud<pcl::PointXYZ> Final;
//     icp.align(Final);

//     std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//     icp.getFitnessScore() << std::endl;

//     std::cout << icp.getFinalTransformation() << std::endl;
//     return 0;
// }
