#include <iostream>
#include <filesystem>
#include <string>
#include <vector>
#include <algorithm>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"

#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

#include "loadImage.h"
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


double k[] = {718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1};
cv::Mat K(3,3,CV_64FC1,k);

double d[] = {0,0,0,0};
cv::Mat distCoeffs(4, 1, CV_64F, d);

int nFeatures = 1000;
bool firstKeyFrameflag = false;



int main(int argc, char* argv[])
{

    if(argc != 2)
    {
        std::cout << "Put your image path.\nOnly one argument available" << std::endl;
        return -1;
    }

    spdlog::info("proSLAM start!!");
    std::string p = argv[1];
    

    std::pair<std::vector<std::string>, std::vector<std::string>> image = argparse(p);
    std::vector<std::string> leftFiles = image.first, rightFiles = image.second;

	if (leftFiles.size() == 0 || rightFiles.size() == 0){
        spdlog::error("이미지가 존재하지 않습니다. 종료합니다.");
        return -1;
    }
    else{
        spdlog::info("image path : {}", p);
        spdlog::info("image 개수 : {} \n\n\t[ start : press key (debug : d)]", leftFiles.size());
        if(getchar() != 'd') spdlog::set_level(spdlog::level::off);
    }

    std::shared_ptr<Data::KeyFrame> currKeyFrame;
    std::shared_ptr<Data::LocalMap> localMap = std::make_shared<Data::LocalMap>();

    pangolin::CreateWindowAndBind("Trajectory viewer",1024,768);
    cv::Mat img = cv::Mat(500,500,CV_8UC3);

    std::string mode = "AKAZE";

	for (int cnt = 0; cnt < leftFiles.size(); ++cnt)
	{

		cv::Mat rawImage_L = cv::imread(leftFiles[cnt]);
        cv::Mat rawImage_R = cv::imread(rightFiles[cnt]);
        spdlog::info("=========== frame number : {} ===========\n", cnt);

        cv::Mat image_L, image_R;
        cv::undistort(rawImage_L, image_L, K, distCoeffs);
        cv::undistort(rawImage_R, image_R, K, distCoeffs);


        std::shared_ptr<Data::Frame> frame_1 = std::make_shared<Data::Frame>(image_L);
        std::shared_ptr<Data::Frame> frame_2 = std::make_shared<Data::Frame>(image_R);


        spdlog::info("|   detectFeatures start   |");
        Frontend::detectFeatures(frame_1, frame_2, mode); // 이미지 코너 검출
        spdlog::info("---detectFeatures complete---\n");

        spdlog::info("|   matchFeatures start   |");
        Frontend::matchFeatures(frame_1, frame_2, mode); // 두 이미지 간 매칭점
        spdlog::info("---matchFeatures complete---\n");

        spdlog::info("|   computeTriangulation start   |");
        Frontend::computeTriangulation(frame_1,frame_2);
        spdlog::info("---computeTriangulation complete---");

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
            spdlog::info("firstKeyFrame done !! [back]");
            continue;
        }

        std::shared_ptr<Data::KeyFrame> prevKeyFrame = keyFrameVec.getKeyFrameVec().back();

        std::shared_ptr<Similarity> sim = std::make_shared<Similarity>(prevKeyFrame, frame_1);
        sim->findSimFeatures(mode);

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

        spdlog::info("|   optimization start   |");
        optimization(prevKeyFrame, currKeyFrame);
        spdlog::info("---optimization complete---\n");

        Frontend::computeWorldPosition(prevKeyFrame, currKeyFrame);
        // cv::Point pt(currKeyFrame->getWorldPosition().x*10, -currKeyFrame->getWorldPosition().z*10);
        // cv::circle(img, pt, 10, (255,255,255),3);

        // cv::imshow("img",img);
        // cv::waitKey(0);

        if(localMap->getLocalMap().size() == 20) // 로컬맵에 키프레임이 20개라면 global map에 저장하고 local map reset
        {
            globalMap.setGlobalMap(localMap);
            localMap->resetLocalMap();
        }
        else // 키프레임이 20장 미만이라면 local map에 keyframe 계속 저장
        {
            localMap->setLocalMap(currKeyFrame);
        }


        // Drawing using pangolin
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
            glClear(GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            drawKeytFrame(poses);
            drawCoordinate(0.5);
            // bool flag = true;
            // drawLine(i, poses, flag);
        }
        pangolin::FinishFrame();

    }
    // std::cout << keyFrameVec.getKeyFrameVec().size() << std::endl;
    // spdlog::info(">>>>>>>>>>>>>>> proSLAM success!! <<<<<<<<<<<<<<");
    return 0;
}
