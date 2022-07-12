#include <iostream>

#include "opencv2/opencv.hpp"
#include "framepoint.h"
#include "frame.h"
#include "keyframe.h"
#include "tracking.h"
#include "similarity.h"
#include "projection.h"
#include "optimization.h"

#include <vector>
#include <spdlog/spdlog.h>

int main()
{

    spdlog::info("proSLAM start!!");

	std::string path("/root/dataset/00/image_0/*.png");
	std::vector<std::string> str;

	int index = 0;
	cv::glob(path, str, false);


	if (str.size() == 0){
        spdlog::error("이미지가 존재하지 않습니다. 종료합니다.");
        return -1;
    }
    else{
        spdlog::info("image path : {}", path);
        spdlog::info("image 개수 : {} \n\n\t[ start : press key (debug : d)]", str.size());
        if(getchar() != 'd') spdlog::set_level(spdlog::level::off);
    }

	for (int cnt = 0; cnt < str.size() - 1; cnt+=2) 
	{
        // if(cnt < 4320) continue;
		cv::Mat image_1 = cv::imread(str[cnt]);
        cv::Mat image_2 = cv::imread(str[cnt+1]);

        if(!image_1.isContinuous()) image_1 = image_1.clone();
        if(!image_2.isContinuous()) image_2 = image_2.clone();

        std::shared_ptr<Data::Frame> frame_1 = std::make_shared<Data::Frame>(image_1);
        std::shared_ptr<Data::Frame> frame_2 = std::make_shared<Data::Frame>(image_2);
        std::shared_ptr<Data::KeyFrame> currKeyFrame;

        int nFeatures = 500;

        spdlog::info("|   detectFeatures start   |");
        Frontend::detectFeatures(frame_1, frame_2, nFeatures); // 이미지 코너 검출
        spdlog::info("---detectFeatures complete---\n");

        spdlog::info("|   matchFeatures start   |");
        Frontend::matchFeatures(frame_1, frame_2); // 두 이미지 간 매칭점
        spdlog::info("---matchFeatures complete---\n");

        spdlog::info("|   computeEssentialMatrix start   |");
        Frontend::computeEssentialMatrix(frame_1, frame_2); // 두 이미지 간 Essential Matrix를 구하는 과정.
        spdlog::info("---computeEssentialMatrix complete---\n");    // 그러나 Mono에서는 KeyFrame에서 구하는 것이기에 의미가 없다.

        spdlog::info("|   computeTriangulation start   |");                                                                 
        Frontend::computeTriangulation(frame_1, frame_2); // Correspondence 간의 Triangulation을 계산
        spdlog::info("---computeTriangulation complete---\n");             

        auto prevKeyFrame = std::make_shared<Data::KeyFrame>(frame_1);
        auto curKeyFrame = std::make_shared<Data::KeyFrame>(frame_2);

        spdlog::info("|   doProjection start   |");             
        doProjection(prevKeyFrame, curKeyFrame);
        spdlog::info("---doProjection complete---\n");             

        spdlog::info("|   optimization start   |");
        optimization(prevKeyFrame, curKeyFrame);
        spdlog::info("---optimization complete---\n");

        spdlog::info("|   re-Projection start   |");
        doProjection(prevKeyFrame, curKeyFrame);
        spdlog::info("---re-Projection complete---\n");

        spdlog::info("=========== frame number : {} ===========\n", cnt);


        // if(currKeyFrame->mvKeyFrameVec.size() == 0) // 첫번째라면
        // {
        //     currKeyFrame = std::make_shared<Data::KeyFrame>(frame_1); //키프레임으로 지정하고
        //     // continue; //처음부터 시작하기
        // }

        // std::shared_ptr<Similarity> sim = std::make_shared<Similarity>(currKeyFrame, frame_1); // 유사성 비교하는 클래스. 키프레임을 뽑기 위한 과정. 왼쪽은 prev, 오른쪽은 curr이 되어야 한다.
        // sim->findSimFeatures(); // 두 이미지의 Correpondence 찾기


        // if (sim->computeSimilarity(nFeatures)) // 충분히 다르다고 생각하면
        // {
        //     currKeyFrame = std::make_shared<Data::KeyFrame>(frame_1); //curr를 키프레임으로 선정.
        // }
        // else // 아니라면 다음 이미지로 넘어가기
        // {
        //     // continue;
        // }



        // 키프레임이 처음이 아니라면 월드좌표 구하고 projection 진행
        // 월드좌표 = 이전 키프레임의 월드좌표 + 이전 키프레임과 현재 키프레임의 상대좌표
        // const auto& prevKeyFrame = currKeyFrame->getPrevKeyFrame();
        // Frontend::computeEssentialMatrix(prevKeyFrame, currKeyFrame); // 이전 키프레임과 현재 키프레임간 상대포즈를 구한다.

        /*
            1. 월드좌표를 구하기 위해서는 월드좌표를 (0,0,0)으로 설정한다
            2. Rt를 구한다.
            3. 이전 좌표에 t를 더한다.
            4. 1로 돌아간다.
        */
        // Frontend::computeWorldPosition(prevKeyFrame, currKeyFrame);


        // K * Rt * World Coordinate (projection)


        // std::shared_ptr<Projection> proj = std::make_shared<Projection>();
        // proj->doProjection(keyFrame);

        // num_plus++;
    }

    spdlog::info(">>>>>>>>>>>>>>> proSLAM success!! <<<<<<<<<<<<<<");

    return 0;
}
