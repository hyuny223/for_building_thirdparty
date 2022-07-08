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

    // cv::Mat image_L = cv::imread("/root/dataset/sequences/00/image_0/000086.png");
    cv::Mat image_L = cv::imread("/root/dataset/sequences/00/image_0/000092.png");
    cv::Mat image_R = cv::imread("/root/dataset/sequences/00/image_0/000096.png");

    std::shared_ptr<Data::Frame> frame_L = std::make_shared<Data::Frame>(image_L);
    std::shared_ptr<Data::Frame> frame_R = std::make_shared<Data::Frame>(image_R);
    std::shared_ptr<Data::KeyFrame> currKeyFrame;



    int nFeatures = 500;
    Frontend::detectFeatures(frame_L, frame_R, nFeatures); // 이미지 코너 검출
    Frontend::matchFeatures(frame_L, frame_R); // 두 이미지 간 매칭점
    Frontend::computeEssentialMatrix(frame_L, frame_R); // 두 이미지 간 Essential Matrix를 구하는 과정.
                                                        // 그러나 Mono에서는 KeyFrame에서 구하는 것이기에 의미가 없다.
    Frontend::computeTriangulation(frame_L, frame_R); // Correspondence 간의 Triangulation을 계산


    if(currKeyFrame->mvKeyFrameVec.size() == 0) // 첫번째라면
    {
        currKeyFrame = std::make_shared<Data::KeyFrame>(frame_L); //키프레임으로 지정하고
        // continue; //처음부터 시작하기
    }

    std::shared_ptr<Similarity> sim = std::make_shared<Similarity>(currKeyFrame, frame_L); // 유사성 비교하는 클래스. 키프레임을 뽑기 위한 과정. 왼쪽은 prev, 오른쪽은 curr이 되어야 한다.
    sim->findSimFeatures(); // 두 이미지의 Correpondence 찾기


    if (sim->computeSimilarity(nFeatures)) // 충분히 다르다고 생각하면
    {
        currKeyFrame = std::make_shared<Data::KeyFrame>(frame_L); //curr를 키프레임으로 선정.
    }
    else // 아니라면 다음 이미지로 넘어가기
    {
        // continue;
    }



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

    return 0;
}
