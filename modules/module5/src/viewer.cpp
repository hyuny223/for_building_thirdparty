#include "Eigen/Dense"
#include "Eigen/Core"
#include "pangolin/pangolin.h"
#include <unistd.h>
#include <iostream>// 궤도값 file의 path 설정
#include <viewer.h>
// std::string trajectory_file = "/home/chl/CLionProjects/Day3_1/thirdparty/Pangolin/Pangolin/examples/trajectory.txt";// DrawTrajectory 선언
// void DrawTrajectory(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>);int main(int argc, char **argv)
// {    // 위치값을 담을 vector를 선언.
//     std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;    //trajectory_file에서 값을 받아 들이 변수. readlines()
//     std::ifstream fin(trajectory_file);    
    
//     if (!fin)
//     {
//         std::cout << "cannot find trajectory file at " << trajectory_file << std::endl;
//         return 1;
//     }    
    
    
//     while (!fin.eof())
//     {
//         // txt 파일에서 read한 값을 받아 들일 변수 설정
//         double time, tx, ty, tz, qx, qy, qz, qw;
//         // 경로에서 받아온 file의 값을 한 줄씩 받아들여 변수에 저장
//         fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;        // typedef transfoem<Scalar=double, Dim=3, Mode=Isometry> 를 의미한다.
//         // Isometry는 거리 보존(distance-preserving) transformation이라고 하는데, 잘 모르겠다.
//         // 쿼터니안 값을 Isometry
//         Eigen::Isometry3d Twr(Eigen::Quaterniond(qw, qx, qy, qz));        std::cout << "This is a Rotation Mat : " << std::endl;
//         std::cout << Twr.rotation() << std::endl;
//         std::cout << "\n";        // Eigen::Vector3d로 표현되는 translate matrix를 Twr에 적용
//         std::cout << "This is a Translation Mat : " << std::endl;
//         Twr.pretranslate(Eigen::Vector3d(tx, ty, tz));
//         std::cout << Twr.rows() << std::endl;
//         std::cout << "\n\n";        // translated한 Isometry3d를 pose vector에 push_back
//         poses.push_back(Twr);
//     }    // 좌표의 size를 출력
//     std::cout << "read total " << poses.size() << " pose entries" << std::endl;    // 궤도를 그린다
// //    DrawTrajectory(poses);
//     return 0;
// }/*******************************************************************************************/
void DrawTrajectory(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses)
{
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);    
    pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );    
    pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f).SetHandler(new pangolin::Handler3D(s_cam));    
    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);
        for (size_t i = 0; i < poses.size(); i++) {
            // 画每个位姿的三个坐标轴
            Eigen::Vector3d Ow = poses[i].translation();
            Eigen::Vector3d Xw = poses[i] * (0.1 * Eigen::Vector3d(1, 0, 0));
            Eigen::Vector3d Yw = poses[i] * (0.1 * Eigen::Vector3d(0, 1, 0));
            Eigen::Vector3d Zw = poses[i] * (0.1 * Eigen::Vector3d(0, 0, 1));
            glBegin(GL_LINES);
            glColor3f(1.0, 0.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Xw[0], Xw[1], Xw[2]);
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Yw[0], Yw[1], Yw[2]);
            glColor3f(0.0, 0.0, 1.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Zw[0], Zw[1], Zw[2]);
            glEnd();
        }        for (size_t i = 0; i < poses.size(); i++) {
            glColor3f(0.0, 0.0, 0.0);
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}