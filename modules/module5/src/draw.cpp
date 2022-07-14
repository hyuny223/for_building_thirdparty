#include <cmath>
#include "draw.h"

#include "opencv2/opencv.hpp"

#include "pangolin/pangolin.h"
#include "Eigen/Core"
#include "Eigen/Geometry"



void drawCurrentFrame(size_t i, std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses)
{
    const float w = 0.06;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();
    pangolin::OpenGlMatrix Twc_(poses[i].matrix());
    glMultMatrixd(Twc_.m);

    glColor3f(299/255.f, 113/255.f, 8/255.f);
    glLineWidth(2);
    glBegin(GL_LINES);

    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);
    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);
    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);
    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);

    glEnd();
    glPopMatrix();
}

void drawKeytFrame(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> keyFrame)
{
    const float w = 0.06;
    const float h = w*0.75;
    const float z = w*0.6;

    for(auto Twc:keyFrame)
    {
        glPushMatrix();
        pangolin::OpenGlMatrix Twc_(Twc.matrix());
        glMultMatrixd(Twc_.m);

        glColor3f(299/255.f, 113/255.f, 8/255.f);
        glLineWidth(2);
        glBegin(GL_LINES);

        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);
        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);
        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);
        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);

        glEnd();
        glPopMatrix();
    }
}
void drawCoordinate(float scale)
{
    glLineWidth(3);
    glBegin(GL_LINES);

    glColor3f(1.0, 0.0, 0.0);
    glVertex3d(0,0,0);
    glVertex3d(scale, 0,0);

    glColor3f(1.0, 0.0, 0.0);
    glVertex3d(0,0,0);
    glVertex3d(0, scale,0);

    glColor3f(1.0, 0.0, 0.0);
    glVertex3d(0,0,0);
    glVertex3d(0, 0, scale);

    glEnd();
}

// void drawLine(size_t i,std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses, bool drawLine)
// {
//     glLineWidth(2);
//     if(drawLine)
//     {
//         for(size_t = )
//     }

// }
