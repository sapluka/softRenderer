#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>
#include <vector>
#include "objLoader.h"
#include "parameter.h"



class Transform {
    private:
    Eigen::Matrix4f viewProjection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f viewport = Eigen::Matrix4f::Identity();
    int bufferWidth = SCREEN_WIDTH;
    int bufferHeight = SCREEN_HEIGHT;

    public:
    Transform() = default;
    // 每个视角准备一次矩阵，顶点变换和阴影查询直接复用。
    Transform(const Eigen::Vector3f& eye, const Eigen::Vector3f& target, const Eigen::Vector3f& up,
              int buffer_width = SCREEN_WIDTH, int buffer_height = SCREEN_HEIGHT,
              float nearPlane = z_near, float farPlane = z_far);
    Eigen::Vector3f apply(const Eigen::Vector3f& point, float& clipW) const;
    int getBufferWidth() const { return bufferWidth; }
    int getBufferHeight() const { return bufferHeight; }

    //transform函数封装了MVP，NDC和Viewport矩阵转换
    static Eigen::Vector3f transform(const Eigen::Vector3f& point, const Eigen::Vector3f& eye,const Eigen::Vector3f& target,
                                     const Eigen::Vector3f& up = Eigen::Vector3f::UnitY());//封装了各种矩阵，掌管整个流程
    static Eigen::Vector3f transform(const Eigen::Vector3f& point, const Eigen::Vector3f& eye, const Eigen::Vector3f& target,
                                     const Eigen::Vector3f& up, float& clipW,
                                     float nearPlane = z_near, float farPlane = z_far,
                                     int buffer_width = SCREEN_WIDTH, int buffer_height = SCREEN_HEIGHT);
    
    static Eigen::Matrix4f getModelMatrix(std::vector<float>&scales,std::vector<float>&translates,Eigen::Vector3f axis,float theta);//先暂时不管输入参数
    static Eigen::Matrix4f getViewMatrix(const Eigen::Vector3f& eye,const Eigen::Vector3f& target,
                                        const Eigen::Vector3f& up = Eigen::Vector3f::UnitY());
    static Eigen::Matrix4f getProjectionMatrix(float nearPlane = z_near, float farPlane = z_far);
    static Eigen::Vector4f getNDC(const Eigen::Vector4f& clipSpacePoint);
    static Eigen::Matrix4f getViewportMatrix(int buffer_width, int buffer_height);

    //下面这个函数是用来计算重心坐标的
    static bool computeBarycentric2D(const Eigen::Vector3f& point, const Eigen::Vector3f& first, const Eigen::Vector3f& second,
                                    const Eigen::Vector3f& third, float& alpha, float& beta, float& gamma);
    static void computeBarycentric3D(const Eigen::Vector3f& p, const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c, float& alpha, float& beta, float& gamma);

    

};
