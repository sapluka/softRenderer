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

    public:

    //transform函数封装了MVP，NDC和Viewport矩阵转换
    static Eigen::Vector3f transform(const Eigen::Vector3f& point, const Eigen::Vector3f& eye,const Eigen::Vector3f& target,
                                     const Eigen::Vector3f& up = Eigen::Vector3f::UnitY());//封装了各种矩阵，掌管整个流程
    static Eigen::Vector3f transform(const Eigen::Vector3f& point, const Eigen::Vector3f& eye, const Eigen::Vector3f& target,
                                     const Eigen::Vector3f& up, float& clipW,
                                     float nearPlane = z_near, float farPlane = z_far);
    
    static Eigen::Matrix4f getModelMatrix(std::vector<float>&scales,std::vector<float>&translates,Eigen::Vector3f axis,float theta);//先暂时不管输入参数
    static Eigen::Matrix4f getViewMatrix(const Eigen::Vector3f& eye,const Eigen::Vector3f& target,
                                        const Eigen::Vector3f& up = Eigen::Vector3f::UnitY());
    static Eigen::Matrix4f getProjectionMatrix(float nearPlane = z_near, float farPlane = z_far);
    static Eigen::Vector4f getNDC(const Eigen::Vector4f& clipSpacePoint);
    static Eigen::Matrix4f getViewportMatrix();

    //下面这个函数是用来计算重心坐标的
    static bool computeBarycentric2D(const Eigen::Vector3f& point, const Eigen::Vector3f& first, const Eigen::Vector3f& second,
                                    const Eigen::Vector3f& third, float& alpha, float& beta, float& gamma);
    static void computeBarycentric3D(const Eigen::Vector3f& p, const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c, float& alpha, float& beta, float& gamma);

    

};
