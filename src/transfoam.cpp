#include "transform.h"
#include <Eigen/core>
#include <cmath>

Eigen::Matrix4f Transform::getModelMatrix(std::vector<float>&scales,std::vector<float>&translates,Eigen::Vector3f axis,float theta)
{
    Eigen::Matrix4f scale;
    scale<<scales[0],0.0f,0.0f,0.0f,
           0.0f,scales[1],0.0f,0.0f,
           0.0f,0.0f,scales[2],0.0f,
           0.0f,0.0f,0.0f,1.0f;

    Eigen::Matrix4f rotation;
    rotation<<cos(theta)+axis.x()*axis.x()*(1-cos(theta)),axis.x()*axis.y()*(1-cos(theta))-axis.z()*sin(theta),axis.x()*axis.z()*(1-cos(theta))+axis.y()*sin(theta),0.0f,
    axis.y()*axis.x()*(1-cos(theta))+axis.z()*sin(theta),cos(theta)+axis.y()*axis.y()*(1-cos(theta)),axis.y()*axis.z()*(1-cos(theta))-axis.x()*sin(theta),0.0f,
    axis.z()*axis.x()*(1-cos(theta))-axis.y()*sin(theta),axis.z()*axis.y()*(1-cos(theta))+axis.x()*sin(theta),cos(theta)+axis.z()*axis.z()*(1-cos(theta)),0.0f,
    0.0f,0.0f,0.0f,1.0f;

    Eigen::Matrix4f translate;
    translate<<1.0f,0.0f,0.0f,translates[0],
               0.0f,1.0f,0.0f,translates[1],
               0.0f,0.0f,1.0f,translates[2],
               0.0f,0.0f,0.0f,1.0f;

    return translate*rotation*scale;
               
}

Eigen::Matrix4f Transform::getViewMatrix(std::vector<float> &translates, Eigen::Vector3f axis, float theta)
{
    Eigen::Matrix4f translate;//此处是摄像机的pos的位移矩阵的逆
    translate<<1.0f,0.0f,0.0f,-translates[0],
               0.0f,1.0f,0.0f,-translates[1],
               0.0f,0.0f,1.0f,-translates[2],
               0.0f,0.0f,0.0f,1.0f;   

    Eigen::Matrix4f rotation;
    rotation<<cos(theta)+axis.x()*axis.x()*(1-cos(theta)),axis.x()*axis.y()*(1-cos(theta))-axis.z()*sin(theta),axis.x()*axis.z()*(1-cos(theta))+axis.y()*sin(theta),0.0f,
    axis.y()*axis.x()*(1-cos(theta))+axis.z()*sin(theta),cos(theta)+axis.y()*axis.y()*(1-cos(theta)),axis.y()*axis.z()*(1-cos(theta))-axis.x()*sin(theta),0.0f,
    axis.z()*axis.x()*(1-cos(theta))-axis.y()*sin(theta),axis.z()*axis.y()*(1-cos(theta))+axis.x()*sin(theta),cos(theta)+axis.z()*axis.z()*(1-cos(theta)),0.0f,
    0.0f,0.0f,0.0f,1.0f;
    
    return rotation.inverse()*translate;

}

Eigen::Matrix4f Transform::getProjectionMatrix()
{
    float f = 1.0f / tan(fov_y / 2.0f);

    Eigen::Matrix4f projection;
    projection<<f/aspect_ratio,0.0f,0.0f,0.0f,
                0.0f,f,0.0f,0.0f,
                0.0f,0.0f,-(z_near+z_far)/(z_far-z_near),-2.0f*z_near*z_far/(z_far-z_near),
                0.0f,0.0f,-1.0f,0.0f;
    return projection;
    //这里可能会出现z的正负定义问题，明天检查一下
    //注意这一步得到的仍然是clip space的坐标，之后还需要进行透视除法得到NDC坐标
}

Eigen::Vector3f Transform::getNDC(const Eigen::Vector4f& clipSpacePoint)
{
    Eigen::Vector3f ndc;
    ndc.x() = clipSpacePoint.x() / clipSpacePoint.w();
    ndc.y() = clipSpacePoint.y() / clipSpacePoint.w();
    ndc.z() = clipSpacePoint.z() / clipSpacePoint.w();
    return ndc;
}

Eigen::Matrix4f Transform::getViewportMatrix()
{
    Eigen::Matrix4f viewport;
    viewport<<SCREEN_WIDTH/2.0f,0.0f,0.0f,SCREEN_WIDTH/2.0f,
              0.0f,-SCREEN_HEIGHT/2.0f,0.0f,SCREEN_HEIGHT/2.0f,
              0.0f,0.0f,1.0f,0.0f,
              0.0f,0.0f,0.0f,1.0f;
    return viewport;
}