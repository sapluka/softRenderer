#pragma once
#include <Eigen/Core>
#include "parameter.h"



class camera{
    private:
    Eigen::Vector3f position ;
    float fov_y;
    float aspect_ratio;
    Eigen::Vector3f direction;//摄像机的朝向
    Eigen::Vector3f up; // 相机当前的上方向，会随相机旋转
    Eigen::Vector3f target;
    float nearPlane = z_near; // 相机空间中的负 Z 坐标
    float farPlane = z_far;
    Eigen::Vector3f modelCenter = Eigen::Vector3f::Zero();
    float modelRadius = 0.0f;
    void updateClipPlanes();
    public:
    camera() ;
    camera(const Eigen::Vector3f& pos, const Eigen::Vector3f& tgt, const Eigen::Vector3f& up);
    camera(const Eigen::Vector3f& point_min, const Eigen::Vector3f& point_max);//按模型包围球自动取景
    void initCamera();//初始化位置，FOV等等
    Eigen::Vector3f getPosition() const { return position; }
    Eigen::Vector3f getDirection() const { return direction; }
    Eigen::Vector3f getTarget() const { return target; }
    Eigen::Vector3f getUp() const { return up; }
    float getNearPlane() const { return nearPlane; }
    float getFarPlane() const { return farPlane; }
    void setPosition(const Eigen::Vector3f& pos);
    void setUp(const Eigen::Vector3f& newUp);
    void setTarget(const Eigen::Vector3f& tar);

    
};
