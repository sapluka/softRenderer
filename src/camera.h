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
    public:
    camera() ;
    camera(const Eigen::Vector3f& pos, const Eigen::Vector3f& tgt, const Eigen::Vector3f& up);
    void initCamera();//初始化位置，FOV等等
    Eigen::Vector3f getPosition() const { return position; }
    Eigen::Vector3f getDirection() const { return direction; }
    Eigen::Vector3f getTarget() const { return target; }
    Eigen::Vector3f getUp() const { return up; }
    void setPosition(const Eigen::Vector3f& pos);
    void setUp(const Eigen::Vector3f& newUp);

    
};
