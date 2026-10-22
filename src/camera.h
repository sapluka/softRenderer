#pragma once
#include <Eigen/Core>
#include "parameter.h"



class camera{
    private:
    Eigen::Vector3f position ;
    float fov_y;
    float aspect_ratio;
    Eigen::Vector3f direction;
    Eigen::Vector3f worldUp;
    Eigen::Vector3f target;
    public:
    camera() ;
    camera(const Eigen::Vector3f& pos, const Eigen::Vector3f& tgt, const Eigen::Vector3f& up);
    void initCamera();//初始化位置，FOV等等
    
};