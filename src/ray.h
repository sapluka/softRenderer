#pragma once

#include "parameter.h"
#include <Eigen/core>
#include <Eigen/Geometry>
#include <cmath>



class ray{
    private:
        Eigen::Vector3f origin;
        Eigen::Vector3f direction;//规定所有方向都应该是单位向量
        bool intersect;
        Eigen::Vector3f intersectionPoint;
    public:
        ray() = default;
        ray(const Eigen::Vector3f& o, const Eigen::Vector3f& d) : origin(o), direction(d) {}
        Eigen::Vector3f getOrigin() const { return origin; }
        Eigen::Vector3f getDirection() const { return direction; }
        
        //下面这个函数会更新ray的intersect和intersectionPoint两个成员变量
        void findIntersectionPoint(Eigen::Vector3f& point0, Eigen::Vector3f& point1, Eigen::Vector3f& point2);
        bool getIntersect()const {return intersect;};
        Eigen::Vector3f getIntersectionPoint() const {return intersectionPoint;};
        
};