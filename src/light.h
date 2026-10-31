#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "parameter.h"


//如果有场景中的light，那么他的数据应该保存在一个数组里面然后分批创建为light
//如果有必要甚至可以去创建一个vector<light>
//其实从这次实践就能看出来继承并不那么好用
class light {
    private:
        Eigen::Vector3f position;
        Eigen::Vector3f target = Eigen::Vector3f::Zero();
        Eigen::Vector3f up = Eigen::Vector3f::UnitZ();
        Eigen::Vector3f color;
        float intensity;
    
    public:
        light(const Eigen::Vector3f& pos, const Eigen::Vector3f& col, float inten)
            : position(pos), color(col), intensity(inten) {}
        light()
        {
            position = Eigen::Vector3f(LIGHT_POS_X, LIGHT_POS_Y, LIGHT_POS_Z);
            color = Eigen::Vector3f(1.0f, 1.0f, 1.0f);
            intensity = 1.0f;
        }
        Eigen::Vector3f getPosition() const { return position; }
        Eigen::Vector3f getTarget() const { return target; }
        Eigen::Vector3f getUp() const { return up; }
        Eigen::Vector3f getColor() const { return color; }
        float getIntensity() const { return intensity; }
        void setPosition(const Eigen::Vector3f& pos) { position = pos; }
        void setTarget(const Eigen::Vector3f& tar) { target = tar; }
        void setUp(const Eigen::Vector3f& newUp) { up = newUp.normalized(); }
};
