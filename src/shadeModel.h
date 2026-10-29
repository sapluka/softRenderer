#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include "light.h"
#include "parameter.h"
#include "objLoader.h"

class shadeModel {
    private:
        light sceneLight;
    public:
        shadeModel() = default;
        shadeModel(const light& l) : sceneLight(l) {}
        Eigen::Vector3f computeShading(const objLoader::Material& material, const Eigen::Vector3f& camera,const Eigen::Vector3f& point,const Eigen::Vector3f& normal , float visibility) const;
        Eigen::Vector3f getLightPosition() const { return sceneLight.getPosition(); }
        const light& getLight() const { return sceneLight; }
        void setLight(const light& l) { sceneLight = l; }
};
