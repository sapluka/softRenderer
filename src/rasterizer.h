#pragma once

#include "parameter.h"
#include <vector>
#include <Eigen/core>
#include <limits>
#include <cmath>
#include "ray.h"
#include "objLoader.h"
#include "transform.h"
#include "camera.h"
#include "shadeModel.h"
 

class rasterizer {
    private:
        Eigen::Array3f frameBuffer[SCREEN_HEIGHT][SCREEN_WIDTH]; // 颜色缓冲区
        float zBuffer[SCREEN_HEIGHT][SCREEN_WIDTH]; // 深度缓冲区
        Eigen::Array3f normalBuffer[SCREEN_HEIGHT][SCREEN_WIDTH]; // 法线缓冲区 
        std::vector<objLoader::Triangle> IntersectedTriangles;
        shadeModel shadeModel;
        public:
        rasterizer() {};
        void initRasterizer();
        //下面这个是基于眼睛的G-buffer，但是我们的GBuffer是不是应该基于光源呢？
        void setGBuffers(const objLoader& model);
        void setGBuffers_Camera(const objLoader& model);
        void rasterize(const objLoader& model, const Eigen::Vector3f& eye,const Eigen::Vector3f& target);//负责光栅化,着色以及更新frameBuffer
        void draw();//负责把frameBuffer呈现在屏幕上
        void shading();

};
