#pragma once

#include "parameter.h"
#include <vector>
#include <Eigen/core>
#include <limits>
#include <cmath>
 

class rasterizer {
    private:
        Eigen::Array3f frameBuffer[SCREEN_HEIGHT][SCREEN_WIDTH]; // 颜色缓冲区
        float zBuffer[SCREEN_HEIGHT][SCREEN_WIDTH]; // 深度缓冲区
        Eigen::Array3f normalBuffer[SCREEN_HEIGHT][SCREEN_WIDTH]; // 法线缓冲区 

        public:
        rasterizer() {};
        void initRasterizer();
        void setBuffers();

};
