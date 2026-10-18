#include "rasterizer.h"


void rasterizer::initRasterizer() {
    // 初始化颜色缓冲区为黑色
    for (int i = 0; i < SCREEN_HEIGHT; ++i) {
        for (int j = 0; j < SCREEN_WIDTH; ++j) {
            frameBuffer[i][j] = Eigen::Array3f(0.0f, 0.0f, 0.0f);
            zBuffer[i][j] = std::numeric_limits<float>::infinity(); 
            normalBuffer[i][j] = Eigen::Array3f(0.0f, 0.0f, 0.0f);
        }
    }
}

void rasterizer::setBuffers() 
{
     float _y = std::tan(fov_y / 2.0f) * z_near;
     float _x = _y * aspect_ratio;
     
}