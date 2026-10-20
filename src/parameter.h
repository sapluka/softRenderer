#pragma once
#include <cmath>

constexpr float EYE_POS_X = 0.0;
constexpr float EYE_POS_Y = 0.0;
constexpr float EYE_POS_Z = 0.0;
constexpr float z_near = -0.1f;
constexpr float z_far = -100.0f;
//这里我把z_near和z_far都改成负数了，才是右手系，要检查有z的地方
constexpr float pi = 3.14159;
constexpr float fov_y = pi/3;
constexpr float aspect_ratio = 4.0f/3.0f;
constexpr int SCREEN_WIDTH = 800;
constexpr int SCREEN_HEIGHT = 600;  
constexpr float LIGHT_POS_X = 0.0f;
constexpr float LIGHT_POS_Y = 10.0f;
constexpr float LIGHT_POS_Z = 0.0f;
