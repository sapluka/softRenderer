#include "camera.h"

void camera::initCamera() {
    position = Eigen::Vector3f(EYE_POS_X, EYE_POS_Y, EYE_POS_Z);
    fov_y = pi / 3.0f; // 60 degrees
    aspect_ratio = 4.0f / 3.0f; // Assuming a 4:3 aspect ratio
}