#include "camera.h"
#include <Eigen/Core>
#include <algorithm>
#include <cmath>

void camera::initCamera() {
    position = Eigen::Vector3f(EYE_POS_X, EYE_POS_Y, EYE_POS_Z);
    fov_y = pi / 3.0f; // 60 degrees
    aspect_ratio = 4.0f / 3.0f; // Assuming a 4:3 aspect ratio
    up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    target = Eigen::Vector3f(0.0f, 0.0f, (z_near+z_far)/2.0f); 
    nearPlane = z_near;
    farPlane = z_far;
    modelCenter = Eigen::Vector3f::Zero();
    modelRadius = 0.0f;
    /*
现在有一个显而易见的问题：物体的位置不应该由.obj决定吗？
我在这里设置target真的有用吗？
所以理论上应该这样做：
要么从objLoader中读取物体的包围盒，然后把target设置为包围盒的中心
要么主动把Model放到target处

这里要动态改变的应该就是target,position
aspect_ratio可能也要改变吧？我决定应该设置一个窗口类，
然后窗口本身具有属性screen_width和screen_height，这两个是可以动态改变的
而aspect_ratio也应该由这两个属性计算出来
    */
    direction = (target - position).normalized();
}

camera::camera():camera(Eigen::Vector3f(EYE_POS_X, EYE_POS_Y, EYE_POS_Z),
                        Eigen::Vector3f(0.0f, 0.0f, (z_near+z_far)/2.0f),
                        Eigen::Vector3f(0.0f, 1.0f, 0.0f))
 {}

camera::camera(const Eigen::Vector3f& point_min, const Eigen::Vector3f& point_max)
    : camera()
{
    modelCenter = (point_min + point_max) * 0.5f;
    modelRadius = std::max((point_max - point_min).norm() * 0.5f, 1e-6f);
    target = modelCenter;

    // 用较小的半视角保证模型的宽、高都能进入画面，并留出 10% 余量。
    const float halfFovY = fov_y * 0.5f;
    const float halfFovX = std::atan(aspect_ratio * std::tan(halfFovY));
    const float distance = 1.1f * modelRadius / std::sin(std::min(halfFovX, halfFovY));
    setPosition(target - direction * distance);
}

void camera::updateClipPlanes()
{
    if (modelRadius <= 0.0f) return;
    const float distance = (modelCenter - position).dot(direction);
    const float margin = modelRadius * 0.05f;
    const float minDistance = std::max(modelRadius * 1e-4f, 1e-6f);
    nearPlane = -std::max(minDistance, distance - modelRadius - margin);
    farPlane = -std::max(minDistance * 2.0f, distance + modelRadius + margin);
}

 camera::camera(const Eigen::Vector3f& pos,
               const Eigen::Vector3f& tgt,
               const Eigen::Vector3f& up)
    : position(pos),
      fov_y(pi / 3.0f),
      aspect_ratio(4.0f / 3.0f),
      direction((tgt - pos).normalized()),
      up(up.normalized()),
      target(tgt)
{}

void camera::setPosition(const Eigen::Vector3f& pos)
{
    position = pos;
    const Eigen::Vector3f forward = target - position;
    direction = forward.normalized();
    updateClipPlanes();
}

void camera::setUp(const Eigen::Vector3f& newUp)
{
    up = newUp.normalized();
}

void camera::setTarget(const Eigen::Vector3f& tar)
{
    target = tar;
    direction = (target - position).normalized();
    updateClipPlanes();
}
