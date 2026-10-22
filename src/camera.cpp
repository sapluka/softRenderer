#include "camera.h"

void camera::initCamera() {
    position = Eigen::Vector3f(EYE_POS_X, EYE_POS_Y, EYE_POS_Z);
    fov_y = pi / 3.0f; // 60 degrees
    aspect_ratio = 4.0f / 3.0f; // Assuming a 4:3 aspect ratio
    worldUp = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    target = Eigen::Vector3f(0.0f, 0.0f, (z_near+z_far)/2.0f); 
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

 camera::camera(const Eigen::Vector3f& pos,
               const Eigen::Vector3f& tgt,
               const Eigen::Vector3f& up)
    : position(pos),
      fov_y(pi / 3.0f),
      aspect_ratio(4.0f / 3.0f),
      direction((tgt - pos).normalized()),
      worldUp(up),
      target(tgt)
{}