#pragma once

#include <vector>
#include <Eigen/core>
#include <Eigen/Geometry>
#include "objLoader.h"
#include "parameter.h"
#include "rasterizer.h"
#include "camera.h"
#include "shadeModel.h"
#include <glad/gl.h>
#include <GLFW/glfw3.h>

struct buttonStatus//这里记录的都是上一帧的状态
{
    bool isPressed;
    Eigen::Vector2f position;

    buttonStatus() : isPressed(false) , position(Eigen::Vector2f::Zero())
    {

    }
};

class scene {
    private:
    objLoader& model_;
    rasterizer& rasterizer_;//变量名尽量不要和类名一样哦，否则会发生遮蔽
    camera& camera_;
    GLFWwindow* window = nullptr;
    bool cameraDragging = false;
    double dragStartX = 0.0;
    double dragStartY = 0.0;
    buttonStatus leftMouseButton;
    public:
    scene(objLoader& model, rasterizer& rasterizer, camera& camera);

    void init();
    void initGLFW();
    void createWindow(int width, int height, const char* title);
    void initGlad();
    void makeCurrentContext();
    void render();//主循环
    void rotateCamera(double x,double y, double u, double v);//使摄像机在固定的球面上旋转
    void releaseResources();//释放资源
    void sync();
    void checkRotate(GLFWwindow* window);//检查是否需要旋转摄像机，通过检测鼠标左键的状态和位置实现

  
};
