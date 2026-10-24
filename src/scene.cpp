#include "scene.h"
#include "parameter.h"
#include "transform.h"
#include <Eigen/core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

scene::scene(objLoader& model, rasterizer& rasterizer, camera& camera):
    model_(model),
    rasterizer_(rasterizer),
    camera_(camera)
{
    leftMouseButton = buttonStatus();
}

void scene::init() {
    // 初始化模型
    initGLFW();
    createWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "test");
    makeCurrentContext();
    initGlad();

}

void scene::initGLFW() {

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

}

void scene::createWindow(int width, int height, const char* title)
{

    window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if(!window)
    {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return;
    }

}

void scene::makeCurrentContext() {
    glfwMakeContextCurrent(window);
}

void scene::initGlad() {
    if (!gladLoadGL(glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        glfwDestroyWindow(window);
        glfwTerminate();
        return;
    }
}

void scene::releaseResources()
{
    if (window) {
        glfwMakeContextCurrent(window);
        rasterizer_.releaseDrawResources();
        glfwDestroyWindow(window);
        window = nullptr;
    }
    glfwTerminate();
}

void scene::render() {
    bool firstFrame = true;
    const unsigned int workerCount = std::min(
        std::max(1u, std::thread::hardware_concurrency()), static_cast<unsigned int>(SCREEN_HEIGHT));
    std::cout << "Rendering " << SCREEN_WIDTH << 'x' << SCREEN_HEIGHT << ", "
              << model_.triangles().size() << " triangles, " << workerCount << " workers" << std::endl;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        {
            glfwSetWindowShouldClose(window, true);
        }
            // 清除屏幕
        glClearColor(0.1f, 0.1f, 0.6f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        checkRotate(window);

        // 渲染场景
        //这里我们set了一个buffer，这个buffer是相对于摄像机的。我们标记了所有intersected triangles，
        // 此时就可以开始rasterize。下面这两行函数基本上就是针对没有阴影的情况下，
        // 我们去进行一个渲染。如果我们想要加入阴影和全局光照，肯定是要再用两个函数。
        //一个是更新Shadow Map的函数，一个是计算GI的函数吗？
        const auto frameStart = std::chrono::steady_clock::now();
        int lastCompletedRows = -1;
        const bool completed = rasterizer_.setGBuffers(
            model_, camera_.getPosition(), camera_.getTarget(), camera_.getUp(), workerCount,
            [this, &lastCompletedRows](int completedRows, int totalRows) {
                glfwPollEvents();
                if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
                    glfwSetWindowShouldClose(window, true);
                if (completedRows != lastCompletedRows) {
                    const std::string title = "Rendering " + std::to_string(completedRows)
                                            + "/" + std::to_string(totalRows) + " rows";
                    glfwSetWindowTitle(window, title.c_str());
                    lastCompletedRows = completedRows;
                }
                return !glfwWindowShouldClose(window);
            });
        if (!completed) break;
        const auto traceEnd = std::chrono::steady_clock::now();
        sync();
        rasterizer_.draw();
        glfwSwapBuffers(window);
        const auto frameMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - frameStart).count();
        const std::string title = "Renderer - " + std::to_string(frameMilliseconds) + " ms";
        glfwSetWindowTitle(window, title.c_str());
        if (firstFrame) {
            const auto traceMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                traceEnd - frameStart).count();
            std::cout << "First frame rendered in " << frameMilliseconds
                      << " ms (trace: " << traceMilliseconds << " ms, display: "
                      << frameMilliseconds - traceMilliseconds << " ms)" << std::endl;
            firstFrame = false;
        }
    }
}

void scene::checkRotate(GLFWwindow* window)
{
    const bool isPressedNow = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    double mouseX, mouseY;
    glfwGetCursorPos(window, &mouseX, &mouseY);

    if(isPressedNow)
    {
        if(leftMouseButton.isPressed)
        {
            rotateCamera(leftMouseButton.position.x(), leftMouseButton.position.y(), mouseX, mouseY);
        }

        leftMouseButton.position = Eigen::Vector2f(static_cast<float>(mouseX), static_cast<float>(mouseY));

    }

    leftMouseButton.isPressed = isPressedNow;
}

void scene::rotateCamera(double x,double y, double u, double v)
{
    const Eigen::Vector2f screenMoveDirection(static_cast<float>(u - x), static_cast<float>(v - y));
    const float dragLength = screenMoveDirection.norm();
    if (!std::isfinite(dragLength) || dragLength <= 1e-6f) {
        return;
    }

    const Eigen::Vector3f target = camera_.getTarget();
    const Eigen::Vector3f offset = camera_.getPosition() - target;//b，即相机的后方向
    const float radius = offset.norm();
    if (!offset.allFinite() || !std::isfinite(radius) || radius <= 1e-6f) {
        return;
    }

    const Eigen::Matrix4f view = Transform::getViewMatrix(camera_.getPosition(), target, camera_.getUp());
    const Eigen::Vector3f right = view.block<1, 3>(0, 0).transpose();
    const Eigen::Vector3f up = view.block<1, 3>(1, 0).transpose();
    const Eigen::Vector3f back = offset / radius;

    // 当前投影矩阵的 X/Y 尺度都为负：屏幕右/下分别对应 -right/+up。
    const Eigen::Vector3f tangent = (-screenMoveDirection.x() * right + screenMoveDirection.y() * up).normalized();//tangent是二维向量三维化的结果
    const Eigen::Vector3f axis = back.cross(tangent).normalized();
    constexpr float radiansPerScreenUnit = 0.005f;
    const Eigen::AngleAxisf rotation(dragLength * radiansPerScreenUnit, axis);

    const Eigen::Vector3f rotatedOffset = (rotation * offset).normalized() * radius;
    camera_.setPosition(target + rotatedOffset);
    // up 是方向向量，只旋转，不加 target；与半径向量共用同一个旋转。
    camera_.setUp(rotation * up);
}



void scene::sync()
{
    // 这里可以实现与其他线程或系统的同步逻辑
    // 例如，等待渲染完成或处理输入事件等
    glfwSwapInterval(1); // 设置垂直同步
}
