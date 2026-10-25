#pragma once

#include "parameter.h"
#include <vector>
#include <Eigen/core>
#include <limits>
#include <cmath>
#include <functional>
#include "ray.h"
#include "objLoader.h"
#include "transform.h"
#include "camera.h"
#include "shadeModel.h"
#include <unordered_set>


class rasterizer {
    friend struct RasterizerDrawTestAccess;
    friend struct RasterizerTraceTestAccess;
    private:
        std::vector<std::vector<Eigen::Vector3f>> frameBuffer; // 颜色缓冲区
        std::vector<std::vector<float>> zBuffer; // 深度缓冲区
        std::vector<std::vector<Eigen::Vector3f>> normalBuffer; // 法线缓冲区
        std::vector<objLoader::Triangle> IntersectedTriangles;
        shadeModel& shadeModel;
        const Eigen::Vector3f backgroundColor{0.5f, 0.8f, 0.9f};
        // 仅用于把 CPU 颜色缓冲显示到窗口，第一次 draw 时创建。
        unsigned int displayTexture = 0;
        unsigned int displayProgram = 0;
        unsigned int displayVAO = 0;
        std::vector<float> displayPixels;
        //
        public:
        rasterizer() = delete;
        rasterizer(class shadeModel& s) : shadeModel(s) {}
        rasterizer(const rasterizer&) = delete;
        rasterizer& operator=(const rasterizer&) = delete;
        void initRasterizer();
        bool setGBuffers(const objLoader& model,const Eigen::Vector3f& origin,const Eigen::Vector3f& target,
                         const Eigen::Vector3f& up = Eigen::Vector3f::UnitY(),
                         unsigned int threadCount = 0,
                         const std::function<bool(int, int)>& progress = {});
        void rasterize(const objLoader& model, const Eigen::Vector3f& eye,const Eigen::Vector3f& target,
                       const Eigen::Vector3f& up = Eigen::Vector3f::UnitY());//负责光栅化,着色以及更新frameBuffer
        void draw();//负责把frameBuffer呈现在屏幕上
        void releaseDrawResources(); // 销毁窗口前，在同一个 OpenGL 上下文中调用
        void clear();//清空frameBuffer,zBuffer,normalBuffer

};
