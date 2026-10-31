#pragma once

#include "parameter.h"
#include <vector>
#include <array>
#include <Eigen/Core>
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <exception>
#include <fstream>
#include <iterator>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <cmath>
#include <functional>
#include "objLoader.h"
#include "transform.h"
#include "camera.h"
#include "shadeModel.h"


class rasterizer {
    friend struct RasterizerDrawTestAccess;
    friend struct RasterizerTraceTestAccess;
    friend struct RasterizerMSAATestAccess;
    friend struct RasterizerShadowTestAccess;
    private:
        struct ScreenTriangle {
            std::vector<Eigen::Vector3f> vertices;
            std::vector<float> inverseW;
            std::array<int, 4> bounds;
        };
        std::vector<std::vector<Eigen::Vector3f>> frameBuffer; // 颜色缓冲区
        std::vector<std::vector<float>> zBuffer; // 深度缓冲区
        std::vector<std::vector<std::array<float, 4>>> subZBuffer; // 每个像素的四份采样深度
        std::vector<std::vector<std::array<Eigen::Vector3f, 4>>> subFrameBuffer; // 每个像素的四份采样颜色
        std::vector<std::vector<float>> zBufferLight; // 光源深度缓冲区
        Transform lightTransform; // 与当前阴影图对应的变换，供片元查询复用
        shadeModel& shadeModel;
        const Eigen::Vector3f backgroundColor{0.5f, 0.8f, 0.9f};
        // 仅用于把 CPU 颜色缓冲显示到窗口，第一次 draw 时创建。
        unsigned int displayTexture = 0;
        unsigned int displayProgram = 0;
        unsigned int displayVAO = 0;
        std::vector<float> displayPixels;
        void clearDepthBuffer(std::vector<std::vector<float>>& buffer);
        void clearSubBuffers();
        void resolveMSAA();//把每个像素的子像素的四份采样颜色平均后写入frameBuffer的单个像素
        bool prepareTriangle(const objLoader::Triangle& triangle, const Transform& viewTransform,
                             ScreenTriangle& screenTriangle);
        void updateDepthBuffer(const std::vector<Eigen::Vector3f>& screenTriangle,
                               std::vector<std::vector<float>>& buffer, int x, int y);
        float getShadowVisibility(const Eigen::Vector3f& worldPos) const;
        
        void rasterizePixel(const objLoader::Triangle& triangle, const objLoader::Material& material,
            const ScreenTriangle& screenTriangle,
            const Eigen::Vector3f& eye, int x, int y);//负责记录每个子像素的color和z
            
        
        public:
        rasterizer() = delete;
        rasterizer(class shadeModel& s) : shadeModel(s) {}
        rasterizer(const rasterizer&) = delete;
        rasterizer& operator=(const rasterizer&) = delete;
        void initRasterizer();
        //从光源视角生成zBufferLight
        void setGBuffers(const objLoader& model,const Eigen::Vector3f& origin,const Eigen::Vector3f& target,
                         const Eigen::Vector3f& up = Eigen::Vector3f::UnitY());
        void rasterize(const objLoader& model, const Eigen::Vector3f& eye,const Eigen::Vector3f& target,
                       const Eigen::Vector3f& up = Eigen::Vector3f::UnitY(),
                       float nearPlane = z_near, float farPlane = z_far);//负责光栅化,着色以及更新frameBuffer
        void draw();//负责把frameBuffer呈现在屏幕上
        void releaseDrawResources(); // 销毁窗口前，在同一个 OpenGL 上下文中调用
        void clear();//清空颜色、深度以及子样本缓冲


};
