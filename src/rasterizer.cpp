#include "rasterizer.h"
#include "objLoader.h"
#include "parameter.h"
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <Eigen/core>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <iterator>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

void rasterizer::initRasterizer() {
    frameBuffer.assign(SCREEN_HEIGHT,std::vector<Eigen::Vector3f>(SCREEN_WIDTH,backgroundColor));
    zBuffer.assign(SCREEN_HEIGHT,std::vector<float>(SCREEN_WIDTH,std::numeric_limits<float>::infinity()));
    normalBuffer.assign(SCREEN_HEIGHT,std::vector<Eigen::Vector3f>(SCREEN_WIDTH,Eigen::Vector3f::Zero()));
}



bool rasterizer::setGBuffers(const objLoader& model, const Eigen::Vector3f& origin,
                            const Eigen::Vector3f& target, const Eigen::Vector3f& up,
                            unsigned int threadCount,
                            const std::function<bool(int, int)>& progress) {
    if (frameBuffer.empty()) initRasterizer();
    clear();
    const int height = static_cast<int>(frameBuffer.size());
    const int width = static_cast<int>(frameBuffer.front().size());
    if (width == 0) return true;

    const Eigen::Vector3f cameraOrigin = origin;
    const Eigen::Matrix4f view = Transform::getViewMatrix(cameraOrigin, target, up);
    const Eigen::Matrix3f cameraToWorld = view.block<3, 3>(0, 0).transpose();
    const Eigen::Matrix4f projection = Transform::getProjectionMatrix();
    const auto& triangles = model.triangles();
    if (threadCount == 0) threadCount = std::max(1u, std::thread::hardware_concurrency());
    const unsigned int workerCount = std::min(threadCount, static_cast<unsigned int>(height));
    const std::size_t noHit = std::numeric_limits<std::size_t>::max();
    std::vector<std::vector<std::size_t>> firstHits(
        workerCount, std::vector<std::size_t>(triangles.size(), noHit));
    std::atomic<int> nextRow{0};
    std::atomic<int> completedRows{0};
    std::atomic<unsigned int> remainingWorkers{workerCount};
    std::atomic<bool> cancelled{false};
    std::exception_ptr workerError;
    std::mutex errorMutex;
    std::vector<std::thread> workers;
    workers.reserve(workerCount);

    if (progress && !progress(0, height)) return false;

    const auto renderRows = [&](unsigned int workerIndex) {
        try {
            while (!cancelled.load(std::memory_order_relaxed)) {
                const int row = nextRow.fetch_add(1, std::memory_order_relaxed);
                if (row >= height) break;
                for (int column = 0; column < width; ++column) {
                    if (cancelled.load(std::memory_order_relaxed)) break;
                    const float ndcX = 2.0f * (column + 0.5f) / width - 1.0f;
                    const float ndcY = 1.0f - 2.0f * (row + 0.5f) / height;
                    const Eigen::Vector3f cameraDirection(
                        ndcX / projection(0, 0), ndcY / projection(1, 1), -1.0f);
                    const Eigen::Vector3f direction = (cameraToWorld * cameraDirection).normalized();
                    ray pixelRay(cameraOrigin, direction);
                    const objLoader::Triangle* nearestTriangle = nullptr;
                    Eigen::Vector3f nearestPoint = Eigen::Vector3f::Zero();
                    float nearestDistanceSquared = std::numeric_limits<float>::infinity();

                    for (std::size_t triangleIndex = 0; triangleIndex < triangles.size(); ++triangleIndex) {
                        const auto& triangle = triangles[triangleIndex];
                        Eigen::Vector3f first = triangle.vertices[0].position;
                        Eigen::Vector3f second = triangle.vertices[1].position;
                        Eigen::Vector3f third = triangle.vertices[2].position;
                        pixelRay.findIntersectionPoint(first, second, third);
                        if (!pixelRay.getIntersect()) continue;

                        const std::size_t pixelIndex = static_cast<std::size_t>(row) * width + column;
                        firstHits[workerIndex][triangleIndex] = std::min(
                            firstHits[workerIndex][triangleIndex], pixelIndex);
                        const Eigen::Vector3f point = pixelRay.getIntersectionPoint();
                        const float distanceSquared = (point - cameraOrigin).squaredNorm();
                        if (distanceSquared < nearestDistanceSquared) {
                            nearestDistanceSquared = distanceSquared;
                            nearestTriangle = &triangle;
                            nearestPoint = point;
                        }
                    }

                    if (!nearestTriangle) continue;
                    const auto& triangle = *nearestTriangle;
                    float alpha, beta, gamma;
                    Transform::computeBarycentric3D(
                        nearestPoint, triangle.vertices[0].position,
                        triangle.vertices[1].position, triangle.vertices[2].position,
                        alpha, beta, gamma);
                    Eigen::Vector3f normal = alpha * triangle.vertices[0].normal
                                          + beta * triangle.vertices[1].normal
                                          + gamma * triangle.vertices[2].normal;
                    if (!normal.allFinite() || normal.squaredNorm() <= 1e-12f) {
                        normal = (triangle.vertices[1].position - triangle.vertices[0].position)
                            .cross(triangle.vertices[2].position - triangle.vertices[0].position);
                    }
                    zBuffer[row][column] = nearestPoint.z();
                    normalBuffer[row][column] = normal.normalized();
                    frameBuffer[row][column] = shadeModel.computeShading(
                        model.materialFor(triangle), cameraOrigin,
                        nearestPoint, normalBuffer[row][column]);
                }
                if (!cancelled.load(std::memory_order_relaxed))
                    completedRows.fetch_add(1, std::memory_order_relaxed);
            }
        } catch (...) {
            std::lock_guard<std::mutex> lock(errorMutex);
            if (!workerError) workerError = std::current_exception();
            cancelled.store(true, std::memory_order_relaxed);
        }
        remainingWorkers.fetch_sub(1, std::memory_order_release);
    };

    const auto joinWorkers = [&] {
        for (auto& worker : workers) {
            if (worker.joinable()) worker.join();
        }
    };
    try {
        for (unsigned int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
            workers.emplace_back(renderRows, workerIndex);
        while (remainingWorkers.load(std::memory_order_acquire) != 0) {
            if (progress && !progress(completedRows.load(std::memory_order_relaxed), height))
                cancelled.store(true, std::memory_order_relaxed);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } catch (...) {
        cancelled.store(true, std::memory_order_relaxed);
        joinWorkers();
        throw;
    }
    joinWorkers();
    if (workerError) std::rethrow_exception(workerError);
    if (cancelled.load(std::memory_order_relaxed)) return false;

    std::vector<std::pair<std::size_t, std::size_t>> hitOrder;
    for (std::size_t triangleIndex = 0; triangleIndex < triangles.size(); ++triangleIndex) {
        std::size_t firstPixel = noHit;
        for (const auto& workerHits : firstHits)
            firstPixel = std::min(firstPixel, workerHits[triangleIndex]);
        if (firstPixel != noHit) hitOrder.emplace_back(firstPixel, triangleIndex);
    }
    std::sort(hitOrder.begin(), hitOrder.end());
    for (const auto& hit : hitOrder) IntersectedTriangles.push_back(triangles[hit.second]);
    return !progress || progress(height, height);
}

void rasterizer::rasterize(const objLoader& model, const Eigen::Vector3f& eye,const Eigen::Vector3f& target,
                           const Eigen::Vector3f& up) {

    if (frameBuffer.empty()) initRasterizer();
    const int height = static_cast<int>(frameBuffer.size());
    const int width = static_cast<int>(frameBuffer.front().size());
    if (width == 0) return;

    for(const auto& triangle: model.triangles())
    {

        std::vector<Eigen::Vector3f>screenTriangle;
        std::vector<float> inverseW;
        for(const auto& vertex: triangle.vertices)
        {
            float clipW;
            Eigen::Vector3f screenPos = Transform::transform(vertex.position, eye, target, up, clipW);
            if (!std::isfinite(clipW) || clipW <= 0.0f || !screenPos.allFinite()) break;
            screenTriangle.push_back(screenPos);
            inverseW.push_back(1.0f / clipW);
        }
        if (screenTriangle.size() != 3) continue;

        const float w_1 = inverseW[0];
        const float w_2 = inverseW[1];
        const float w_3 = inverseW[2];

        float min_x = std::max(0.0f, std::min({screenTriangle[0].x(),screenTriangle[1].x(),screenTriangle[2].x()}));
        float max_x = std::min(static_cast<float>(width), std::max({screenTriangle[0].x(),screenTriangle[1].x(),screenTriangle[2].x()}));
        float min_y = std::max(0.0f, std::min({screenTriangle[0].y(),screenTriangle[1].y(),screenTriangle[2].y()}));
        float max_y = std::min(static_cast<float>(height), std::max({screenTriangle[0].y(),screenTriangle[1].y(),screenTriangle[2].y()}));
        if (min_x > max_x || min_y > max_y) continue;

        for(int x = static_cast<int>(std::floor(min_x)); x <= std::min(width - 1, static_cast<int>(std::ceil(max_x))); ++x)
        {
            for(int y = static_cast<int>(std::floor(min_y)); y <= std::min(height - 1, static_cast<int>(std::ceil(max_y))); ++y)
            {
                float alpha, beta, gamma;
                if (!Transform::computeBarycentric2D(Eigen::Vector3f(x+0.5f,y+0.5f,0.0f),screenTriangle[0],screenTriangle[1],screenTriangle[2],alpha, beta, gamma)) continue;
                if(alpha < 0 || beta < 0 || gamma < 0) continue;
                float z_interpolated = alpha * screenTriangle[0].z() + beta * screenTriangle[1].z() + gamma * screenTriangle[2].z();
                if (!std::isfinite(z_interpolated) || z_interpolated < -1.0f || z_interpolated > 1.0f) continue;
                if(z_interpolated < zBuffer[y][x])
                {
                    float denom = alpha * w_1 + beta * w_2 + gamma * w_3;
                    if (!std::isfinite(denom) || denom <= 0.0f) continue;
                    //经过下面的变换之后alpha,beta,gamma就还原成了世界空间的插值权重了

                    alpha = alpha *w_1/denom;
                    beta = beta * w_2/denom;
                    gamma = gamma * w_3/denom;

                    Eigen::Vector3f normal = alpha * triangle.vertices[0].normal + beta * triangle.vertices[1].normal + gamma * triangle.vertices[2].normal;
                    Eigen::Vector3f worldPosition = alpha * triangle.vertices[0].position + beta * triangle.vertices[1].position + gamma * triangle.vertices[2].position;
                    if (!normal.allFinite() || normal.squaredNorm() <= 1e-12f || !worldPosition.allFinite()) continue;
                    normal.normalize();
                    Eigen::Vector3f color = shadeModel.computeShading(model.materialFor(triangle), eye, worldPosition, normal);
                    if (!color.allFinite()) continue;
                    zBuffer[y][x] = z_interpolated;
                    normalBuffer[y][x] = normal;
                    frameBuffer[y][x] = color;
                }

                /*
                if(x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT) continue;
                float alpha, beta, gamma;
                Transform::computeBarycentric2D(Eigen::Vector3f(x+0.5f,y+0.5f,0.0f),screenTriangle[0],screenTriangle[1],screenTriangle[2],alpha, beta, gamma);
                if(alpha < 0 || beta < 0 || gamma < 0) continue;
                float denom = alpha * w_1 + beta * w_2 + gamma * w_3;
                if (denom == 0.0f) continue;
                alpha = alpha *w_1/denom;
                beta = beta * w_2/denom;
                gamma = gamma * w_3/denom;
                Eigen::Vector3f world_interpolated_position = alpha * triangle.vertices[0].position + beta * triangle.vertices[1].position + gamma * triangle.vertices[2].position;
                if((world_interpolated_position-eye).norm() < zBuffer[y][x])
                {
                    zBuffer[y][x] = (world_interpolated_position-eye).norm();
                    normalBuffer[y][x] = (alpha * triangle.vertices[0].normal + beta * triangle.vertices[1].normal + gamma * triangle.vertices[2].normal).normalized();
                    frameBuffer[y][x] = shadeModel.computeShading(model.materialFor(triangle), eye, world_interpolated_position, normalBuffer[y][x]);
                }
                */
            }
        }

    }
}

void rasterizer::draw() {
    if (frameBuffer.empty()) return;

    // 第一次绘制：准备两个三角形、贴图和最简单的纹理着色器。
    if (!displayProgram) {
        auto compile = [](GLenum type, const char* filename) {
            const std::string path = std::string(RENDERER_SHADER_DIR) + "/" + filename;
            std::ifstream file(path);
            if (!file) throw std::runtime_error("Cannot open shader file: " + path);
            std::string source{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
            const char* code = source.c_str();
            GLuint shader = glCreateShader(type);
            glShaderSource(shader, 1, &code, nullptr);
            glCompileShader(shader);
            return shader;
        };
        GLuint vertexShader = compile(GL_VERTEX_SHADER, "display.vert");
        GLuint fragmentShader = compile(GL_FRAGMENT_SHADER, "display.frag");
        displayProgram = glCreateProgram();
        glAttachShader(displayProgram, vertexShader);
        glAttachShader(displayProgram, fragmentShader);
        glLinkProgram(displayProgram);
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        glUseProgram(displayProgram);
        glUniform1i(glGetUniformLocation(displayProgram, "screenTexture"), 0);
        glGenVertexArrays(1, &displayVAO);
        glGenTextures(1, &displayTexture);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, displayTexture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }

    // 每帧把二维颜色缓冲整理为连续的 RGB 数据，再上传纹理。
    displayPixels.clear();
    for (const auto& row : frameBuffer) {
        for (const auto& color : row) {
            displayPixels.insert(displayPixels.end(), color.data(), color.data() + 3);
        }
    }
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, displayTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, static_cast<int>(frameBuffer[0].size()),
                 static_cast<int>(frameBuffer.size()), 0, GL_RGB, GL_FLOAT, displayPixels.data());

    // 绘制覆盖窗口的矩形；着色器中的 uv 已处理上下翻转。
    int width, height;
    glfwGetFramebufferSize(glfwGetCurrentContext(), &width, &height);
    glViewport(0, 0, width, height);
    glDisable(GL_DEPTH_TEST);
    glUseProgram(displayProgram);
    glBindVertexArray(displayVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
}

void rasterizer::releaseDrawResources() {
    if (!displayProgram) return;
    glDeleteProgram(displayProgram);
    glDeleteTextures(1, &displayTexture);
    glDeleteVertexArrays(1, &displayVAO);
    displayProgram = displayTexture = displayVAO = 0;
}

void rasterizer::clear()
{
    for (auto& row : frameBuffer)
        std::fill(row.begin(), row.end(), backgroundColor);
    for (auto& row : zBuffer)
        std::fill(row.begin(), row.end(), std::numeric_limits<float>::infinity());
    for (auto& row : normalBuffer)
        std::fill(row.begin(), row.end(), Eigen::Vector3f::Zero());
    IntersectedTriangles.clear();
}

