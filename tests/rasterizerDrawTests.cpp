#include "rasterizer.h"
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

struct RasterizerDrawTestAccess {
    static void setFrame(rasterizer& renderer,
                         const std::vector<std::vector<Eigen::Vector3f>>& pixels) {
        renderer.frameBuffer = pixels;
    }
};

namespace {
void expectPixel(GLFWwindow* window, float x, float y, const std::array<int, 3>& expected) {
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    std::array<unsigned char, 4> pixel{};
    glReadPixels(static_cast<int>(x * width), height - 1 - static_cast<int>(y * height),
                 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
    for (int channel = 0; channel < 3; ++channel) {
        if (std::abs(static_cast<int>(pixel[channel]) - expected[channel]) > 1)
            throw std::runtime_error("Unexpected displayed pixel color or orientation.");
    }
}
} // namespace

int main() {
    if (!glfwInit()) {
        std::cout << "SKIP: GLFW initialization unavailable.\n";
        return 77;
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(64, 48, "rasterizer draw test", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        std::cout << "SKIP: OpenGL 3.3 core context unavailable.\n";
        return 77;
    }
    glfwMakeContextCurrent(window);
    if (!gladLoadGL(glfwGetProcAddress) || !GLAD_GL_VERSION_3_3) {
        glfwDestroyWindow(window);
        glfwTerminate();
        std::cout << "SKIP: OpenGL 3.3 functions unavailable.\n";
        return 77;
    }

    shadeModel shader;
    rasterizer renderer(shader);
    int result = 0;
    try {
        using Eigen::Vector3f;
        renderer.initRasterizer();
        renderer.draw();
        glReadBuffer(GL_BACK);
        expectPixel(window, .1f, .1f, {0, 0, 128});
        renderer.clear();
        renderer.draw();
        expectPixel(window, .9f, .9f, {0, 0, 128});
        RasterizerDrawTestAccess::setFrame(renderer, {
            {Vector3f(1, 0, 0), Vector3f(0, 1, 0)},
            {Vector3f(0, 0, 1), Vector3f(1, 1, 0)}
        });
        renderer.draw();
        glReadBuffer(GL_BACK);
        expectPixel(window, .25f, .25f, {255, 0, 0});
        expectPixel(window, .75f, .25f, {0, 255, 0});
        expectPixel(window, .25f, .75f, {0, 0, 255});
        expectPixel(window, .75f, .75f, {255, 255, 0});

        RasterizerDrawTestAccess::setFrame(renderer,
            std::vector<std::vector<Vector3f>>(2, std::vector<Vector3f>(2, Vector3f(.25f, .5f, .75f))));
        renderer.draw();
        expectPixel(window, .25f, .25f, {64, 128, 191});
        expectPixel(window, .75f, .75f, {64, 128, 191});

        glfwSetWindowSize(window, 83, 57);
        glfwPollEvents();
        RasterizerDrawTestAccess::setFrame(renderer,
            {{Vector3f(1, 1, 0), Vector3f(1, 1, 0), Vector3f(1, 1, 0)}});
        renderer.draw();
        expectPixel(window, .1f, .1f, {255, 255, 0});
        expectPixel(window, .9f, .9f, {255, 255, 0});

        renderer.releaseDrawResources();
        RasterizerDrawTestAccess::setFrame(renderer, {{Vector3f(1, 0, 1)}});
        renderer.draw();
        expectPixel(window, .5f, .5f, {255, 0, 255});
        if (glGetError() != GL_NO_ERROR) throw std::runtime_error("OpenGL reported an error.");
        std::cout << "PASS rasterizer draw: orientation, colors, updates, resize and recreation\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL rasterizer draw: " << error.what() << '\n';
        result = 1;
    }
    renderer.releaseDrawResources();
    glfwDestroyWindow(window);
    glfwTerminate();
    return result;
}
