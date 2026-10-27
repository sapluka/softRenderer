#include "rasterizer.h"

void rasterizer::initRasterizer() {
    frameBuffer.assign(SCREEN_HEIGHT,std::vector<Eigen::Vector3f>(SCREEN_WIDTH,backgroundColor));
    zBuffer.assign(SCREEN_HEIGHT,std::vector<float>(SCREEN_WIDTH,std::numeric_limits<float>::infinity()));
    subZBuffer.assign(SCREEN_HEIGHT,std::vector<std::array<float, 4>>(SCREEN_WIDTH));
    subFrameBuffer.assign(SCREEN_HEIGHT,std::vector<std::array<Eigen::Vector3f, 4>>(SCREEN_WIDTH));
    clearSubBuffers();
}



bool rasterizer::setGBuffers(const objLoader& model, const Eigen::Vector3f& origin,
                            const Eigen::Vector3f& target, const Eigen::Vector3f& up,
                            unsigned int threadCount,
                            const std::function<bool(int, int)>& progress) {


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

        float min_x = std::max(0.0f, std::min({screenTriangle[0].x(),screenTriangle[1].x(),screenTriangle[2].x()}));
        float max_x = std::min(static_cast<float>(width), std::max({screenTriangle[0].x(),screenTriangle[1].x(),screenTriangle[2].x()}));
        float min_y = std::max(0.0f, std::min({screenTriangle[0].y(),screenTriangle[1].y(),screenTriangle[2].y()}));
        float max_y = std::min(static_cast<float>(height), std::max({screenTriangle[0].y(),screenTriangle[1].y(),screenTriangle[2].y()}));
        if (min_x > max_x || min_y > max_y) continue;

        const auto& material = model.materialFor(triangle);
        for(int x = static_cast<int>(std::floor(min_x)); x <= std::min(width - 1, static_cast<int>(std::ceil(max_x))); ++x)
        {
            for(int y = static_cast<int>(std::floor(min_y)); y <= std::min(height - 1, static_cast<int>(std::ceil(max_y))); ++y)
            {
                rasterizePixel(triangle, material, screenTriangle, inverseW, eye, x, y);
            }
        }

    }
    resolveMSAA();
}

void rasterizer::rasterizePixel(const objLoader::Triangle& triangle, const objLoader::Material& material,
                              const std::vector<Eigen::Vector3f>& screenTriangle, const std::vector<float>& inverseW,
                              const Eigen::Vector3f& eye, int x, int y)
{
    std::array<bool, 4> visibleSamples{};
    bool hasCoveredSample = false;
    bool hasVisibleSample = false;
    Eigen::Vector3f coveredWeights = Eigen::Vector3f::Zero();
    for(int i = 0; i < 2; ++i)
    {
        for(int j = 0; j < 2; ++j)
        {
            const int sample = i * 2 + j;
            const Eigen::Vector3f point(x + (i + 0.5f) * 0.5f, y + (j + 0.5f) * 0.5f, 0.0f);
            float alpha, beta, gamma;
            if(!Transform::computeBarycentric2D(point, screenTriangle[0], screenTriangle[1], screenTriangle[2], alpha, beta, gamma)) continue;
            if(alpha < 0.0f || beta < 0.0f || gamma < 0.0f) continue;
            if(!hasCoveredSample)
            {
                coveredWeights = Eigen::Vector3f(alpha, beta, gamma);
                hasCoveredSample = true;
            }
            const float z = alpha * screenTriangle[0].z() + beta * screenTriangle[1].z() + gamma * screenTriangle[2].z();
            if(z >= subZBuffer[y][x][sample]) continue;
            subZBuffer[y][x][sample] = z;
            visibleSamples[sample] = true;
            hasVisibleSample = true;
        }
    }
    if(!hasVisibleSample) return;

    // 每个三角形对当前像素只着色一次，中心不在三角形内时使用首个被覆盖的采样点。
    float alpha, beta, gamma;
    const Eigen::Vector3f center(x + 0.5f, y + 0.5f, 0.0f);
    if(!Transform::computeBarycentric2D(center, screenTriangle[0], screenTriangle[1], screenTriangle[2], alpha, beta, gamma)
       || alpha < 0.0f || beta < 0.0f || gamma < 0.0f)
    {
        alpha = coveredWeights.x();
        beta = coveredWeights.y();
        gamma = coveredWeights.z();
    }
    const float denom = alpha * inverseW[0] + beta * inverseW[1] + gamma * inverseW[2];
    alpha = alpha * inverseW[0] / denom;
    beta = beta * inverseW[1] / denom;
    gamma = gamma * inverseW[2] / denom;
    const Eigen::Vector3f worldPos = alpha * triangle.vertices[0].position + beta * triangle.vertices[1].position + gamma * triangle.vertices[2].position;
    const Eigen::Vector3f normal = (alpha * triangle.vertices[0].normal + beta * triangle.vertices[1].normal + gamma * triangle.vertices[2].normal).normalized();
    const Eigen::Vector3f color = shadeModel.computeShading(material, eye, worldPos, normal);
    for(int sample = 0; sample < 4; ++sample)
    {
        if(visibleSamples[sample]) subFrameBuffer[y][x][sample] = color;
    }
}

void rasterizer::resolveMSAA()
{
    const int height = static_cast<int>(frameBuffer.size());
    const int width = static_cast<int>(frameBuffer.front().size());
    for(int y = 0; y < height; ++y)
    {
        for(int x = 0; x < width; ++x)
        {
            Eigen::Vector3f color_sum = Eigen::Vector3f::Zero();
            for(const auto& color : subFrameBuffer[y][x]) color_sum += color;
            frameBuffer[y][x] = color_sum / 4.0f;
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
    clearSubBuffers();
}

void rasterizer::clearSubBuffers()
{
    for(auto& row : subZBuffer)
        for(auto& samples : row) samples.fill(std::numeric_limits<float>::infinity());
    for(auto& row : subFrameBuffer)
        for(auto& samples : row) samples.fill(backgroundColor);
}

