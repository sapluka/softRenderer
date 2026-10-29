#include "rasterizer.h"

void rasterizer::initRasterizer() {
    frameBuffer.assign(SCREEN_HEIGHT,std::vector<Eigen::Vector3f>(SCREEN_WIDTH,backgroundColor));
    zBuffer.assign(SCREEN_HEIGHT,std::vector<float>(SCREEN_WIDTH,std::numeric_limits<float>::infinity()));
    zBufferLight.assign(SCREEN_HEIGHT,std::vector<float>(SCREEN_WIDTH,std::numeric_limits<float>::infinity()));
    subZBuffer.assign(SCREEN_HEIGHT,std::vector<std::array<float, 4>>(SCREEN_WIDTH));
    subFrameBuffer.assign(SCREEN_HEIGHT,std::vector<std::array<Eigen::Vector3f, 4>>(SCREEN_WIDTH));
    clearSubBuffers();
}



void rasterizer::setGBuffers(const objLoader& model, const Eigen::Vector3f& origin,
                            const Eigen::Vector3f& target, const Eigen::Vector3f& up) {
    if (frameBuffer.empty()) initRasterizer();
    const int width = static_cast<int>(frameBuffer.front().size());
    if (width == 0) return;
    clearDepthBuffer(zBufferLight);

    for(const auto& triangle: model.triangles())
    {

        std::vector<Eigen::Vector3f> screenTriangle;
        std::vector<float> inverseW;
        std::array<int, 4> bounds;
        if(!prepareTriangle(triangle, origin, target, up, screenTriangle, inverseW, bounds)) continue;
        for(int x = bounds[0]; x <= bounds[1]; ++x)
        {
            for(int y = bounds[2]; y <= bounds[3]; ++y)
            {
                updateDepthBuffer(screenTriangle, zBufferLight, x, y);
            }
        }

    }
}


void rasterizer::rasterize(const objLoader& model, const Eigen::Vector3f& origin,const Eigen::Vector3f& target,
                           const Eigen::Vector3f& up, float nearPlane, float farPlane) {

    if (frameBuffer.empty()) initRasterizer();
    const int width = static_cast<int>(frameBuffer.front().size());
    if (width == 0) return;
    const light& sceneLight = shadeModel.getLight();
    setGBuffers(model, sceneLight.getPosition(), sceneLight.getTarget(), sceneLight.getUp());

    for(const auto& triangle: model.triangles())
    {

        std::vector<Eigen::Vector3f> screenTriangle;
        std::vector<float> inverseW;
        std::array<int, 4> bounds;
        if(!prepareTriangle(triangle, origin, target, up, screenTriangle, inverseW, bounds, nearPlane, farPlane)) continue;

        const auto& material = model.materialFor(triangle);
        for(int x = bounds[0]; x <= bounds[1]; ++x)
        {
            for(int y = bounds[2]; y <= bounds[3]; ++y)
            {
                updateDepthBuffer(screenTriangle, zBuffer, x, y);
                rasterizePixel(triangle, material, screenTriangle, inverseW, origin, x, y);
            }
        }

    }
    resolveMSAA();
}

bool rasterizer::prepareTriangle(const objLoader::Triangle& triangle, const Eigen::Vector3f& origin,
                                 const Eigen::Vector3f& target, const Eigen::Vector3f& up,
                                 std::vector<Eigen::Vector3f>& screenTriangle, std::vector<float>& inverseW,
                                 std::array<int, 4>& bounds, float nearPlane, float farPlane)
{
    for(const auto& vertex : triangle.vertices)
    {
        float clipW;
        const Eigen::Vector3f screenPos = Transform::transform(vertex.position, origin, target, up, clipW, nearPlane, farPlane);
        if(!std::isfinite(clipW) || clipW <= 0.0f || !screenPos.allFinite()) return false;
        screenTriangle.push_back(screenPos);
        inverseW.push_back(1.0f / clipW);
    }
    const int height = static_cast<int>(frameBuffer.size());
    const int width = static_cast<int>(frameBuffer.front().size());
    const float min_x = std::max(0.0f, std::min({screenTriangle[0].x(), screenTriangle[1].x(), screenTriangle[2].x()}));
    const float max_x = std::min(static_cast<float>(width), std::max({screenTriangle[0].x(), screenTriangle[1].x(), screenTriangle[2].x()}));
    const float min_y = std::max(0.0f, std::min({screenTriangle[0].y(), screenTriangle[1].y(), screenTriangle[2].y()}));
    const float max_y = std::min(static_cast<float>(height), std::max({screenTriangle[0].y(), screenTriangle[1].y(), screenTriangle[2].y()}));
    if(min_x > max_x || min_y > max_y) return false;
    bounds = {static_cast<int>(std::floor(min_x)), std::min(width - 1, static_cast<int>(std::ceil(max_x))),
              static_cast<int>(std::floor(min_y)), std::min(height - 1, static_cast<int>(std::ceil(max_y)))};
    return true;
}

void rasterizer::updateDepthBuffer(const std::vector<Eigen::Vector3f>& screenTriangle,
                                   std::vector<std::vector<float>>& buffer, int x, int y)
{
    float alpha, beta, gamma;
    const Eigen::Vector3f center(x + 0.5f, y + 0.5f, 0.0f);
    if(!Transform::computeBarycentric2D(center, screenTriangle[0], screenTriangle[1], screenTriangle[2], alpha, beta, gamma)) return;
    if(alpha < 0.0f || beta < 0.0f || gamma < 0.0f) return;
    const float z = alpha * screenTriangle[0].z() + beta * screenTriangle[1].z() + gamma * screenTriangle[2].z();
    if(!std::isfinite(z) || z < -1.0f || z > 1.0f) return;
    if(z < buffer[y][x]) buffer[y][x] = z;
}

float rasterizer::getShadowVisibility(const Eigen::Vector3f& worldPos) const
{
    const light& sceneLight = shadeModel.getLight();
    float clipW;
    const Eigen::Vector3f lightPos = Transform::transform(worldPos, sceneLight.getPosition(),
                                                        sceneLight.getTarget(), sceneLight.getUp(), clipW);
    const int height = static_cast<int>(zBufferLight.size());
    const int width = static_cast<int>(zBufferLight.front().size());
    // 这张二维深度图只覆盖光源视锥内的区域。
    if(clipW <= 0.0f || lightPos.x() < 0.0f || lightPos.x() >= width || lightPos.y() < 0.0f || lightPos.y() >= height
       || lightPos.z() < -1.0f || lightPos.z() > 1.0f) return 1.0f;
    const int x = static_cast<int>(lightPos.x());
    const int y = static_cast<int>(lightPos.y());
    constexpr float shadowBias = 1e-4f;
    return lightPos.z() > zBufferLight[y][x] + shadowBias ? 0.0f : 1.0f;
}

void rasterizer::rasterizePixel(const objLoader::Triangle& triangle, const objLoader::Material& material,
                              const std::vector<Eigen::Vector3f>& screenTriangle, const std::vector<float>& inverseW,
                              const Eigen::Vector3f& origin, int x, int y)
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
            if(z < -1.0f || z > 1.0f) continue;
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
    const float visibility = getShadowVisibility(worldPos);
    const Eigen::Vector3f normal = (alpha * triangle.vertices[0].normal + beta * triangle.vertices[1].normal + gamma * triangle.vertices[2].normal).normalized();
    const Eigen::Vector3f color = shadeModel.computeShading(material, origin, worldPos, normal, visibility);
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
    clearDepthBuffer(zBuffer);
    clearDepthBuffer(zBufferLight);
    clearSubBuffers();
}

void rasterizer::clearDepthBuffer(std::vector<std::vector<float>>& buffer)
{
    for(auto& row : buffer)
        std::fill(row.begin(), row.end(), std::numeric_limits<float>::infinity());
}

void rasterizer::clearSubBuffers()
{
    for(auto& row : subZBuffer)
        for(auto& samples : row) samples.fill(std::numeric_limits<float>::infinity());
    for(auto& row : subFrameBuffer)
        for(auto& samples : row) samples.fill(backgroundColor);
}

