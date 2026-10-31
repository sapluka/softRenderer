#include "rasterizer.h"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

struct RasterizerShadowTestAccess {
    static float visibility(const rasterizer& renderer, const Eigen::Vector3f& point) {
        return renderer.getShadowVisibility(point);
    }
    static const std::vector<std::vector<float>>& lightDepths(const rasterizer& renderer) {
        return renderer.zBufferLight;
    }
    static void resizeLight(rasterizer& renderer, int width, int height) {
        renderer.zBufferLight.assign(height,
            std::vector<float>(width, std::numeric_limits<float>::infinity()));
        const light& source = renderer.shadeModel.getLight();
        renderer.lightTransform = Transform(source.getPosition(), source.getTarget(), source.getUp(), width, height);
    }
    static void setLightDepth(rasterizer& renderer, int x, int y, float depth) {
        renderer.zBufferLight[y][x] = depth;
    }
    static void fillLightDepth(rasterizer& renderer, float depth) {
        for (auto& row : renderer.zBufferLight) std::fill(row.begin(), row.end(), depth);
    }
    static void fillCameraDepth(rasterizer& renderer, float depth) {
        for (auto& row : renderer.zBuffer) std::fill(row.begin(), row.end(), depth);
    }
    static float cameraDepth(const rasterizer& renderer, int x, int y) {
        return renderer.zBuffer[y][x];
    }
    static const Eigen::Vector3f& pixel(const rasterizer& renderer, int x, int y) {
        return renderer.frameBuffer[y][x];
    }
};

namespace {
using Eigen::Vector3f;
namespace fs = std::filesystem;
const Vector3f receiverPoint(0.0f, 0.0f, -4.0f);
const Vector3f litPoint(1.5f, 0.0f, -4.0f);
const Vector3f cameraPosition(3.0f, 0.0f, -1.0f);

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void requireNear(const Vector3f& actual, const Vector3f& expected, const std::string& message) {
    require(actual.allFinite() && (actual - expected).norm() < 2e-5f, message);
}

struct Face {
    std::array<Vector3f, 3> points;
    std::string material = "surface";
};

Face blocker(float distance) {
    return {{{Vector3f(-.5f, -.5f, -distance), Vector3f(.5f, -.5f, -distance),
              Vector3f(0.0f, .5f, -distance)}}};
}

class Fixtures {
public:
    Fixtures() {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        directory = fs::temp_directory_path() / ("renderer_shadow_tests_" + std::to_string(stamp));
        require(fs::create_directory(directory), "Cannot create shadow test fixtures.");
        write("materials.mtl", "newmtl surface\nKa .1 .1 .1\nKd .6 .4 .2\nKs 0 0 0\n");
    }

    ~Fixtures() {
        std::error_code ignored;
        for (const auto& file : files) fs::remove(file, ignored);
        fs::remove(directory, ignored);
    }

    objLoader model(const std::string& name, const std::vector<Face>& faces) {
        std::ostringstream text;
        text << std::setprecision(std::numeric_limits<float>::max_digits10);
        text << "mtllib materials.mtl\nvn 0 0 1\n";
        std::size_t index = 1;
        for (const auto& face : faces) {
            for (const auto& point : face.points)
                text << "v " << point.x() << ' ' << point.y() << ' ' << point.z() << '\n';
            text << "usemtl " << face.material << "\nf " << index << "//1 "
                 << index + 1 << "//1 " << index + 2 << "//1\n";
            index += 3;
        }
        const fs::path path = write(name, text.str());
        objLoader result;
        require(result.loadObj(path), "Cannot load shadow fixture: " + result.error());
        return result;
    }

private:
    fs::path directory;
    std::vector<fs::path> files;

    fs::path write(const std::string& name, const std::string& contents) {
        const fs::path path = directory / name;
        std::ofstream stream(path);
        stream << contents;
        require(static_cast<bool>(stream), "Cannot write shadow fixture.");
        files.push_back(path);
        return path;
    }
};

light baseLight() {
    light source(Vector3f::Zero(), Vector3f::Ones(), 1.0f);
    source.setTarget(receiverPoint);
    source.setUp(Vector3f::UnitY());
    return source;
}

void buildMap(rasterizer& renderer, const objLoader& model, const light& source) {
    renderer.setGBuffers(model, source.getPosition(), source.getTarget(), source.getUp());
}

float centerDepth(const rasterizer& renderer) {
    const auto& depths = RasterizerShadowTestAccess::lightDepths(renderer);
    return depths[depths.size() / 2][depths.front().size() / 2];
}

void requireVisibility(const rasterizer& renderer, const Vector3f& point, float expected,
                       const std::string& message) {
    require(RasterizerShadowTestAccess::visibility(renderer, point) == expected, message);
}
} // namespace

int main() {
    Fixtures files;
    const light source = baseLight();
    shadeModel shader(source);
    rasterizer renderer(shader);
    renderer.initRasterizer();
    const objLoader occluder = files.model("occluder.obj", {blocker(2.0f)});
    unsigned int failed = 0;
    const auto run = [&](const std::string& name, const std::function<void()>& test) {
        try {
            test();
            std::cout << "PASS " << name << '\n';
        } catch (const std::exception& error) {
            ++failed;
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        }
    };

    run("shadow viewport covers its larger buffer independently of camera dimensions", [&] {
        const auto& depths = RasterizerShadowTestAccess::lightDepths(renderer);
        require(depths.size() == SHADOW_MAP_HEIGHT && depths.front().size() == SHADOW_MAP_WIDTH,
                "The shadow map did not allocate its independent dimensions.");
        require(SHADOW_MAP_WIDTH == SCREEN_WIDTH * 2 && SHADOW_MAP_HEIGHT == SCREEN_HEIGHT * 2,
                "The configured shadow map does not have four times as many texels.");
        const Eigen::Matrix4f viewport = Transform::getViewportMatrix(SHADOW_MAP_WIDTH, SHADOW_MAP_HEIGHT);
        requireNear((viewport * Eigen::Vector4f(-1, 1, 0, 1)).head<3>(), Vector3f::Zero(),
                    "The shadow viewport did not map the upper-left NDC corner to the origin.");
        requireNear((viewport * Eigen::Vector4f(1, -1, 0, 1)).head<3>(),
                    Vector3f(SHADOW_MAP_WIDTH, SHADOW_MAP_HEIGHT, 0),
                    "The shadow viewport retained the camera dimensions.");

        Face shifted = blocker(2.0f);
        for (auto& point : shifted.points) point += Vector3f(1.0f, -.6f, 0.0f);
        const objLoader shiftedModel = files.model("large_viewport.obj", {shifted});
        buildMap(renderer, shiftedModel, source);
        float clipW;
        const Vector3f screen = Transform::transform(Vector3f(1, -.6f, -2), source.getPosition(),
            source.getTarget(), source.getUp(), clipW, z_near, z_far, SHADOW_MAP_WIDTH, SHADOW_MAP_HEIGHT);
        const int x = static_cast<int>(screen.x());
        const int y = static_cast<int>(screen.y());
        require(x >= SCREEN_WIDTH && x < SHADOW_MAP_WIDTH && y >= SCREEN_HEIGHT && y < SHADOW_MAP_HEIGHT,
                "The fixture does not cover the newly available shadow region.");
        require(std::isfinite(depths[y][x]), "Rasterization stopped at the old camera bounds.");
        requireVisibility(renderer, Vector3f(2, -1.2f, -4), 0.0f,
                          "Shadow queries did not use the same larger viewport as map generation.");
    });

    run("light depth keeps the nearest triangle independently of camera depth and draw order", [&] {
        shader.setLight(source);
        const objLoader nearFirst = files.model("near_first.obj", {blocker(1.0f), blocker(3.0f)});
        const objLoader farFirst = files.model("far_first.obj", {blocker(3.0f), blocker(1.0f)});
        RasterizerShadowTestAccess::fillCameraDepth(renderer, -1.0f);
        buildMap(renderer, nearFirst, source);
        const float first = centerDepth(renderer);
        require(std::isfinite(first), "The light pass incorrectly tested against camera depth.");
        const float expected = (z_near + z_far) / (z_far - z_near)
                             + 2.0f * z_near * z_far / (z_far - z_near);
        require(std::abs(first - expected) < 2e-5f, "The light map did not retain the near surface.");
        require(RasterizerShadowTestAccess::cameraDepth(renderer, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2) == -1.0f,
                "Building the light map modified camera depth.");
        buildMap(renderer, farFirst, source);
        require(std::abs(centerDepth(renderer) - first) < 2e-5f, "Triangle order changed the light depth.");
    });

    run("world points query light-space occlusion and do not shadow their own surface", [&] {
        buildMap(renderer, occluder, source);
        requireVisibility(renderer, receiverPoint, 0.0f, "A point behind the blocker was lit.");
        requireVisibility(renderer, litPoint, 1.0f, "An unobstructed point was shadowed.");
        requireVisibility(renderer, Vector3f(0, 0, -2), 1.0f, "The depth map shadowed its own surface.");
        requireVisibility(renderer, Vector3f(0, 0, 1), 1.0f, "A point behind the light queried the map.");
        requireVisibility(renderer, Vector3f(100, 0, -2), 1.0f, "A point outside the map queried unrelated depth.");
    });

    run("rebuilding the map removes old occluders", [&] {
        buildMap(renderer, occluder, source);
        requireVisibility(renderer, receiverPoint, 0.0f, "The initial blocker was not recorded.");
        objLoader empty;
        buildMap(renderer, empty, source);
        require(centerDepth(renderer) == std::numeric_limits<float>::infinity(), "Old light depth survived rebuilding.");
        requireVisibility(renderer, receiverPoint, 1.0f, "A removed blocker still cast a shadow.");
    });

    run("moving the camera preserves the light map and a fixed world's visibility", [&] {
        shader.setLight(source);
        renderer.clear();
        renderer.rasterize(occluder, cameraPosition, receiverPoint);
        const auto firstMap = RasterizerShadowTestAccess::lightDepths(renderer);
        requireVisibility(renderer, receiverPoint, 0.0f, "The first camera pose lost the blocker.");
        renderer.clear();
        renderer.rasterize(occluder, Vector3f(-3, 1, -1), receiverPoint);
        require(RasterizerShadowTestAccess::lightDepths(renderer) == firstMap,
                "Camera motion changed the light's depth map.");
        requireVisibility(renderer, receiverPoint, 0.0f, "Camera motion changed a world's shadow.");
    });

    run("light position target and up stay consistent between map generation and queries", [&] {
        light changed = source;
        changed.setTarget(Vector3f(.6f, 0, -4));
        changed.setUp(Vector3f::UnitX());
        shader.setLight(changed);
        renderer.clear();
        renderer.rasterize(occluder, cameraPosition, receiverPoint);
        requireVisibility(renderer, receiverPoint, 0.0f, "Changing light orientation broke the world-to-map query.");
        requireVisibility(renderer, litPoint, 1.0f, "Changing light up shadowed a clear light path.");

        changed.setPosition(Vector3f(3, 0, 0));
        changed.setTarget(receiverPoint);
        changed.setUp(Vector3f::UnitY());
        shader.setLight(changed);
        renderer.clear();
        renderer.rasterize(occluder, cameraPosition, receiverPoint);
        requireVisibility(renderer, receiverPoint, 1.0f, "Moving the light retained the previous shadow.");
        shader.setLight(source);
    });

    run("rendering applies shadow visibility to direct light while preserving ambient light", [&] {
        const objLoader scene = files.model("receiver.obj", {
            blocker(2.0f),
            {{{Vector3f(-2, -2, -4), Vector3f(2, -2, -4), Vector3f(2, 2, -4)}}},
            {{{Vector3f(-2, -2, -4), Vector3f(2, 2, -4), Vector3f(-2, 2, -4)}}}
        });
        shader.setLight(source);
        renderer.clear();
        renderer.rasterize(scene, cameraPosition, receiverPoint);
        requireNear(RasterizerShadowTestAccess::pixel(renderer, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2),
                    Vector3f::Constant(.1f), "The receiver's shadow did not retain only ambient light.");
        const Vector3f screen = Transform::transform(litPoint, cameraPosition, receiverPoint);
        const Vector3f color = RasterizerShadowTestAccess::pixel(renderer, static_cast<int>(screen.x()), static_cast<int>(screen.y()));
        require(color.x() > .6f && color.y() > .4f && color.z() > .2f,
                "An unobstructed receiver lost direct illumination.");
    });

    run("PCF compares all nine depths before averaging visibility for one receiver", [&] {
        shader.setLight(source);
        const float receiverDepth = Transform::transform(receiverPoint, source.getPosition(),
            source.getTarget(), source.getUp()).z();
        for (int blocked = 0; blocked <= 9; ++blocked) {
            RasterizerShadowTestAccess::resizeLight(renderer, 7, 5);
            for (int sample = 0; sample < blocked; ++sample)
                RasterizerShadowTestAccess::setLightDepth(renderer,
                    3 + sample % 3 - 1, 2 + sample / 3 - 1, receiverDepth - .01f);
            const float visibility = RasterizerShadowTestAccess::visibility(renderer, receiverPoint);
            require(std::abs(visibility - (9 - blocked) / 9.0f) < 1e-6f,
                    "PCF did not average the nine independent depth comparisons.");
        }
    });

    run("PCF treats missing border taps as unoccluded without reading outside the map", [&] {
        RasterizerShadowTestAccess::resizeLight(renderer, 7, 5);
        RasterizerShadowTestAccess::fillLightDepth(renderer, -1.0f);
        const Eigen::Matrix4f projection = Transform::getProjectionMatrix();
        for (float x : {.5f, 6.5f}) {
            for (float y : {.5f, 4.5f}) {
                const Vector3f point((2.0f * x / 7.0f - 1.0f) * 4.0f / projection(0, 0),
                                    (1.0f - 2.0f * y / 5.0f) * 4.0f / projection(1, 1), -4.0f);
                require(std::abs(RasterizerShadowTestAccess::visibility(renderer, point) - 5.0f / 9.0f) < 1e-6f,
                        "PCF border taps did not retain the fixed nine-sample weighting.");
            }
        }
        requireVisibility(renderer, Vector3f(100, 0, -2), 1.0f,
                          "A receiver outside the shadow map lost the unoccluded fallback.");
    });

    return failed == 0 ? 0 : 1;
}
