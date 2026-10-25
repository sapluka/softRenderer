#include "rasterizer.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

struct RasterizerTraceTestAccess {
    using VectorBuffer = std::vector<std::vector<Eigen::Vector3f>>;
    using DepthBuffer = std::vector<std::vector<float>>;

    struct Snapshot {
        VectorBuffer colors;
        DepthBuffer depths;
        VectorBuffer normals;
        std::vector<objLoader::Triangle> triangles;
    };

    static void configure(rasterizer& renderer, int width, int height) {
        renderer.frameBuffer.assign(height,
            std::vector<Eigen::Vector3f>(width, Eigen::Vector3f::Zero()));
        renderer.zBuffer.assign(height,
            std::vector<float>(width, std::numeric_limits<float>::infinity()));
        renderer.normalBuffer.assign(height,
            std::vector<Eigen::Vector3f>(width, Eigen::Vector3f::Zero()));
        renderer.IntersectedTriangles.clear();
    }

    static void poison(rasterizer& renderer) {
        for (auto& row : renderer.frameBuffer)
            std::fill(row.begin(), row.end(), Eigen::Vector3f(9.0f, 8.0f, 7.0f));
        for (auto& row : renderer.zBuffer)
            std::fill(row.begin(), row.end(), 10000.0f);
        for (auto& row : renderer.normalBuffer)
            std::fill(row.begin(), row.end(), Eigen::Vector3f(1.0f, 2.0f, 3.0f));
        renderer.IntersectedTriangles.assign(1, objLoader::Triangle{});
    }

    static Snapshot capture(const rasterizer& renderer) {
        return {renderer.frameBuffer, renderer.zBuffer, renderer.normalBuffer,
                renderer.IntersectedTriangles};
    }
};

namespace {
namespace fs = std::filesystem;
using Eigen::Vector3f;
using Snapshot = RasterizerTraceTestAccess::Snapshot;
const Vector3f backgroundColor(0.0f, 0.0f, 0.5f);
constexpr int testWidth = 13;
constexpr int testHeight = 9;
constexpr float tolerance = 2e-5f;
constexpr float emptyDepth = std::numeric_limits<float>::infinity();

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void requireNear(const Vector3f& actual, const Vector3f& expected,
                 const std::string& message) {
    require(actual.allFinite() && expected.allFinite()
            && (actual - expected).norm() <= tolerance, message);
}

void requireDepth(float actual, float expected, const std::string& message) {
    if (expected == emptyDepth) {
        require(actual == emptyDepth, message);
    } else {
        require(std::isfinite(actual) && std::isfinite(expected)
                && std::abs(actual - expected) <= tolerance * (1.0f + std::abs(expected)),
                message);
    }
}

void requireDimensions(const Snapshot& buffers, int width, int height) {
    require(buffers.colors.size() == static_cast<std::size_t>(height)
            && buffers.depths.size() == static_cast<std::size_t>(height)
            && buffers.normals.size() == static_cast<std::size_t>(height),
            "Buffer height changed.");
    for (int row = 0; row < height; ++row) {
        require(buffers.colors[row].size() == static_cast<std::size_t>(width)
                && buffers.depths[row].size() == static_cast<std::size_t>(width)
                && buffers.normals[row].size() == static_cast<std::size_t>(width),
                "Buffer width changed.");
    }
}

bool sameTriangle(const objLoader::Triangle& actual, const objLoader::Triangle& expected) {
    if (actual.materialIndex != expected.materialIndex
        || actual.objectIndex != expected.objectIndex) return false;
    for (std::size_t corner = 0; corner < actual.vertices.size(); ++corner) {
        if ((actual.vertices[corner].position - expected.vertices[corner].position).squaredNorm() != 0.0f
            || (actual.vertices[corner].normal - expected.vertices[corner].normal).squaredNorm() != 0.0f)
            return false;
    }
    return true;
}

void requireTriangles(const std::vector<objLoader::Triangle>& actual,
                      const std::vector<objLoader::Triangle>& expected) {
    require(actual.size() == expected.size(), "Wrong number of unique intersected triangles.");
    for (const auto& triangle : expected) {
        require(std::count_if(actual.begin(), actual.end(), [&](const auto& candidate) {
            return sameTriangle(candidate, triangle);
        }) == 1, "An intersected triangle is missing, duplicated or corrupted.");
    }
}

void requireBuffers(const Snapshot& actual, const Snapshot& expected, bool compareTriangleOrder = true) {
    const int height = static_cast<int>(expected.colors.size());
    const int width = height == 0 ? 0 : static_cast<int>(expected.colors.front().size());
    requireDimensions(actual, width, height);
    requireDimensions(expected, width, height);
    for (int row = 0; row < height; ++row) {
        for (int column = 0; column < width; ++column) {
            const std::string pixel = " at pixel (" + std::to_string(column)
                                      + ", " + std::to_string(row) + ")";
            requireNear(actual.colors[row][column], expected.colors[row][column],
                        "Color differs" + pixel);
            requireDepth(actual.depths[row][column], expected.depths[row][column],
                         "Depth or hit mask differs" + pixel);
            requireNear(actual.normals[row][column], expected.normals[row][column],
                        "Normal differs" + pixel);
        }
    }
    requireTriangles(actual.triangles, expected.triangles);
    if (compareTriangleOrder) {
        for (std::size_t index = 0; index < expected.triangles.size(); ++index) {
            require(sameTriangle(actual.triangles[index], expected.triangles[index]),
                    "Intersected triangles did not retain serial first-hit order.");
        }
    }
}

void requireEmpty(const Snapshot& buffers, int width, int height) {
    requireDimensions(buffers, width, height);
    require(buffers.triangles.empty(), "Old intersected triangles survived clearing.");
    for (int row = 0; row < height; ++row) {
        for (int column = 0; column < width; ++column) {
            requireNear(buffers.colors[row][column], backgroundColor, "Color was not cleared to the blue background.");
            requireDepth(buffers.depths[row][column], emptyDepth, "Depth was not cleared to +infinity.");
            requireNear(buffers.normals[row][column], Vector3f::Zero(), "Normal was not cleared.");
        }
    }
}

class FixtureFiles {
public:
    FixtureFiles() {
        const auto timestamp = std::chrono::steady_clock::now().time_since_epoch().count();
        for (unsigned int attempt = 0; attempt < 100; ++attempt) {
            root_ = fs::temp_directory_path() / ("renderer_trace_tests_"
                + std::to_string(timestamp) + "_" + std::to_string(attempt));
            if (fs::create_directory(root_)) return;
        }
        throw std::runtime_error("Cannot create a unique trace test fixture directory.");
    }

    FixtureFiles(const FixtureFiles&) = delete;
    FixtureFiles& operator=(const FixtureFiles&) = delete;

    ~FixtureFiles() {
        std::error_code ignored;
        fs::remove_all(root_, ignored);
    }

    fs::path write(const std::string& filename, const std::string& text) const {
        const fs::path path = root_ / filename;
        std::ofstream stream(path, std::ios::binary);
        stream << text;
        stream.close();
        require(static_cast<bool>(stream), "Cannot write trace test fixture: " + path.u8string());
        return path;
    }

private:
    fs::path root_;
};

objLoader loadModel(const fs::path& path) {
    objLoader model;
    require(model.loadObj(path), "Cannot load trace fixture: " + path.u8string() + ": " + model.error());
    return model;
}

struct RasterizerFixture {
    shadeModel shader;
    rasterizer renderer;

    RasterizerFixture(int width = testWidth, int height = testHeight,
                      const light& sceneLight = light())
        : shader(sceneLight), renderer(shader) {
        RasterizerTraceTestAccess::configure(renderer, width, height);
    }
};

struct Pose {
    Vector3f eye;
    Vector3f target;
    Vector3f up;
};

Snapshot trace(rasterizer& renderer, const objLoader& model, const Pose& pose,
               unsigned int workers) {
    require(renderer.setGBuffers(model, pose.eye, pose.target, pose.up, workers),
            "A trace without cancellation returned false.");
    return RasterizerTraceTestAccess::capture(renderer);
}

void requireCenter(const Snapshot& buffers, float depth, const Vector3f& color,
                   const Vector3f& normal) {
    const std::size_t row = buffers.colors.size() / 2;
    const std::size_t column = buffers.colors[row].size() / 2;
    requireDepth(buffers.depths[row][column], depth, "Center did not retain the nearest surface's world Z.");
    requireNear(buffers.colors[row][column], color, "Center has the wrong material or shading.");
    requireNear(buffers.normals[row][column], normal, "Center has the wrong surface normal.");
}

struct ProgressLog {
    std::thread::id caller = std::this_thread::get_id();
    std::mutex mutex;
    std::vector<std::pair<int, int>> reports;
    bool wrongThread = false;

    void record(int completed, int total) {
        std::lock_guard<std::mutex> guard(mutex);
        wrongThread = wrongThread || std::this_thread::get_id() != caller;
        reports.emplace_back(completed, total);
    }

    void check(int height, bool completed) {
        std::lock_guard<std::mutex> guard(mutex);
        require(!wrongThread, "Progress callback ran outside the calling thread.");
        require(!reports.empty(), "Progress callback was never invoked.");
        require(reports.front().first == 0, "Progress did not start with zero completed rows.");
        int previous = 0;
        for (const auto& report : reports) {
            require(report.second == height, "Progress reported the wrong total row count.");
            require(report.first >= previous && report.first <= height,
                    "Completed row count is out of range or moved backwards.");
            previous = report.first;
        }
        if (completed) require(previous == height, "Successful trace did not report all rows complete.");
    }
};

int runTests() {
    FixtureFiles files;
    files.write("trace.mtl",
        "newmtl red\nKa .7 .1 .2\nKd 0 0 0\nKs 0 0 0\nNs 1\n"
        "newmtl blue\nKa .1 .3 .8\nKd 0 0 0\nKs 0 0 0\nNs 1\n"
        "newmtl shaded\nKa .05 .1 .15\nKd .2 .4 .6\nKs 0 0 0\nNs 1\n");
    const std::string layers =
        "mtllib trace.mtl\n"
        "v -20 -20 1\nv 20 -20 1\nv 0 20 1\n"
        "v -20 -20 -1\nv 20 -20 -1\nv 0 20 -1\n"
        "vn 0 0 1\nvn 0 0 -1\n";
    const std::array<fs::path, 2> layerPaths = {
        files.write("layers.obj", layers + "usemtl blue\nf 4//2 5//2 6//2\nusemtl red\nf 1//1 2//1 3//1\n"),
        files.write("layers_reversed.obj", layers + "usemtl red\nf 1//1 2//1 3//1\nusemtl blue\nf 4//2 5//2 6//2\n")
    };
    const fs::path partialPath = files.write("partial.obj",
        "mtllib trace.mtl\nv -1 -1 0\nv 1 -1 0\nv 0 1 0\n"
        "vn 0 0 1\nvn 0 .6 .8\nvn .6 0 .8\nusemtl shaded\nf 1//1 2//2 3//3\n");
    const fs::path orderedPath = files.write("ordered.obj",
        "mtllib trace.mtl\nv -2.5 -1 0\nv -.5 -1 0\nv -1.5 1 0\n"
        "v .5 -1 0\nv 2.5 -1 0\nv 1.5 1 0\nvn 0 0 1\n"
        "usemtl blue\nf 4//1 5//1 6//1\nusemtl red\nf 1//1 2//1 3//1\n");
    const fs::path upperRightPath = files.write("upper_right.obj",
        "mtllib trace.mtl\nv .2 .2 0\nv 2.5 .2 0\nv .2 2.5 0\n"
        "vn 0 0 1\nusemtl red\nf 1//1 2//1 3//1\n");
    const fs::path tiltedPath = files.write("tilted.obj",
        "mtllib trace.mtl\nv -1.5 -1 1\nv 1.5 -1 -1\nv 0 1.5 0\n"
        "vn 0 0 1\nvn 0 .6 .8\nvn .6 0 .8\nusemtl shaded\nf 1//1 2//2 3//3\n");
    const fs::path edgeOnPath = files.write("edge_on.obj",
        "v 0 -1 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\n");
    const objLoader layered = loadModel(layerPaths[0]);
    const objLoader reversed = loadModel(layerPaths[1]);
    const objLoader partial = loadModel(partialPath);
    const Pose front{Vector3f(0, 0, 5), Vector3f::Zero(), Vector3f::UnitY()};
    const Pose back{Vector3f(0, 0, -5), Vector3f::Zero(), Vector3f::UnitY()};
    const Vector3f red(.7f, .1f, .2f);
    const Vector3f blue(.1f, .3f, .8f);
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

    run("screen barycentrics ignore Z while world barycentrics remain three-dimensional", [&] {
        float alpha, beta, gamma;
        require(Transform::computeBarycentric2D(Vector3f(1, 1, 99),
            Vector3f(0, 0, .2f), Vector3f(2, 0, .8f), Vector3f(0, 2, -.5f), alpha, beta, gamma),
            "Valid screen triangle was rejected.");
        requireNear(Vector3f(alpha, beta, gamma), Vector3f(0, .5f, .5f),
                    "Screen barycentrics included depth in their calculation.");
        require(!Transform::computeBarycentric2D(Vector3f(1, 1, 0),
            Vector3f(0, 0, 0), Vector3f(1, 1, 2), Vector3f(2, 2, 5), alpha, beta, gamma),
            "Screen-space degenerate triangle was accepted.");
        Transform::computeBarycentric3D(Vector3f(0, 1, 1),
            Vector3f(0, 0, 0), Vector3f(0, 2, 0), Vector3f(0, 0, 2), alpha, beta, gamma);
        requireNear(Vector3f(alpha, beta, gamma), Vector3f(0, .5f, .5f),
                    "Legacy world-space barycentrics lost their Z component.");
    });

    run("transform returns clip W before perspective division for translated and rotated cameras", [&] {
        for (const Pose& pose : {front, back,
             Pose{Vector3f(5, 0, 0), Vector3f::Zero(), Vector3f::UnitY()}}) {
            float clipW;
            const Vector3f screen = Transform::transform(Vector3f::Zero(), pose.eye, pose.target, pose.up, clipW);
            require(std::abs(clipW - 5.0f) < 1e-5f, "Clip W was confused with world Z or divided away.");
            requireNear(screen, Transform::transform(Vector3f::Zero(), pose.eye, pose.target, pose.up),
                        "Returning clip W changed the original screen-position result.");
        }
    });

    run("rasterization fills the frame and keeps the near surface from either camera side", [&] {
        for (const Pose& pose : {front, back}) {
            RasterizerFixture fixture(SCREEN_WIDTH, SCREEN_HEIGHT);
            Snapshot expected;
            for (const objLoader* model : {&layered, &reversed}) {
                fixture.renderer.clear();
                fixture.renderer.rasterize(*model, pose.eye, pose.target, pose.up);
                const Snapshot buffers = RasterizerTraceTestAccess::capture(fixture.renderer);
                const bool reverseView = pose.eye.z() < 0;
                const Vector3f nearest(0, 0, reverseView ? -1.0f : 1.0f);
                requireDepth(buffers.depths[SCREEN_HEIGHT / 2][SCREEN_WIDTH / 2],
                    Transform::transform(nearest, pose.eye, pose.target, pose.up).z(),
                    "Rasterization retained the farther projected depth.");
                requireNear(buffers.colors[SCREEN_HEIGHT / 2][SCREEN_WIDTH / 2], reverseView ? blue : red,
                            "The farther material covered the near surface.");
                std::size_t covered = 0;
                for (const auto& row : buffers.depths)
                    covered += std::count_if(row.begin(), row.end(), [](float depth) { return std::isfinite(depth); });
                require(covered == static_cast<std::size_t>(SCREEN_WIDTH) * SCREEN_HEIGHT,
                        "Rasterization stopped before processing all covered pixels.");
                if (expected.colors.empty()) expected = buffers;
                else requireBuffers(buffers, expected);
            }
        }
    });

    run("rasterized perspective-correct attributes agree with world-space ray intersections", [&] {
        const objLoader tilted = loadModel(tiltedPath);
        const auto& triangle = tilted.triangles().front();
        const light sceneLight(Vector3f(3, 4, 6), Vector3f(.8f, .6f, .4f), 1.0f);
        const std::array<Pose, 3> poses = {
            front, Pose{Vector3f(4, 2, 5), Vector3f::Zero(), Vector3f::UnitY()},
            Pose{front.eye, front.target, Vector3f::UnitX()}
        };
        const std::array<std::pair<int, int>, 3> pixels = {
            std::pair<int, int>{SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2},
            {SCREEN_WIDTH / 2 + 20, SCREEN_HEIGHT / 2 - 15},
            {SCREEN_WIDTH / 2 - 20, SCREEN_HEIGHT / 2 + 10}
        };
        for (const Pose& pose : poses) {
            RasterizerFixture fixture(SCREEN_WIDTH, SCREEN_HEIGHT, sceneLight);
            fixture.renderer.clear();
            fixture.renderer.rasterize(tilted, pose.eye, pose.target, pose.up);
            const Snapshot buffers = RasterizerTraceTestAccess::capture(fixture.renderer);
            const Eigen::Matrix4f view = Transform::getViewMatrix(pose.eye, pose.target, pose.up);
            const Eigen::Matrix3f cameraToWorld = view.block<3, 3>(0, 0).transpose();
            const Eigen::Matrix4f projection = Transform::getProjectionMatrix();
            for (const auto& pixel : pixels) {
                const float ndcX = 2.0f * (pixel.first + .5f) / SCREEN_WIDTH - 1.0f;
                const float ndcY = 1.0f - 2.0f * (pixel.second + .5f) / SCREEN_HEIGHT;
                const Vector3f cameraDirection(ndcX / projection(0, 0), ndcY / projection(1, 1), -1.0f);
                const Vector3f direction = (cameraToWorld * cameraDirection).normalized();
                ray sample(pose.eye, direction);
                Vector3f first = triangle.vertices[0].position;
                Vector3f second = triangle.vertices[1].position;
                Vector3f third = triangle.vertices[2].position;
                sample.findIntersectionPoint(first, second, third);
                require(sample.getIntersect(), "Reference ray missed the tilted fixture.");
                const Vector3f point = sample.getIntersectionPoint();
                float alpha, beta, gamma;
                Transform::computeBarycentric3D(point, first, second, third, alpha, beta, gamma);
                const Vector3f normal = (alpha * triangle.vertices[0].normal
                    + beta * triangle.vertices[1].normal + gamma * triangle.vertices[2].normal).normalized();
                const Vector3f color = fixture.shader.computeShading(tilted.materialFor(triangle), pose.eye, point, normal);
                requireNear(buffers.normals[pixel.second][pixel.first], normal,
                            "Rasterized normal did not use perspective-correct weights.");
                requireNear(buffers.colors[pixel.second][pixel.first], color,
                            "Rasterized lighting used an incorrect world position or normal.");
                requireDepth(buffers.depths[pixel.second][pixel.first],
                    Transform::transform(point, pose.eye, pose.target, pose.up).z(),
                    "Projected depth received an extra perspective correction.");
            }
        }
    });

    run("cleared raster frames discard old pixels and screen-degenerate triangles", [&] {
        RasterizerFixture fixture(SCREEN_WIDTH, SCREEN_HEIGHT);
        fixture.renderer.clear();
        fixture.renderer.rasterize(partial, front.eye, front.target, front.up);
        require(std::isfinite(RasterizerTraceTestAccess::capture(fixture.renderer)
                .depths[SCREEN_HEIGHT / 2][SCREEN_WIDTH / 2]), "World Z zero prevented a valid triangle from drawing.");
        fixture.renderer.clear();
        fixture.renderer.rasterize(partial, front.eye, Vector3f(0, 0, 10), front.up);
        requireEmpty(RasterizerTraceTestAccess::capture(fixture.renderer), SCREEN_WIDTH, SCREEN_HEIGHT);
        fixture.renderer.clear();
        fixture.renderer.rasterize(loadModel(edgeOnPath), front.eye, front.target, front.up);
        requireEmpty(RasterizerTraceTestAccess::capture(fixture.renderer), SCREEN_WIDTH, SCREEN_HEIGHT);
    });

    run("worker counts, camera poses and triangle orders preserve all buffers", [&] {
        const std::array<Pose, 4> poses = {
            front, back,
            Pose{Vector3f(4, 3, 5), Vector3f::Zero(), Vector3f::UnitY()},
            Pose{front.eye, front.target, Vector3f::UnitX()}
        };
        for (const auto& pose : poses) {
            RasterizerFixture baseline;
            const Snapshot expected = trace(baseline.renderer, layered, pose, 1);
            requireDimensions(expected, testWidth, testHeight);
            requireTriangles(expected.triangles, layered.triangles());
            const bool reverseView = pose.eye.z() < 0;
            requireCenter(expected, reverseView ? -1.0f : 1.0f,
                          reverseView ? blue : red,
                          reverseView ? Vector3f(0, 0, -1) : Vector3f(0, 0, 1));
            for (const objLoader* model : {&layered, &reversed}) {
                const Snapshot orderedExpected = trace(baseline.renderer, *model, pose, 1);
                requireBuffers(orderedExpected, expected, false);
                for (unsigned int workers : {1u, 2u, 4u, 14u, 0u}) {
                    RasterizerFixture fixture;
                    RasterizerTraceTestAccess::poison(fixture.renderer);
                    requireBuffers(trace(fixture.renderer, *model, pose, workers), orderedExpected);
                }
            }
        }
    });

    run("small and single-row buffers schedule every pixel", [&] {
        for (const auto& size : {std::pair<int, int>{13, 9}, {7, 1}, {1, 5}, {1, 1}}) {
            for (unsigned int workers : {1u, 2u, 4u, 14u}) {
                RasterizerFixture fixture(size.first, size.second);
                const Snapshot buffers = trace(fixture.renderer, layered, front, workers);
                requireDimensions(buffers, size.first, size.second);
                requireTriangles(buffers.triangles, layered.triangles());
                for (std::size_t index = 0; index < layered.triangles().size(); ++index) {
                    require(sameTriangle(buffers.triangles[index], layered.triangles()[index]),
                            "Triangles hit by the first pixel were not ordered by model index.");
                }
                for (int row = 0; row < size.second; ++row) {
                    for (int column = 0; column < size.first; ++column) {
                        requireDepth(buffers.depths[row][column], 1.0f, "A scheduled row or pixel was skipped.");
                        requireNear(buffers.colors[row][column], red, "A scheduled pixel has the wrong color.");
                        requireNear(buffers.normals[row][column], Vector3f::UnitZ(),
                                    "A scheduled pixel has the wrong normal.");
                    }
                }
            }
        }
    });

    run("triangle merge preserves pixel-first order rather than model-first order", [&] {
        const objLoader ordered = loadModel(orderedPath);
        const std::vector<objLoader::Triangle> expected = {
            ordered.triangles()[1], ordered.triangles()[0]
        };
        for (unsigned int workers : {1u, 2u, 4u, 14u}) {
            RasterizerFixture fixture;
            const Snapshot buffers = trace(fixture.renderer, ordered, front, workers);
            requireTriangles(buffers.triangles, expected);
            for (std::size_t index = 0; index < expected.size(); ++index) {
                require(sameTriangle(buffers.triangles[index], expected[index]),
                        "Merge ignored the first row-major pixel intersecting each triangle.");
            }
        }
    });

    run("interpolated normals and diffuse colors have independent expected values", [&] {
        const Vector3f lightColor(.8f, .6f, .4f);
        const light sceneLight(Vector3f(0, 0, 5), lightColor, 1.0f);
        const Vector3f normal = Vector3f(.3f, .15f, .85f).normalized();
        const Vector3f color = (Vector3f(.05f, .1f, .15f)
            + Vector3f(.2f, .4f, .6f) * normal.z()).cwiseProduct(lightColor);
        RasterizerFixture baseline(testWidth, testHeight, sceneLight);
        const Snapshot expected = trace(baseline.renderer, partial, front, 1);
        requireCenter(expected, 0.0f, color, normal);
        requireTriangles(expected.triangles, partial.triangles());
        requireDepth(expected.depths.front().front(), emptyDepth, "Background pixel unexpectedly hit a surface.");
        requireNear(expected.colors.front().front(), backgroundColor, "Background color is not blue.");
        requireNear(expected.normals.front().front(), Vector3f::Zero(), "Background normal is not zero.");
        for (unsigned int workers : {2u, 4u, 14u}) {
            RasterizerFixture fixture(testWidth, testHeight, sceneLight);
            requireBuffers(trace(fixture.renderer, partial, front, workers), expected);
        }
    });

    run("world upper-right geometry remains in the upper-right screen quadrant", [&] {
        const objLoader upperRight = loadModel(upperRightPath);
        for (unsigned int workers : {1u, 4u}) {
            RasterizerFixture fixture;
            const Snapshot buffers = trace(fixture.renderer, upperRight, front, workers);
            requireDepth(buffers.depths[2][8], 0.0f, "Upper-right geometry was not hit in the upper-right quadrant.");
            requireNear(buffers.colors[2][8], red, "Upper-right pixel has the wrong color.");
            requireDepth(buffers.depths[6][4], emptyDepth, "Upper-right geometry was mirrored into the lower-left quadrant.");
            requireNear(buffers.colors[6][4], backgroundColor, "Missed ray did not retain the blue background.");
        }
    });

    run("initialization and clear retain the same blue background", [&] {
        shadeModel shader;
        rasterizer renderer(shader);
        renderer.initRasterizer();
        requireEmpty(RasterizerTraceTestAccess::capture(renderer), SCREEN_WIDTH, SCREEN_HEIGHT);
        RasterizerTraceTestAccess::poison(renderer);
        renderer.clear();
        requireEmpty(RasterizerTraceTestAccess::capture(renderer), SCREEN_WIDTH, SCREEN_HEIGHT);
    });

    run("clear preserves dimensions, resets all CPU buffers and is repeatable", [&] {
        RasterizerFixture fixture;
        const Snapshot expected = trace(fixture.renderer, layered, front, 4);
        fixture.renderer.clear();
        requireEmpty(RasterizerTraceTestAccess::capture(fixture.renderer), testWidth, testHeight);
        fixture.renderer.clear();
        requireEmpty(RasterizerTraceTestAccess::capture(fixture.renderer), testWidth, testHeight);
        requireBuffers(trace(fixture.renderer, layered, front, 2), expected);
    });

    run("successive frames automatically clear hits, misses and triangle statistics", [&] {
        RasterizerFixture fixture;
        const Snapshot first = trace(fixture.renderer, layered, front, 1);
        requireCenter(trace(fixture.renderer, layered, back, 4), -1.0f, blue, Vector3f(0, 0, -1));
        const Pose lookingAway{front.eye, Vector3f(0, 0, 10), front.up};
        requireEmpty(trace(fixture.renderer, layered, lookingAway, 2), testWidth, testHeight);
        requireBuffers(trace(fixture.renderer, layered, front, 4), first);
        objLoader empty;
        requireEmpty(trace(fixture.renderer, empty, front, 4), testWidth, testHeight);
        require(fixture.renderer.setGBuffers(layered, front.eye, front.target),
                "Default up and automatic worker selection failed.");
        requireBuffers(RasterizerTraceTestAccess::capture(fixture.renderer), first);
    });

    run("progress is monotonic, complete and delivered on the calling thread", [&] {
        for (unsigned int workers : {1u, 4u, 14u}) {
            RasterizerFixture fixture;
            ProgressLog progress;
            require(fixture.renderer.setGBuffers(layered, front.eye, front.target, front.up, workers,
                [&](int completed, int total) {
                    progress.record(completed, total);
                    return true;
                }), "A progress callback that accepts work cancelled the trace.");
            progress.check(testHeight, true);
            requireCenter(RasterizerTraceTestAccess::capture(fixture.renderer), 1.0f, red, Vector3f::UnitZ());
        }
    });

    run("cancellation returns false and permits clean subsequent renders", [&] {
        for (int cancelAt : {0, 1}) {
            RasterizerFixture fixture(37, 31);
            RasterizerFixture baseline(37, 31);
            const Snapshot expected = trace(baseline.renderer, layered, front, 1);
            ProgressLog progress;
            const bool finished = fixture.renderer.setGBuffers(
                layered, front.eye, front.target, front.up, 4,
                [&](int completed, int total) {
                    progress.record(completed, total);
                    return completed < cancelAt;
                });
            require(!finished, "Returning false from progress did not cancel the trace.");
            progress.check(31, false);
            requireBuffers(trace(fixture.renderer, layered, front, 2), expected);
            objLoader empty;
            requireEmpty(trace(fixture.renderer, empty, front, 4), 37, 31);
            requireBuffers(trace(fixture.renderer, layered, front, 1), expected);
        }
    });

    std::cout << (failed == 0 ? "All trace tests passed.\n" : "Trace tests failed.\n");
    return failed == 0 ? 0 : 1;
}

unsigned int parseUnsigned(const char* text, const std::string& name, bool allowZero) {
    const std::string value(text);
    require(!value.empty() && value.front() != '-', name + " must be an unsigned integer.");
    std::size_t consumed = 0;
    const unsigned long long parsed = std::stoull(value, &consumed);
    require(consumed == value.size() && parsed <= std::numeric_limits<unsigned int>::max()
            && (allowZero || parsed > 0), name + " is out of range.");
    return static_cast<unsigned int>(parsed);
}

void hashWord(std::uint64_t& hash, std::uint32_t word) {
    for (unsigned int shift = 0; shift < 32; shift += 8) {
        hash ^= (word >> shift) & 0xffu;
        hash *= 1099511628211ull;
    }
}

void hashFloat(std::uint64_t& hash, float value) {
    static_assert(sizeof(float) == sizeof(std::uint32_t), "Expected 32-bit floats.");
    std::uint32_t bits = 0;
    if (value != 0.0f) std::memcpy(&bits, &value, sizeof(bits));
    hashWord(hash, bits);
}

void writePPM(const fs::path& path, const Snapshot& buffers, int width, int height) {
    std::ofstream stream(path, std::ios::binary);
    require(static_cast<bool>(stream), "Cannot open PPM output: " + path.u8string());
    stream << "P6\n" << width << ' ' << height << "\n255\n";
    for (const auto& row : buffers.colors) {
        for (const auto& color : row) {
            std::array<unsigned char, 3> pixel{};
            for (int channel = 0; channel < 3; ++channel) {
                pixel[channel] = static_cast<unsigned char>(std::clamp(color[channel], 0.0f, 1.0f) * 255.0f);
            }
            stream.write(reinterpret_cast<const char*>(pixel.data()), pixel.size());
        }
    }
    stream.close();
    require(static_cast<bool>(stream), "Cannot write PPM output: " + path.u8string());
}

int benchmark(const fs::path& path, unsigned int workers, int width, int height,
              const fs::path& output) {
    const objLoader model = loadModel(path);
    Vector3f minimum = Vector3f::Constant(std::numeric_limits<float>::infinity());
    Vector3f maximum = Vector3f::Constant(-std::numeric_limits<float>::infinity());
    for (const auto& triangle : model.triangles()) {
        for (const auto& vertex : triangle.vertices) {
            minimum = minimum.cwiseMin(vertex.position);
            maximum = maximum.cwiseMax(vertex.position);
        }
    }
    const Vector3f center = (minimum + maximum) * .5f;
    const float extent = std::max((maximum - minimum).maxCoeff(), .1f);
    const Pose pose{center + Vector3f(0, 0, 2.0f * extent), center, Vector3f::UnitY()};
    const light sceneLight(center + Vector3f(extent, extent, 2.0f * extent), Vector3f::Ones(), 1.0f);
    RasterizerFixture fixture(width, height, sceneLight);
    const auto start = std::chrono::steady_clock::now();
    const bool completed = fixture.renderer.setGBuffers(model, pose.eye, pose.target, pose.up, workers);
    const auto stop = std::chrono::steady_clock::now();
    require(completed, "Benchmark trace unexpectedly cancelled.");
    const Snapshot buffers = RasterizerTraceTestAccess::capture(fixture.renderer);
    requireDimensions(buffers, width, height);
    std::uint64_t colorHash = 14695981039346656037ull;
    std::uint64_t depthHash = colorHash;
    std::uint64_t normalHash = colorHash;
    for (auto* hash : {&colorHash, &depthHash, &normalHash}) {
        hashWord(*hash, static_cast<std::uint32_t>(width));
        hashWord(*hash, static_cast<std::uint32_t>(height));
    }
    std::size_t hitPixels = 0;
    std::size_t nonblackPixels = 0;
    std::size_t invalidPixels = 0;
    for (int row = 0; row < height; ++row) {
        for (int column = 0; column < width; ++column) {
            const Vector3f& color = buffers.colors[row][column];
            const Vector3f& normal = buffers.normals[row][column];
            const float depth = buffers.depths[row][column];
            if (std::isfinite(depth)) ++hitPixels;
            if (color.cwiseAbs().maxCoeff() > 1e-6f) ++nonblackPixels;
            if (!color.allFinite() || !normal.allFinite()
                || (!std::isfinite(depth) && depth != emptyDepth)) ++invalidPixels;
            for (int channel = 0; channel < 3; ++channel) {
                hashFloat(colorHash, color[channel]);
                hashFloat(normalHash, normal[channel]);
            }
            hashFloat(depthHash, depth);
        }
    }
    const double milliseconds = std::chrono::duration<double, std::milli>(stop - start).count();
    std::cout << "model=" << path.u8string() << '\n'
              << "size=" << width << 'x' << height << " workers=" << workers
              << " triangles=" << model.triangles().size()
              << " intersected_triangles=" << buffers.triangles.size() << '\n'
              << "eye=" << pose.eye.transpose() << " target=" << pose.target.transpose() << '\n'
              << "elapsed_ms=" << std::fixed << std::setprecision(3) << milliseconds << '\n'
              << "hit_pixels=" << hitPixels << " nonblack_pixels=" << nonblackPixels
              << " invalid_pixels=" << invalidPixels << '\n'
              << std::hex << std::setfill('0')
              << "color_hash=" << std::setw(16) << colorHash
              << " depth_hash=" << std::setw(16) << depthHash
              << " normal_hash=" << std::setw(16) << normalHash << '\n'
              << std::dec << std::setfill(' ');
    require(invalidPixels == 0, "Benchmark produced non-finite buffer values.");
    if (!output.empty()) {
        writePPM(output, buffers, width, height);
        std::cout << "ppm=" << output.u8string() << '\n';
    }
    return 0;
}
}

int main(int argc, char** argv) {
    try {
        if (argc == 1) return runTests();
        if (argc >= 4 && argc <= 7 && std::string(argv[1]) == "--benchmark") {
            const unsigned int workers = parseUnsigned(argv[3], "workers", true);
            const bool customSize = argc >= 6;
            const unsigned int width = customSize ? parseUnsigned(argv[4], "width", false) : SCREEN_WIDTH;
            const unsigned int height = customSize ? parseUnsigned(argv[5], "height", false) : SCREEN_HEIGHT;
            const fs::path output = argc == 5 ? fs::u8path(argv[4])
                : argc == 7 ? fs::u8path(argv[6]) : fs::path{};
            require(width <= static_cast<unsigned int>(std::numeric_limits<int>::max())
                    && height <= static_cast<unsigned int>(std::numeric_limits<int>::max()),
                    "Benchmark dimensions exceed the supported index range.");
            return benchmark(fs::u8path(argv[2]), workers, static_cast<int>(width), static_cast<int>(height), output);
        }
        std::cerr << "Usage: rasterizer_trace_tests [--benchmark <modelPath> <workers> [width height] [output.ppm]]\n";
        return 2;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
