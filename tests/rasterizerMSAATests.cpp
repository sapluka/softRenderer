#include "rasterizer.h"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

struct RasterizerMSAATestAccess {
    static const Eigen::Vector3f& pixel(const rasterizer& renderer, int x, int y) {
        return renderer.frameBuffer[y][x];
    }
    static const std::array<Eigen::Vector3f, 4>& colors(const rasterizer& renderer, int x, int y) {
        return renderer.subFrameBuffer[y][x];
    }
    static const std::array<float, 4>& depths(const rasterizer& renderer, int x, int y) {
        return renderer.subZBuffer[y][x];
    }
};

namespace {
using Eigen::Vector2f;
using Eigen::Vector3f;
namespace fs = std::filesystem;
constexpr int pixelX = SCREEN_WIDTH / 2;
constexpr int pixelY = SCREEN_HEIGHT / 2;
const Vector3f eye = Vector3f::Zero();
const Vector3f target(0.0f, 0.0f, -1.0f);
const Vector3f background(.5f, .8f, .9f);
const Vector3f red(.8f, .1f, .1f);
const Vector3f blue(.1f, .2f, .8f);

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void requireNear(const Vector3f& actual, const Vector3f& expected, const std::string& message) {
    require(actual.allFinite() && (actual - expected).norm() < 2e-5f, message);
}

struct Face {
    std::array<Vector2f, 3> points;
    float distance;
    std::string material;
    bool varyingNormals = false;
};

Face leftFace(float distance, const std::string& material) {
    return {{{Vector2f(pixelX - 4.0f, pixelY + .5f),
              Vector2f(pixelX + .6f, pixelY - 4.0f),
              Vector2f(pixelX + .6f, pixelY + 5.0f)}}, distance, material};
}

Face rightFace(float distance, const std::string& material) {
    return {{{Vector2f(pixelX + .6f, pixelY - 4.0f),
              Vector2f(pixelX + 5.0f, pixelY + .5f),
              Vector2f(pixelX + .6f, pixelY + 5.0f)}}, distance, material};
}

Face fullFace(float distance, const std::string& material, bool varyingNormals = false) {
    return {{{Vector2f(pixelX - 2.0f, pixelY - 2.0f),
              Vector2f(pixelX + 4.0f, pixelY - 2.0f),
              Vector2f(pixelX + .5f, pixelY + 5.0f)}}, distance, material, varyingNormals};
}

// For this identity-view camera, invert only the known projection and viewport.
Vector3f worldPoint(const Vector2f& screen, float distance) {
    const Eigen::Matrix4f projection = Transform::getProjectionMatrix();
    const float ndcX = 2.0f * screen.x() / SCREEN_WIDTH - 1.0f;
    const float ndcY = 1.0f - 2.0f * screen.y() / SCREEN_HEIGHT;
    return Vector3f(ndcX * distance / projection(0, 0),
                    ndcY * distance / projection(1, 1), -distance);
}

class Fixtures {
public:
    Fixtures() {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        directory = fs::temp_directory_path() / ("renderer_msaa_tests_" + std::to_string(stamp));
        require(fs::create_directory(directory), "Cannot create MSAA test fixtures.");
        write("materials.mtl",
              "newmtl white\nKa 1 1 1\nKd 0 0 0\nKs 0 0 0\n"
              "newmtl red\nKa .8 .1 .1\nKd 0 0 0\nKs 0 0 0\n"
              "newmtl blue\nKa .1 .2 .8\nKd 0 0 0\nKs 0 0 0\n"
              "newmtl shaded\nKa .03 .04 .05\nKd .6 .4 .2\nKs 0 0 0\n");
    }

    ~Fixtures() {
        std::error_code ignored;
        for (const auto& file : files) fs::remove(file, ignored);
        fs::remove(directory, ignored);
    }

    objLoader model(const std::string& name, const std::vector<Face>& faces) {
        std::ostringstream text;
        text << std::setprecision(std::numeric_limits<float>::max_digits10);
        text << "mtllib materials.mtl\n";
        std::size_t index = 1;
        for (const auto& face : faces) {
            for (const auto& screen : face.points) {
                const Vector3f point = worldPoint(screen, face.distance);
                text << "v " << point.x() << ' ' << point.y() << ' ' << point.z() << '\n';
            }
            if (face.varyingNormals) text << "vn 0 0 1\nvn .8 0 .6\nvn 0 .8 .6\n";
            else text << "vn 0 0 1\nvn 0 0 1\nvn 0 0 1\n";
            text << "usemtl " << face.material << "\nf ";
            for (std::size_t corner = 0; corner < 3; ++corner)
                text << index + corner << "//" << index + corner << (corner == 2 ? '\n' : ' ');
            index += 3;
        }
        const fs::path path = write(name, text.str());
        objLoader result;
        require(result.loadObj(path), "Cannot load MSAA fixture: " + result.error());
        return result;
    }

private:
    fs::path directory;
    std::vector<fs::path> files;

    fs::path write(const std::string& name, const std::string& contents) {
        const fs::path path = directory / name;
        std::ofstream stream(path);
        stream << contents;
        require(static_cast<bool>(stream), "Cannot write MSAA fixture.");
        files.push_back(path);
        return path;
    }
};

void render(rasterizer& renderer, const objLoader& model) {
    renderer.clear();
    renderer.rasterize(model, eye, target);
}

void requireSamples(const rasterizer& renderer, const std::array<Vector3f, 4>& expected) {
    const auto& colors = RasterizerMSAATestAccess::colors(renderer, pixelX, pixelY);
    for (int sample = 0; sample < 4; ++sample)
        requireNear(colors[sample], expected[sample], "Unexpected color at sample " + std::to_string(sample));
}
} // namespace

int main() {
    Fixtures files;
    shadeModel shader;
    rasterizer renderer(shader);
    renderer.initRasterizer();
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

    run("adjacent triangles preserve complementary samples without a dark seam", [&] {
        const objLoader adjacent = files.model("adjacent.obj", {leftFace(2.0f, "white"), rightFace(2.0f, "white")});
        render(renderer, adjacent);
        requireSamples(renderer, {Vector3f::Ones(), Vector3f::Ones(), Vector3f::Ones(), Vector3f::Ones()});
        requireNear(RasterizerMSAATestAccess::pixel(renderer, pixelX, pixelY), Vector3f::Ones(),
                    "Complementary triangles darkened a fully covered pixel.");

        const Vector2f topLeft(pixelX - .5f, pixelY - .5f);
        const Vector2f topRight(pixelX + 1.5f, pixelY - .5f);
        const Vector2f bottomRight(pixelX + 1.5f, pixelY + 1.5f);
        const Vector2f bottomLeft(pixelX - .5f, pixelY + 1.5f);
        const objLoader diagonal = files.model("diagonal.obj", {
            {{{topLeft, topRight, bottomRight}}, 2.0f, "white"},
            {{{topLeft, bottomRight, bottomLeft}}, 2.0f, "white"}
        });
        render(renderer, diagonal);
        requireSamples(renderer, {Vector3f::Ones(), Vector3f::Ones(), Vector3f::Ones(), Vector3f::Ones()});
        requireNear(RasterizerMSAATestAccess::pixel(renderer, pixelX, pixelY), Vector3f::Ones(),
                    "Samples exactly on a shared diagonal left a hole.");
    });

    run("partial coverage averages foreground with the original background", [&] {
        const objLoader partial = files.model("partial.obj", {leftFace(2.0f, "red")});
        render(renderer, partial);
        requireSamples(renderer, {red, red, background, background});
        requireNear(RasterizerMSAATestAccess::pixel(renderer, pixelX, pixelY), (red + background) * .5f,
                    "Partially covered pixel lost or counted the background twice.");
        const auto& depths = RasterizerMSAATestAccess::depths(renderer, pixelX, pixelY);
        require(std::isfinite(depths[0]) && std::isfinite(depths[1])
                    && depths[2] == std::numeric_limits<float>::infinity()
                    && depths[3] == std::numeric_limits<float>::infinity(),
                "Uncovered samples acquired foreground depth.");
    });

    run("independent sample depths preserve occlusion in either triangle order", [&] {
        const objLoader nearFirst = files.model("near_first.obj", {leftFace(1.0f, "red"), fullFace(3.0f, "blue")});
        const objLoader farFirst = files.model("far_first.obj", {fullFace(3.0f, "blue"), leftFace(1.0f, "red")});
        render(renderer, nearFirst);
        requireSamples(renderer, {red, red, blue, blue});
        const auto firstDepths = RasterizerMSAATestAccess::depths(renderer, pixelX, pixelY);
        const Vector3f firstColor = RasterizerMSAATestAccess::pixel(renderer, pixelX, pixelY);
        require(firstDepths[0] < firstDepths[2] && firstDepths[1] < firstDepths[3],
                "The near surface did not retain its own sample depths.");
        requireNear(firstColor, (red + blue) * .5f, "Per-sample visibility resolved to the wrong color.");
        render(renderer, farFirst);
        requireSamples(renderer, {red, red, blue, blue});
        requireNear(RasterizerMSAATestAccess::pixel(renderer, pixelX, pixelY), firstColor,
                    "Triangle order changed the resolved color.");
        const auto& secondDepths = RasterizerMSAATestAccess::depths(renderer, pixelX, pixelY);
        for (int sample = 0; sample < 4; ++sample)
            require(std::abs(secondDepths[sample] - firstDepths[sample]) < 2e-5f,
                    "Triangle order changed sample depth.");
    });

    run("MSAA shares one shaded color across a triangle's visible samples", [&] {
        const objLoader shaded = files.model("shaded.obj", {fullFace(2.0f, "shaded", true)});
        render(renderer, shaded);
        const auto& colors = RasterizerMSAATestAccess::colors(renderer, pixelX, pixelY);
        for (int sample = 1; sample < 4; ++sample)
            requireNear(colors[sample], colors[0], "A pixel was independently shaded at each sample as in SSAA.");
        require((colors[0] - background).norm() > .1f, "The shaded triangle did not replace the background.");
        requireNear(RasterizerMSAATestAccess::pixel(renderer, pixelX, pixelY), colors[0],
                    "A fully covered pixel added background to its foreground color.");
    });

    run("clear resets all sample state before an empty frame", [&] {
        renderer.clear();
        objLoader empty;
        renderer.rasterize(empty, eye, target);
        requireSamples(renderer, {background, background, background, background});
        for (float depth : RasterizerMSAATestAccess::depths(renderer, pixelX, pixelY))
            require(depth == std::numeric_limits<float>::infinity(), "Old sample depth survived clearing.");
        requireNear(RasterizerMSAATestAccess::pixel(renderer, pixelX, pixelY), background,
                    "An empty frame retained a previous frame's foreground.");
        requireNear(RasterizerMSAATestAccess::pixel(renderer, 0, 0), background,
                    "Untouched pixels did not retain the background.");
    });

    return failed == 0 ? 0 : 1;
}
