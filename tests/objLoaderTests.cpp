#include "objLoader.h"
#include <Eigen/Geometry>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
namespace fs = std::filesystem;
const std::string points = "v 0 0 0\nv 1 0 0\nv 0 1 0\n";

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

bool near(float a, float b) { return std::abs(a - b) < 1e-5f; }

fs::path writeFixture(const fs::path& root, const fs::path& name, const std::string& text) {
    const fs::path path = root / name;
    fs::create_directories(path.parent_path());
    std::ofstream stream(path, std::ios::binary);
    stream << text;
    require(static_cast<bool>(stream), "Cannot write test fixture.");
    return path;
}

void load(objLoader& model, const fs::path& path) {
    require(model.loadObj(path), path.u8string() + ": " + model.error());
}

void checkModel(const objLoader& model) {
    require(!model.empty(), "Expected triangle surfaces.");
    std::vector<unsigned int> owners(model.triangles().size(), 0);
    for (std::size_t object = 0; object < model.objects().size(); ++object) {
        for (std::size_t index : model.objects()[object].triangleIndices) {
            require(index < owners.size(), "Invalid object triangle index.");
            require(model.triangles()[index].objectIndex == object, "Wrong triangle owner.");
            ++owners[index];
        }
    }
    for (std::size_t i = 0; i < model.triangles().size(); ++i) {
        const auto& triangle = model.triangles()[i];
        require(owners[i] == 1, "Triangle must belong to exactly one object.");
        require(triangle.materialIndex < model.materials().size(), "Invalid material index.");
        for (const auto& vertex : triangle.vertices) {
            require(vertex.position.allFinite(), "Non-finite position.");
            require(vertex.normal.allFinite() && near(vertex.normal.norm(), 1.0f), "Normal must be finite and unit length.");
            require(vertex.uv.allFinite() && vertex.color.allFinite(), "Non-finite vertex attributes.");
        }
    }
}
} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: obj_loader_tests <assets-directory>\n";
        return 2;
    }
    const fs::path assets = fs::absolute(fs::u8path(argv[1]));
    const fs::path fixtures = fs::current_path() / "obj_loader_test_fixtures";
    std::size_t passed = 0;
    std::size_t failed = 0;
    const auto run = [&](const std::string& name, const std::function<void()>& test) {
        try {
            test();
            ++passed;
            std::cout << "PASS " << name << '\n';
        } catch (const std::exception& error) {
            ++failed;
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        }
    };

    const std::vector<std::pair<std::string, std::size_t>> assetCases = {
        {"bunny/bunny.obj", 4968}, {"monkey/monkey.obj", 968},
        {"cornell_box/cornell_box.obj", 36}, {"test/test.obj", 1060},
        {"test1/test1.obj", 980}
    };
    for (const auto& item : assetCases) {
        run("asset " + item.first, [&] {
            objLoader model;
            load(model, assets / item.first);
            require(model.triangles().size() == item.second, "Unexpected triangulated face count.");
            checkModel(model);
        });
    }

    run("four corner formats and relative indices", [&] {
        const auto path = writeFixture(fixtures, "formats.obj", points +
            "vt 0 0\nvt 1 0\nvt 0 1\nvn 0 0 2\n"
            "f 1 2 3\nf 1/1 2/2 3/3\nf 1//1 2//1 3//1\n"
            "f -3/-3/-1 -2/-2/-1 -1/-1/-1 # relative attributes\n");
        objLoader model;
        load(model, path);
        require(model.triangles().size() == 4, "Lost faces.");
        require(!model.triangles()[0].hasUVs() && model.triangles()[1].hasUVs(), "Wrong UV availability.");
        require(!model.triangles()[1].vertices[0].normalFromFile, "Missing normal not generated.");
        require(model.triangles()[2].vertices[0].normalFromFile, "Explicit normal lost.");
        require(model.triangles()[3].vertices[1].position.isApprox(Eigen::Vector3f(1, 0, 0)), "Wrong relative position.");
        require(model.triangles()[3].vertices[2].uv.isApprox(Eigen::Vector2f(0, 1)), "Wrong relative UV.");
        checkModel(model);
    });

    run("independent UV and normal indices at a shared position", [&] {
        const auto path = writeFixture(fixtures, "seams.obj", points +
            "v 0 0 1\nvt 0 0\nvt 1 0\nvt 0 1\nvt .25 .75\nvt .5 .5\nvt 1 1\n"
            "vn 0 0 1\nvn 1 0 0\nf 1/1/1 2/2/1 3/3/1\nf 1/4/2 3/5/2 4/6/2\n");
        objLoader model;
        load(model, path);
        const auto& a = model.triangles()[0].vertices[0];
        const auto& b = model.triangles()[1].vertices[0];
        require(a.position.isApprox(b.position), "Shared position changed.");
        require(a.uv.isApprox(Eigen::Vector2f(0, 0)) && b.uv.isApprox(Eigen::Vector2f(.25f, .75f)), "UV seam was merged.");
        require(a.normal.isApprox(Eigen::Vector3f::UnitZ()) && b.normal.isApprox(Eigen::Vector3f::UnitX()), "Hard edge was merged.");
        checkModel(model);
    });

    run("objects retain global OBJ numbering", [&] {
        const auto path = writeFixture(fixtures, "objects.obj", points +
            "o first\nf 1 2 3\no second\nv 0 0 1\nf -4 -2 -1\n");
        objLoader model;
        load(model, path);
        require(model.objects().size() == 2, "Object names lost.");
        require(model.objectFor(model.triangles()[1]).name == "second", "Wrong object mapping.");
        require(model.triangles()[1].vertices[0].position.isZero(), "Indices restarted at o.");
        checkModel(model);
    });

    for (bool clockwise : {false, true}) {
        run(clockwise ? "clockwise concave polygon" : "counterclockwise concave polygon", [&] {
            const auto path = writeFixture(fixtures, clockwise ? "concave_cw.obj" : "concave.obj",
                "v 0 0 0\nv 2 0 0\nv 2 2 0\nv 1 1 0\nv 0 2 0\n" +
                std::string(clockwise ? "f 5 4 3 2 1\n" : "f 1 2 3 4 5\n"));
            objLoader model;
            load(model, path);
            require(model.triangles().size() == 3, "Pentagon must produce three triangles.");
            float area = 0;
            for (const auto& triangle : model.triangles()) {
                const auto& v = triangle.vertices;
                const float signedArea = .5f * (v[1].position - v[0].position).cross(v[2].position - v[0].position).z();
                require(clockwise ? signedArea < 0 : signedArea > 0, "Winding was reversed.");
                area += signedArea;
                const Eigen::Vector3f center = (v[0].position + v[1].position + v[2].position) / 3.0f;
                require(center.y() <= std::max(2.0f - center.x(), center.x()) + 1e-5f, "Triangle crosses the concave notch.");
            }
            require(near(area, clockwise ? -3.0f : 3.0f), "Triangulation changed polygon area.");
            checkModel(model);
        });
    }

    run("concave quad uses its interior diagonal", [&] {
        const auto path = writeFixture(fixtures, "concave_quad.obj",
            "v 0 0 0\nv 10 0 0\nv 9 .1 0\nv 10 .2 0\nf 1 2 3 4\n");
        objLoader model;
        load(model, path);
        require(model.triangles().size() == 2, "Quad must produce two triangles.");
        float area = 0;
        for (const auto& triangle : model.triangles()) {
            const auto& v = triangle.vertices;
            const float signedArea = .5f * (v[1].position - v[0].position).cross(v[2].position - v[0].position).z();
            require(signedArea > 0, "Quad split introduced an overlapping or reversed triangle.");
            area += signedArea;
        }
        require(near(area, .9f), "Quad area changed.");
    });

    for (bool smooth : {false, true}) {
        run(smooth ? "generated smoothing-group normals" : "generated flat normals", [&] {
            const auto path = writeFixture(fixtures, smooth ? "smooth.obj" : "flat.obj", points +
                "v 0 0 1\n" + std::string(smooth ? "s 1\n" : "s off\n") + "f 1 2 3\nf 1 3 4\n");
            objLoader model;
            load(model, path);
            const Eigen::Vector3f expected = smooth ? Eigen::Vector3f(1, 0, 1).normalized() : Eigen::Vector3f::UnitZ();
            require(model.triangles()[0].vertices[0].normal.isApprox(expected), "Wrong generated normal.");
            require(!model.triangles()[0].vertices[0].normalFromFile, "Wrong normal source flag.");
            checkModel(model);
        });
    }

    run("zero-length explicit normals are repaired", [&] {
        objLoader model;
        load(model, writeFixture(fixtures, "zero_normal.obj", points + "vn 0 0 0\nf 1//1 2//1 3//1\n"));
        require(!model.triangles()[0].vertices[0].normalFromFile, "Invalid normal was used.");
        checkModel(model);
    });

    run("missing and empty MTL use gray material", [&] {
        writeFixture(fixtures, "empty.mtl", "# no materials\n");
        for (const std::string name : {"absent.mtl", "empty.mtl"}) {
            objLoader model;
            load(model, writeFixture(fixtures, "missing_material.obj", "mtllib " + name + "\n" + points + "usemtl ghost\nf 1 2 3\n"));
            require(model.triangles()[0].materialIndex == 0, "Default material not assigned.");
            require(model.materialFor(model.triangles()[0]).diffuse.isApprox(Eigen::Vector3f::Constant(.8f)), "Wrong gray fallback.");
            require(!model.warnings().empty(), "Missing material must be reported.");
        }
    });

    run("custom default MTL and fallback on missing default file", [&] {
        writeFixture(fixtures, "fallback.mtl", "newmtl fallback\nKd .2 .3 .4\n");
        const auto path = writeFixture(fixtures, "fallback.obj", points + "f 1 2 3\n");
        objLoader model;
        objLoader::Options options;
        options.defaultMaterialFile = "fallback.mtl";
        require(model.loadObj(path, options), model.error());
        require(model.materialFor(model.triangles()[0]).diffuse.isApprox(Eigen::Vector3f(.2f, .3f, .4f)), "Custom fallback ignored.");
        options.defaultMaterialFile = "not_present.mtl";
        require(model.loadObj(path, options), model.error());
        require(model.materialFor(model.triangles()[0]).diffuse.isApprox(Eigen::Vector3f::Constant(.8f)), "Built-in fallback lost.");
        require(!model.warnings().empty(), "Missing default file not reported.");
    });

    run("material changes and MTL-relative texture options", [&] {
        writeFixture(fixtures, "mats/red.mtl", "newmtl red\nKa .1 .2 .3\nKd 1 0 0\nKs .4 .5 .6\nNs 77\nNi 1.33\nd .75\nKe .2 .1 0\n"
            "map_Kd -o .1 .2 0 -s 2 3 1 -clamp on ../textures/image.png\n");
        writeFixture(fixtures, "mats/blue.mtl", "newmtl blue\nKd 0 0 1\n");
        const auto path = writeFixture(fixtures, "materials.obj", "mtllib mats/red.mtl mats/blue.mtl\n" + points +
            "usemtl red\nf 1 2 3\nusemtl blue\nf 1 2 3\nusemtl unknown\nf 1 2 3\n");
        objLoader model;
        load(model, path);
        const auto& red = model.materialFor(model.triangles()[0]);
        require(red.name == "red" && red.diffuse.isApprox(Eigen::Vector3f(1, 0, 0)), "Wrong material.");
        require(near(red.shininess, 77) && near(red.opacity, .75f) && near(red.ior, 1.33f), "Material coefficients lost.");
        require(red.diffuseTexture.path == (fixtures / "textures/image.png").lexically_normal(), "Texture path must be relative to its MTL.");
        require(red.diffuseTexture.scale.isApprox(Eigen::Vector3f(2, 3, 1)) && red.diffuseTexture.clamp, "Texture options lost.");
        require(red.diffuseTexture.offset.isApprox(Eigen::Vector3f(.1f, .2f, 0)), "Texture offset lost.");
        require(model.materialFor(model.triangles()[1]).name == "blue", "Material switching failed.");
        require(model.triangles()[2].materialIndex == 0, "Unknown material must fall back.");
        checkModel(model);
    });

    run("vertex colors and white fallback", [&] {
        const auto path = writeFixture(fixtures, "colors.obj", "v 0 0 0 1 0 0\nv 1 0 0\nv 0 1 0 0 0 1\nf 1 2 3\n");
        objLoader model;
        load(model, path);
        const auto& v = model.triangles()[0].vertices;
        require(v[0].color.isApprox(Eigen::Vector3f(1, 0, 0)), "Red vertex lost.");
        require(v[1].color.isApprox(Eigen::Vector3f::Ones()), "Missing vertex color must be white.");
        require(v[2].color.isApprox(Eigen::Vector3f(0, 0, 1)), "Blue vertex lost.");
    });

    run("UTF-8 object, material and file names", [&] {
        const auto folder = fs::u8path(u8"中文目录");
        writeFixture(fixtures, folder / fs::u8path(u8"材质.mtl"), u8"newmtl 红色\nKd 1 0 0\nmap_Kd 纹理.png\n");
        const auto path = writeFixture(fixtures, folder / fs::u8path(u8"模型.obj"),
            std::string(u8"mtllib 材质.mtl\no 中文物体\n") + points + u8"usemtl 红色\nf 1 2 3\n");
        objLoader model;
        load(model, path);
        require(model.objects()[0].name == u8"中文物体", "UTF-8 object name lost.");
        require(model.materialFor(model.triangles()[0]).name == u8"红色", "UTF-8 material name lost.");
        require(model.materialFor(model.triangles()[0]).diffuseTexture.path.filename() == fs::u8path(u8"纹理.png"), "UTF-8 texture name lost.");
    });

    run("invalid indices fail without retaining an old model", [&] {
        const auto good = writeFixture(fixtures, "good.obj", points + "f 1 2 3\n");
        const std::vector<std::string> invalid = {
            "f 1 2 99\n", "f 0 2 3\n", "f -4 -2 -1\n",
            "vt 0 0\nf 1/1 2/9 3/1\n", "vn 0 0 1\nf 1//1 2//9 3//1\n"
        };
        objLoader model;
        for (const auto& face : invalid) {
            load(model, good);
            const auto bad = writeFixture(fixtures, "bad.obj", points + face);
            require(!model.loadObj(bad), "Invalid index was accepted.");
            require(model.empty() && model.objects().empty(), "Failed load retained geometry.");
            require(!model.error().empty(), "Failed load needs an error message.");
        }
        load(model, good);
        require(!model.loadObj(fixtures / "missing.obj") && model.empty(), "Missing file retained old geometry.");
        load(model, good);
        load(model, good);
        require(model.triangles().size() == 1, "Repeated load appended geometry.");
    });

    run("empty and degenerate surfaces", [&] {
        objLoader model;
        require(!model.loadObj(writeFixture(fixtures, "empty.obj", "# empty\n")), "Empty file accepted.");
        require(!model.loadObj(writeFixture(fixtures, "degenerate.obj", points + "f 1 1 2\n")), "Zero-area-only model accepted.");
        load(model, writeFixture(fixtures, "partly_degenerate.obj", points + "f 1 2 3\nf 1 1 2\n"));
        require(model.triangles().size() == 1 && !model.warnings().empty(), "Zero-area face was not skipped and reported.");
    });

    std::cout << passed << " passed, " << failed << " failed\n";
    return failed == 0 ? 0 : 1;
}
