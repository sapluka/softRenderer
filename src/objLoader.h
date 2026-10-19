#pragma once

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

// OBJ 的独立索引在读取时处理完，渲染代码只需要使用这些完整的角点。
class objLoader {
public:
    struct Vertex {
        Eigen::Vector3f position = Eigen::Vector3f::Zero();
        Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();
        Eigen::Vector2f uv = Eigen::Vector2f::Zero();
        Eigen::Vector3f color = Eigen::Vector3f::Ones();
        bool hasUV = false;
        bool normalFromFile = false;
    };

    struct TextureMap {
        std::filesystem::path path;
        Eigen::Vector3f offset = Eigen::Vector3f::Zero();
        Eigen::Vector3f scale = Eigen::Vector3f::Ones();
        bool clamp = false;
        float strength = 1.0f; // bump 贴图的 -bm 参数。
    };

    struct Material {//系数
        std::string name = "default";
        Eigen::Vector3f ambient = Eigen::Vector3f::Constant(0.1f); // Ka
        Eigen::Vector3f diffuse = Eigen::Vector3f::Constant(0.8f); // Kd
        Eigen::Vector3f specular = Eigen::Vector3f::Zero();        // Ks
        Eigen::Vector3f emission = Eigen::Vector3f::Zero();        // Ke
        float shininess = 1.0f;  // Ns
        float opacity = 1.0f;    // d，1 为不透明。
        float ior = 1.5f;        // Ni
        int illuminationModel = 1;
        float roughness = 0.5f;
        float metallic = 0.0f;
        TextureMap diffuseTexture;
        TextureMap specularTexture;
        TextureMap emissionTexture;
        TextureMap normalTexture;
        TextureMap bumpTexture;
        TextureMap opacityTexture;
    };

    struct Triangle {
        std::array<Vertex, 3> vertices;
        std::size_t materialIndex = 0;
        std::size_t objectIndex = 0;
        bool isIntersected = false;
        bool hasUVs() const noexcept {
            return vertices[0].hasUV && vertices[1].hasUV && vertices[2].hasUV;
        }
    };

    // 对应 tinyobjloader 输出的命名 object/group，不复制全模型的属性数组。
    struct Object {
        std::string name;
        std::vector<std::size_t> triangleIndices;
    };

    struct Options {
        // 可选：取这个 MTL 的第一个材质作为默认材质。
        // 相对路径以 OBJ 所在目录为基准；为空时使用内置灰色材质。
        std::filesystem::path defaultMaterialFile;
    };

    objLoader();

    // 每次加载替换旧模型。失败时返回 false，模型为空，原因见 error()。
    bool loadObj(const std::filesystem::path& filename);
    bool loadObj(const std::filesystem::path& filename, const Options& options);
    void clear();

    const std::vector<Triangle>& triangles() const noexcept { return triangles_; }
    const std::vector<Material>& materials() const noexcept { return materials_; }
    const std::vector<Object>& objects() const noexcept { return objects_; }
    const Material& materialFor(const Triangle& triangle) const {
        return materials_.at(triangle.materialIndex);
    }
    const Object& objectFor(const Triangle& triangle) const {
        return objects_.at(triangle.objectIndex);
    }
    const std::string& error() const noexcept { return error_; }
    const std::string& warnings() const noexcept { return warnings_; }
    bool empty() const noexcept { return triangles_.empty(); }

private:
    std::vector<Triangle> triangles_;
    std::vector<Material> materials_;
    std::vector<Object> objects_;
    std::string error_;
    std::string warnings_;
};
