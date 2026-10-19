#include "objLoader.h"

#define TINYOBJLOADER_IMPLEMENTATION
#define TINYOBJLOADER_USE_MAPBOX_EARCUT
#include <tiny_obj_loader.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace {
namespace fs = std::filesystem;

fs::path referencedPath(const fs::path& directory, std::string name) {
    std::replace(name.begin(), name.end(), '\\', '/');
    const fs::path path = fs::u8path(name);
    return (path.is_absolute() ? path : directory / path).lexically_normal();
}

// 保留每份 MTL 的目录，保证子目录中的 map_Kd 等路径也能正确解析。
class MaterialReader final : public tinyobj::MaterialReader {
public:
    explicit MaterialReader(fs::path directory) : directory_(std::move(directory)) {}

    bool operator()(const std::string& name,
                    std::vector<tinyobj::material_t>* materials,
                    std::map<std::string, int>* materialMap,
                    std::string* warning, std::string* error) override {
        const fs::path path = referencedPath(directory_, name);
        std::ifstream stream(path);
        if (!stream) {
            if (warning) *warning += "Cannot open material file: " + path.u8string() + "\n";
            return false;
        }
        const std::size_t first = materials->size();
        tinyobj::LoadMtl(materialMap, materials, &stream, warning, error);
        directories.resize(materials->size());
        for (std::size_t i = first; i < materials->size(); ++i) {
            directories[i] = path.parent_path();
        }
        return true;
    }

    std::vector<fs::path> directories;

private:
    fs::path directory_;
};

Eigen::Vector3f vector3(const tinyobj::real_t* values) {
    return Eigen::Vector3f(values[0], values[1], values[2]);
}

Eigen::Vector3f read3(const std::vector<tinyobj::real_t>& data, int index,
                     const char* attribute) {
    if (index < 0 || static_cast<std::size_t>(index) >= data.size() / 3) {
        throw std::runtime_error(std::string("Out-of-range ") + attribute + " index.");
    }
    const Eigen::Vector3f value = vector3(data.data() + 3 * static_cast<std::size_t>(index));
    if (!value.allFinite()) {
        throw std::runtime_error(std::string("Non-finite ") + attribute + " value.");
    }
    return value;
}

objLoader::TextureMap convertTexture(const std::string& name,
                                    const tinyobj::texture_option_t& options,
                                    const fs::path& directory) {
    objLoader::TextureMap result;
    if (!name.empty()) result.path = referencedPath(directory, name);
    result.offset = vector3(options.origin_offset);
    result.scale = vector3(options.scale);
    result.clamp = options.clamp;
    result.strength = options.bump_multiplier;
    return result;
}

objLoader::Material convertMaterial(const tinyobj::material_t& source,
                                    const fs::path& directory) {
    objLoader::Material result;
    result.name = source.name;
    result.ambient = vector3(source.ambient);
    result.diffuse = vector3(source.diffuse);
    result.specular = vector3(source.specular);
    result.emission = vector3(source.emission);
    result.shininess = source.shininess;
    result.opacity = source.dissolve;
    result.ior = source.ior;
    result.illuminationModel = source.illum;
    result.roughness = source.roughness;
    result.metallic = source.metallic;
    result.diffuseTexture = convertTexture(source.diffuse_texname, source.diffuse_texopt, directory);
    result.specularTexture = convertTexture(source.specular_texname, source.specular_texopt, directory);
    result.emissionTexture = convertTexture(source.emissive_texname, source.emissive_texopt, directory);
    result.normalTexture = convertTexture(source.normal_texname, source.normal_texopt, directory);
    result.bumpTexture = convertTexture(source.bump_texname, source.bump_texopt, directory);
    result.opacityTexture = convertTexture(source.alpha_texname, source.alpha_texopt, directory);
    return result;
}

objLoader::Material defaultMaterial(const objLoader::Options& options,
                                    const fs::path& objDirectory, std::string& warnings) {
    objLoader::Material result;
    if (options.defaultMaterialFile.empty()) return result;

    const fs::path path = options.defaultMaterialFile.is_absolute()
        ? options.defaultMaterialFile : objDirectory / options.defaultMaterialFile;
    std::ifstream stream(path);
    if (!stream) {
        warnings += "Cannot open default material file: " + path.u8string()
                  + "; using built-in gray material.\n";
        return result;
    }
    std::vector<tinyobj::material_t> materials;
    std::map<std::string, int> names;
    std::string error;
    tinyobj::LoadMtl(&names, &materials, &stream, &warnings, &error);
    if (!error.empty()) warnings += error;
    if (materials.empty() || !error.empty()) {
        warnings += "Default MTL has no usable material; using built-in gray material.\n";
        return result;
    }
    return convertMaterial(materials.front(), path.parent_path());
}

struct SmoothKey {
    int position;
    unsigned int group;
    bool operator==(const SmoothKey& other) const noexcept {
        return position == other.position && group == other.group;
    }
};

struct SmoothHash {
    std::size_t operator()(const SmoothKey& key) const noexcept {
        const std::size_t a = std::hash<int>{}(key.position);
        const std::size_t b = std::hash<unsigned int>{}(key.group);
        return a ^ (b + std::size_t(0x9e3779b9) + (a << 6) + (a >> 2));
    }
};

struct PendingTriangle {
    objLoader::Triangle triangle;
    std::array<int, 3> positions;
    unsigned int smoothingGroup = 0;
    Eigen::Vector3d areaNormal;
};
} // namespace

objLoader::objLoader() { clear(); }

void objLoader::clear() {
    triangles_.clear();
    objects_.clear();
    materials_.assign(1, Material{});
    error_.clear();
    warnings_.clear();
}

bool objLoader::loadObj(const fs::path& filename) {
    return loadObj(filename, Options{});
}

bool objLoader::loadObj(const fs::path& filename, const Options& options) {
    clear();
    try {
        // filesystem::path + ifstream 支持 Windows 上含中文的文件路径。
        const fs::path path = fs::absolute(filename).lexically_normal();
        std::ifstream stream(path);
        if (!stream) {
            error_ = "Cannot open OBJ file: " + path.u8string();
            return false;
        }

        tinyobj::attrib_t attributes;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> sourceMaterials;
        MaterialReader materialReader(path.parent_path());
        // 最后两个 true：统一三角化、缺少顶点颜色时填白色。
        if (!tinyobj::LoadObj(&attributes, &shapes, &sourceMaterials,
                             &warnings_, &error_, &stream, &materialReader, true, true)) {
            if (error_.empty()) error_ = "Failed to parse OBJ: " + path.u8string();
            return false;
        }

        std::vector<Material> materials;
        materials.push_back(defaultMaterial(options, path.parent_path(), warnings_));
        for (std::size_t i = 0; i < sourceMaterials.size(); ++i) {
            materials.push_back(convertMaterial(sourceMaterials[i], materialReader.directories.at(i)));
        }

        std::vector<Object> objects;
        std::vector<PendingTriangle> pending;
        std::unordered_map<SmoothKey, Eigen::Vector3d, SmoothHash> smoothNormals;
        std::size_t degenerateCount = 0;
        bool ignoredPrimitives = false;
        for (const auto& shape : shapes) {
            ignoredPrimitives = ignoredPrimitives || !shape.lines.indices.empty()
                                || !shape.points.indices.empty();
            Object object;
            object.name = shape.name.empty() ? "default" : shape.name;
            const std::size_t objectIndex = objects.size();
            std::size_t offset = 0;
            for (std::size_t face = 0; face < shape.mesh.num_face_vertices.size(); ++face) {
                if (shape.mesh.num_face_vertices[face] != 3 || offset + 3 > shape.mesh.indices.size()) {
                    throw std::runtime_error("OBJ triangulation did not produce a valid triangle.");
                }
                PendingTriangle entry;
                entry.triangle.objectIndex = objectIndex;
                const int material = face < shape.mesh.material_ids.size()
                    ? shape.mesh.material_ids[face] : -1;
                if (material >= 0 && static_cast<std::size_t>(material) < sourceMaterials.size()) {
                    entry.triangle.materialIndex = static_cast<std::size_t>(material) + 1;
                }
                entry.smoothingGroup = face < shape.mesh.smoothing_group_ids.size()
                    ? shape.mesh.smoothing_group_ids[face] : 0;

                for (std::size_t corner = 0; corner < 3; ++corner) {
                    const tinyobj::index_t index = shape.mesh.indices[offset + corner];
                    Vertex& vertex = entry.triangle.vertices[corner];
                    entry.positions[corner] = index.vertex_index;
                    vertex.position = read3(attributes.vertices, index.vertex_index, "position");
                    if (index.normal_index >= 0) {
                        const Eigen::Vector3d normal = read3(attributes.normals, index.normal_index, "normal").cast<double>();
                        if (normal.squaredNorm() > 0.0) {
                            vertex.normal = normal.normalized().cast<float>();
                            vertex.normalFromFile = true;
                        }
                    }
                    if (index.texcoord_index >= 0) {
                        const std::size_t uvIndex = static_cast<std::size_t>(index.texcoord_index);
                        if (uvIndex >= attributes.texcoords.size() / 2) {
                            throw std::runtime_error("Out-of-range UV index.");
                        }
                        vertex.uv = Eigen::Vector2f(attributes.texcoords[2 * uvIndex],
                                                    attributes.texcoords[2 * uvIndex + 1]);
                        if (!vertex.uv.allFinite()) throw std::runtime_error("Non-finite UV value.");
                        vertex.hasUV = true;
                    }
                    if (static_cast<std::size_t>(index.vertex_index) < attributes.colors.size() / 3) {
                        vertex.color = read3(attributes.colors, index.vertex_index, "color");
                    }
                }
                offset += 3;
                const auto& vertices = entry.triangle.vertices;
                const Eigen::Vector3d e1 = vertices[1].position.cast<double>() - vertices[0].position.cast<double>();
                const Eigen::Vector3d e2 = vertices[2].position.cast<double>() - vertices[0].position.cast<double>();
                entry.areaNormal = e1.cross(e2);
                if (!entry.areaNormal.allFinite() || entry.areaNormal.squaredNorm() == 0.0) {
                    ++degenerateCount;
                    continue;
                }
                if (entry.smoothingGroup != 0) {
                    for (int position : entry.positions) {
                        const SmoothKey key{position, entry.smoothingGroup};
                        auto found = smoothNormals.try_emplace(key, Eigen::Vector3d::Zero());
                        found.first->second += entry.areaNormal;
                    }
                }
                object.triangleIndices.push_back(pending.size());
                pending.push_back(std::move(entry));
            }
            objects.push_back(std::move(object));
        }
        if (pending.empty()) {
            error_ = "OBJ contains no non-degenerate triangle surfaces: " + path.u8string();
            return false;
        }

        std::vector<Triangle> triangles;
        triangles.reserve(pending.size());
        for (auto& entry : pending) {
            for (std::size_t corner = 0; corner < 3; ++corner) {
                Vertex& vertex = entry.triangle.vertices[corner];
                if (vertex.normalFromFile) continue;
                Eigen::Vector3d normal = entry.areaNormal;
                if (entry.smoothingGroup != 0) {
                    const auto found = smoothNormals.find({entry.positions[corner], entry.smoothingGroup});
                    if (found != smoothNormals.end() && found->second.squaredNorm() > 0.0) {
                        normal = found->second;
                    }
                }
                vertex.normal = normal.normalized().cast<float>();
            }
            triangles.push_back(std::move(entry.triangle));
        }
        if (degenerateCount != 0) {
            warnings_ += "Skipped " + std::to_string(degenerateCount) + " zero-area triangles.\n";
        }
        if (ignoredPrimitives) warnings_ += "Ignored line/point primitives; output contains triangle surfaces only.\n";
        triangles_ = std::move(triangles);
        materials_ = std::move(materials);
        objects_ = std::move(objects);
        return true;
    } catch (const std::exception& exception) {
        error_ = exception.what();
        return false;
    }
}
