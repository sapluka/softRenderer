#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <Eigen/core>


static std::vector<Eigen::Vector3f> vertices;
static std::vector<Eigen::Vector3f> normals;
static std::vector<Eigen::Vector2f> uvs;
static std::vector<std::vector<float>> faces;

class ObjLoader {

    private:
    

    public:
        ObjLoader() = default;

        static void loadObj(const std::string& filename) {
            std::ifstream file(filename);
            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << filename << std::endl;
                return;
            }

            std::string line;
            while (std::getline(file, line)) {
                parseObjLine(line);
            }
        }

        static void parseObjLine(const std::string& line) {
            std::istringstream iss(line);
            std::string prefix;
            iss >> prefix;

            if (prefix == "v") {
                Eigen::Vector3f vertex;
                iss >> vertex[0] >> vertex[1] >> vertex[2];
                vertices.push_back(vertex);
            } else if (prefix == "vn") {
                Eigen::Vector3f normal;
                iss >> normal[0] >> normal[1] >> normal[2];
                normals.push_back(normal);
            } else if (prefix == "vt") {
                Eigen::Vector2f uv;
                iss >> uv[0] >> uv[1];
                uvs.push_back(uv);
            } else if (prefix == "f") {
                std::vector<float> face;
                std::string vertexStr;
                while (iss >> vertexStr) {
                    std::istringstream viss(vertexStr);
                    std::string indexStr;
                    while (std::getline(viss, indexStr, '/')) {
                        face.push_back(std::stof(indexStr));
                    }
                }
                faces.push_back(face);
            }
        }

        //很显然你还需要把.obj拖入窗口的事件给写成函数
};
