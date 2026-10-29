#include "shadeModel.h"

Eigen::Vector3f shadeModel::computeShading(const objLoader::Material& material, const Eigen::Vector3f& camera, const Eigen::Vector3f& point, const Eigen::Vector3f& normal , float visibility) const {
    // 计算光源方向
    Eigen::Vector3f lightDir = (sceneLight.getPosition() - point).normalized();
    
    // 计算视线方向
    Eigen::Vector3f viewDir = (camera - point).normalized();
    
    Eigen::Vector3f h = (lightDir + viewDir).normalized(); // 半程向量
    // 假设法线为单位向量，使用光源方向和视线方向进行简单的 Lambertian 漫反射计算
    //color 不需要归一化
    Eigen::Vector3f color_ambient = material.ambient.cwiseProduct(sceneLight.getColor()); // 假设使用第一个材质的环境光
    // 计算最终颜色，假设光源颜色为白色，强度为 1.0
    Eigen::Vector3f color_diffuse = material.diffuse.cwiseProduct(sceneLight.getColor()) * std::max(0.0f, lightDir.dot(normal)); 
    Eigen::Vector3f color_specular = material.specular.cwiseProduct(sceneLight.getColor()) * std::pow(std::max(0.0f, h.dot(normal)), material.shininess);
    Eigen::Vector3f finalColor = color_ambient + (color_diffuse + color_specular)*visibility;
    return finalColor;
}
