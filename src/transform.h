#include <Eigen/core>
#include <vector>
#include "objLoader.h"
#include "parameter.h"



class Transform {
    private:
    Eigen::Matrix4f modelMatrix;
    Eigen::Matrix4f viewMatrix;
    Eigen::Matrix4f projectionMatrix;


    public:
    static Eigen::Vector3f transform(const Eigen::Vector3f& point);//封装了各种矩阵，掌管整个流程
    static Eigen::Matrix4f getModelMatrix(std::vector<float>&scales,std::vector<float>&translates,Eigen::Vector3f axis,float theta);//先暂时不管输入参数
    static Eigen::Matrix4f getViewMatrix(std::vector<float>&translates,Eigen::Vector3f axis,float theta);
    static Eigen::Matrix4f getProjectionMatrix();
    static Eigen::Vector3f getNDC(const Eigen::Vector4f& clipSpacePoint);
    static Eigen::Matrix4f getViewportMatrix();

    //我说白了有点乱了，先保持吧
};
