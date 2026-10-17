#include <Eigen/core>

class Transform {
    private:

    public:
    static Eigen::Vector3f transform(const Eigen::Vector3f& point);//封装了各种矩阵，掌管整个流程
    static Eigen::Matrix4f getModelMatrix();//先暂时不管输入参数
    static Eigen::Matrix4f getViewMatrix();
    static Eigen::Matrix4f getProjectionMatrix();
    static Eigen::Matrix4f getViewportMatrix();

    
};
