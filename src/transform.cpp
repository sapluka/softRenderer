#include "transform.h"
#include <Eigen/core>
#include <cmath>

Eigen::Matrix4f Transform::getModelMatrix(std::vector<float>&scales,std::vector<float>&translates,Eigen::Vector3f axis,float theta)
{
    Eigen::Matrix4f scale;
    scale<<scales[0],0.0f,0.0f,0.0f,
           0.0f,scales[1],0.0f,0.0f,
           0.0f,0.0f,scales[2],0.0f,
           0.0f,0.0f,0.0f,1.0f;

    Eigen::Matrix4f rotation;
    rotation<<cos(theta)+axis.x()*axis.x()*(1-cos(theta)),axis.x()*axis.y()*(1-cos(theta))-axis.z()*sin(theta),axis.x()*axis.z()*(1-cos(theta))+axis.y()*sin(theta),0.0f,
    axis.y()*axis.x()*(1-cos(theta))+axis.z()*sin(theta),cos(theta)+axis.y()*axis.y()*(1-cos(theta)),axis.y()*axis.z()*(1-cos(theta))-axis.x()*sin(theta),0.0f,
    axis.z()*axis.x()*(1-cos(theta))-axis.y()*sin(theta),axis.z()*axis.y()*(1-cos(theta))+axis.x()*sin(theta),cos(theta)+axis.z()*axis.z()*(1-cos(theta)),0.0f,
    0.0f,0.0f,0.0f,1.0f;

    Eigen::Matrix4f translate;
    translate<<1.0f,0.0f,0.0f,translates[0],
               0.0f,1.0f,0.0f,translates[1],
               0.0f,0.0f,1.0f,translates[2],
               0.0f,0.0f,0.0f,1.0f;

    return translate*rotation*scale;
               
}

Eigen::Matrix4f Transform::getViewMatrix(const Eigen::Vector3f& eye, const Eigen::Vector3f& target)
{
    // 这里可以使用相机的lookAt函数来计算视图矩阵,不会的去问问ai
    Eigen::Vector3f up(0.0f, 1.0f, 0.0f); // 假设世界空间的上方向为Y轴正方向
    Eigen::Vector3f b = (eye - target).normalized(); // 计算相机的后方向
    Eigen::Vector3f r = up.cross(b).normalized(); // 计算相机的右方向
    Eigen::Vector3f u = b.cross(r).normalized(); // 计算相机的上方向

    Eigen::Matrix4f view;
    view << r.x(), r.y(), r.z(), -r.dot(eye),
            u.x(), u.y(), u.z(), -u.dot(eye),
            b.x(), b.y(), b.z(), -b.dot(eye),
            0.0f, 0.0f, 0.0f, 1.0f;
    return view;
}

Eigen::Matrix4f Transform::getProjectionMatrix()
{
    float f = 1.0f / tan(fov_y / 2.0f);

    Eigen::Matrix4f projection;
    projection<<-f/aspect_ratio,0.0f,0.0f,0.0f,
                0.0f,-f,0.0f,0.0f,
                0.0f,0.0f,-(z_near+z_far)/(z_far-z_near),2.0f*z_near*z_far/(z_far-z_near),
                0.0f,0.0f,-1.0f,0.0f;
    return projection;
    //这里可能会出现z的正负定义问题，明天检查一下(已检查完毕)
    //注意这一步得到的仍然是clip space的坐标，之后还需要进行透视除法得到NDC坐标
}

Eigen::Vector4f Transform::getNDC(const Eigen::Vector4f& clipSpacePoint)
{
    Eigen::Vector4f ndc;
    ndc.x() = clipSpacePoint.x() / clipSpacePoint.w();
    ndc.y() = clipSpacePoint.y() / clipSpacePoint.w();
    ndc.z() = clipSpacePoint.z() / clipSpacePoint.w();
    ndc.w() = 1.0f;
    return ndc;
}

Eigen::Matrix4f Transform::getViewportMatrix()
{
    Eigen::Matrix4f viewport;
    viewport<<SCREEN_WIDTH/2.0f,0.0f,0.0f,SCREEN_WIDTH/2.0f,
              0.0f,-SCREEN_HEIGHT/2.0f,0.0f,SCREEN_HEIGHT/2.0f,
              0.0f,0.0f,1.0f,0.0f,
              0.0f,0.0f,0.0f,1.0f;
    return viewport;
}

void Transform::computeBarycentric2D(const Eigen::Vector3f& p, const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c, float& alpha, float& beta, float& gamma) {
                    Eigen::Vector3f e_1 = b - a;
                    Eigen::Vector3f e_2 = c - a;
                    Eigen::Vector3f q = p - a;

                    float x = e_1.dot(e_1);
                    float y = e_1.dot(e_2);
                    float z = e_2.dot(e_2);
                    float u = e_1.dot(q);
                    float v = e_2.dot(q);

                    float denom = x * z - y * y;

                    beta = (z * u - y * v) / denom;
                    gamma = (x * v - y * u) / denom;
                    alpha = 1.0f - beta - gamma;

}

Eigen::Vector3f transform(const Eigen::Vector3f& point, const Eigen::Vector3f& eye,const Eigen::Vector3f& target)
{
    Eigen::Vector4f worldPos(point.x(), point.y(), point.z(), 1.0f);
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f view = Transform::getViewMatrix(eye, target);
    Eigen::Matrix4f projection = Transform::getProjectionMatrix();
    Eigen::Vector4f clipSpacePos = projection * view * model * worldPos;
    Eigen::Vector4f ndcPos = Transform::getNDC(clipSpacePos);
    Eigen::Matrix4f viewport = Transform::getViewportMatrix();
    Eigen::Vector4f screenPos = viewport * ndcPos;
    return Eigen::Vector3f(screenPos.x(), screenPos.y(), screenPos.z());
}
