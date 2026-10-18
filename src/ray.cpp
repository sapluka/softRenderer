#include "ray.h"


void ray::findIntersectionPoint(Eigen::Vector3f& point0, Eigen::Vector3f& point1, Eigen::Vector3f& point2) {
    // 计算三角形的法向量
    Eigen::Vector3f edge1 = point1 - point0;
    Eigen::Vector3f edge2 = point2 - point0;
    Eigen::Vector3f normal = edge1.cross(edge2).normalized();

    // 计算射线与三角形所在平面的交点
    float denom = normal.dot(direction);
    if (std::abs(denom) < 1e-6) {
        intersect = false; // 射线与平面平行
        return;
    }

    float t = (point0 - origin).dot(normal) / denom;
    if (t < 0) {
        intersect = false; // 交点在射线的反方向
        return;
    }

    intersectionPoint = origin + t * direction;

    // 检查交点是否在三角形内
    Eigen::Vector3f c0 = intersectionPoint - point0;
    Eigen::Vector3f c1 = intersectionPoint - point1;
    Eigen::Vector3f c2 = intersectionPoint - point2;

    if (normal.dot(edge1.cross(c0)) >= 0 &&
        normal.dot((point2 - point1).cross(c1)) >= 0 &&
        normal.dot((point0 - point2).cross(c2)) >= 0) {
        intersect = true; // 交点在三角形内
    } else {
        intersect = false; // 交点在三角形外
    }
}
