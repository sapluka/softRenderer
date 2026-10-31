#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "objLoader.h"
#include "ray.h"
#include "transform.h"

//与光线求交的场景都用的到BVH吧？

struct BVHNode {
    Eigen::Vector3f minBounds; // AABB最小点
    Eigen::Vector3f maxBounds; // AABB最大点
    BVHNode* leftChild; // 左子节点
    BVHNode* rightChild; // 右子节点
    int triangleIndex; // 三角形索引（叶子节点使用）
    bool isLeaf; // 是否为叶子节点
    BVHNode() : leftChild(nullptr), rightChild(nullptr), triangleIndex(-1), isLeaf(false) {}
};

class BVH{
    private:
    BVHNode* root;
    std::vector<Eigen::Vector3f> center_coordinates; // 下标与原始 triangle index 一致
    Eigen::Vector3f findMinPoint(objLoader& model , const std::vector<int>& array);
    Eigen::Vector3f findMinPoint(objLoader& model , int index);
    Eigen::Vector3f findMaxPoint(objLoader& model , const std::vector<int>& array);
    void calculateCenterCoordinates(objLoader& model);
    std::vector<int> sortTriangleIndices(std::vector<int> array) const;
    void clear(BVHNode* node);
    

    public:
    BVH();
    ~BVH();
    BVH(const BVH&) = delete;
    BVH& operator=(const BVH&) = delete;

    void build(objLoader& model);
    BVHNode* getRoot() const { return root; }
    // 从 build 开始构建；divide 接收已排序的原始三角形编号，写入新节点。
    void divide(BVHNode* node,objLoader& model,std::vector<int>array);
    Eigen::Vector3f findIntersection(ray& r, objLoader& model, BVHNode* node);

};
