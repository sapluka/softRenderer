#include "BVH.h"
#include "objLoader.h"
#include <Eigen/Core>
#include <algorithm>
#include <cstdlib>
#include <limits>
#include <numeric>
#include <vector>

BVH::BVH()
{
    root = nullptr; 
}

BVH::~BVH()
{
    clear(root);
}

void BVH::clear(BVHNode* node)
{
    if(node == nullptr)
    {
        return;
    }
    clear(node->leftChild);
    clear(node->rightChild);
    delete node;
}

void BVH::build(objLoader& model)
{
    clear(root);
    root = nullptr;
    calculateCenterCoordinates(model);
    if(center_coordinates.empty())
    {
        return;
    }

    // 排序的是原始三角形编号，不改变 model 中三角形的顺序。
    std::vector<int> array(center_coordinates.size());
    std::iota(array.begin(), array.end(), 0); // 填入 0、1、2……这些原始 triangle index。
    array = sortTriangleIndices(array);

    root = new BVHNode();
    divide(root, model, array);
}

std::vector<int> BVH::sortTriangleIndices(std::vector<int> array) const
{
    if(array.size() <= 1)
    {
        return array;
    }

    // 只评估当前组的重心，不能每一层都用整个模型的分布范围。
    Eigen::Vector3f minCenter = center_coordinates[array[0]];
    Eigen::Vector3f maxCenter = center_coordinates[array[0]];
    for(int index : array)
    {
        minCenter = minCenter.cwiseMin(center_coordinates[index]);
        maxCenter = maxCenter.cwiseMax(center_coordinates[index]);
    }
    Eigen::Vector3f extent = maxCenter - minCenter;
    int axis = 0; // 0、1、2 分别表示 x、y、z；范围相同时优先选前面的轴。
    for(int i = 1; i < 3; ++i)
    {
        if(extent[i] > extent[axis])
        {
            axis = i;
        }
    }

    std::sort(array.begin(), array.end(), [this, axis](int left, int right) {
        float leftCoordinate = center_coordinates[left][axis];
        float rightCoordinate = center_coordinates[right][axis];
        if(leftCoordinate == rightCoordinate)
        {
            return left < right;
        }
        return leftCoordinate < rightCoordinate;
    });
    return array;
}

Eigen::Vector3f BVH::findMinPoint(objLoader& model , const std::vector<int>& array)
{
    Eigen::Vector3f minPoint = model.triangles()[array[0]].vertices[0].position;
    for(unsigned int i = 0; i < array.size() ; ++i)
    {
        for(unsigned int j = 0; j < 3 ; ++j)
        {
            Eigen::Vector3f currentPoint = model.triangles()[array[i]].vertices[j].position;
            if(currentPoint.x() < minPoint.x())
            {
                minPoint.x() = currentPoint.x();
            }
            if(currentPoint.y() < minPoint.y())
            {
                minPoint.y() = currentPoint.y();
            }
            if(currentPoint.z() < minPoint.z())
            {
                minPoint.z() = currentPoint.z();
            }
        }

    }
    return minPoint;
}

Eigen::Vector3f BVH::findMaxPoint(objLoader& model , const std::vector<int>& array)
{
    Eigen::Vector3f maxPoint = model.triangles()[array[0]].vertices[0].position;
    for(unsigned int i = 0; i < array.size() ; ++i)
    {
        for(unsigned int j = 0; j < 3 ; ++j)
        {
            Eigen::Vector3f currentPoint = model.triangles()[array[i]].vertices[j].position;
            if(currentPoint.x() > maxPoint.x())
            {
                maxPoint.x() = currentPoint.x();
            }
            if(currentPoint.y() > maxPoint.y())
            {
                maxPoint.y() = currentPoint.y();
            }
            if(currentPoint.z() > maxPoint.z())
            {
                maxPoint.z() = currentPoint.z();
            }
        }

    }
    return maxPoint;
}

void BVH::divide(BVHNode*node,objLoader& model,std::vector<int>array)
{
    if(node == nullptr || array.empty())
    {
        return;
    }

    // 叶子也需要完整的包围盒，不能只包住重心。
    node->minBounds = findMinPoint(model,array);
    node->maxBounds = findMaxPoint(model,array);
    if(array.size() == 1)
    {
        node->triangleIndex = array[0];
        node->isLeaf = true;
        return;
    }
    node->leftChild = new BVHNode();
    node->rightChild = new BVHNode();

    unsigned int middleNum = array.size()/2;
    std::vector<int>array_left;
    std::vector<int>array_right;
    for(unsigned int i = 0; i < middleNum ; ++ i )
    {
        array_left.push_back(array[i]);
    }

    for(unsigned int i = middleNum; i < array.size() ; ++i)
    {
        array_right.push_back(array[i]);
    }
    

    // 每个子组独立重新选轴、排序，再交给下一层按数量对半分。
    array_left = sortTriangleIndices(array_left);
    array_right = sortTriangleIndices(array_right);
    divide(node->leftChild,model, array_left);
    divide(node->rightChild,model, array_right);

}

Eigen::Vector3f BVH::findMinPoint(objLoader& model , int index)
{
    Eigen::Vector3f minPoint = model.triangles()[index].vertices[0].position;
    for(auto& vertex : model.triangles()[index].vertices)
    {
        if(vertex.position.x() < minPoint.x())
        {
            minPoint.x() = vertex.position.x();
        }
        if(vertex.position.y() < minPoint.y())
        {
            minPoint.y() = vertex.position.y();
        }
        if(vertex.position.z() < minPoint.z())
        {
            minPoint.z() = vertex.position.z();
        }
    }
    return minPoint;
}


Eigen::Vector3f BVH::findIntersection(ray& r, objLoader& model, BVHNode* node)
{
    r.setIntersect(false);
    if(node == nullptr)
    {
        return Eigen::Vector3f::Zero();
    }
    float t0 = 0.0f; // 射线参数的最小值
    float t1 = std::numeric_limits<float>::max(); // 射线参数的最大值
    if(node->isLeaf)
    {
        r.findIntersectionPoint(model.triangles()[node->triangleIndex]);
        if(r.getIntersect())
        {
            return r.getIntersectionPoint();
        }
        else
        {
            return Eigen::Vector3f(0.0f, 0.0f, 0.0f); // 没有交点
        }
    }
    else 
    {
        // 处理非叶子节点的情况
        Eigen::Vector3f minBounds = node->minBounds;
        Eigen::Vector3f maxBounds = node->maxBounds;
        for(int i = 0; i < 3; ++i) {
            if (r.getDirection()[i] != 0.0f) {
                float tMin = (minBounds[i] - r.getOrigin()[i]) / r.getDirection()[i];
                float tMax = (maxBounds[i] - r.getOrigin()[i]) / r.getDirection()[i];
                if (tMin > tMax) std::swap(tMin, tMax);
                if (tMin > t0) t0 = tMin;
                if (tMax < t1) t1 = tMax;
            }
            else
            {
                if (r.getOrigin()[i] < minBounds[i] || r.getOrigin()[i] > maxBounds[i]) {
                    return Eigen::Vector3f(0.0f, 0.0f, 0.0f); // 没有交点
                }
            }

        }
        if(t0 > t1)
        {
            return Eigen::Vector3f(0.0f, 0.0f, 0.0f); // 没有交点
        }
        else
        {
            ray leftRay = r;
            ray rightRay = r;

            Eigen::Vector3f leftPoint =
                findIntersection(leftRay, model, node->leftChild);
            Eigen::Vector3f rightPoint =
                findIntersection(rightRay, model, node->rightChild);

            bool leftHit = leftRay.getIntersect();
            bool rightHit = rightRay.getIntersect();

            if(!leftHit && !rightHit) {
                return Eigen::Vector3f::Zero();
            }

            if(leftHit && (!rightHit ||(leftPoint - r.getOrigin()).squaredNorm() <=(rightPoint - r.getOrigin()).squaredNorm())) {
                r = leftRay;
                return leftPoint;
            }

            r = rightRay;
            return rightPoint;
        }
    }
    
}

void BVH::calculateCenterCoordinates(objLoader& model)
{
    center_coordinates.clear();
    for (const auto& triangle : model.triangles()) {
        Eigen::Vector3f center = (triangle.vertices[0].position + triangle.vertices[1].position + triangle.vertices[2].position) / 3.0f;
        center_coordinates.push_back(center);
    }
}
