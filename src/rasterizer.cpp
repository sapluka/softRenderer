#include "rasterizer.h"
#include "objLoader.h"
#include "parameter.h"
#include <Eigen/Core>
#include <Eigen/core>
#include <unordered_set>
#include <vector>


void rasterizer::initRasterizer() {
    frameBuffer.assign(SCREEN_HEIGHT,std::vector<Eigen::Vector3f>(SCREEN_WIDTH,Eigen::Vector3f::Zero()));
    zBuffer.assign(SCREEN_HEIGHT,std::vector<float>(SCREEN_WIDTH,-std::numeric_limits<float>::infinity()));
    normalBuffer.assign(SCREEN_HEIGHT,std::vector<Eigen::Vector3f>(SCREEN_WIDTH,Eigen::Vector3f::Zero()));
}



void rasterizer::setGBuffers(const objLoader& model,const Eigen::Vector3f& origin)
{
    //坏菜了这个_x,_y好像是硬编码，当我们摄像机位置改变时就不对了
    //应该根据摄像机的位置和摄像机的direction来计算_x,_y,offset_x,offset_y
    IntersectedTriangles.clear(); // 清空已处理的三角形列表
    std::unordered_set<const objLoader::Triangle*> added;

     float _y = std::tan(fov_y / 2.0f) * z_near;
     float _x = _y * aspect_ratio;
     
     float offset_x = _x/(SCREEN_WIDTH/2.0f);
     float offset_y = _y/(SCREEN_HEIGHT/2.0f);
    //这里得把SCREEN_WIDTH和SCREEN_HEIGHT改成window.getWidth()和window.getHeight()
     for(int i = 0; i < SCREEN_HEIGHT; ++i) {
        for (int j = 0; j < SCREEN_WIDTH; ++j) {
            float x = -_x + j * offset_x;
            float y = _y - i * offset_y;
            Eigen::Vector3f direction(x, y, -z_near);
            direction.normalize();
            ray r(origin, direction);
            // 在这里可以使用射线 r 进行进一步的处理，例如与场景中的物体进行交互
            for (const auto& triangle : model.triangles())
            {
                // Loader 已经三角化，并为每个角点配好位置、法线和 UV。
                Eigen::Vector3f point0 = triangle.vertices[0].position;
                Eigen::Vector3f point1 = triangle.vertices[1].position;
                Eigen::Vector3f point2 = triangle.vertices[2].position;
                r.findIntersectionPoint(point0,point1,point2);
                if(r.getIntersect())
                {   
                    if(added.insert(&triangle).second)
                    {
                        IntersectedTriangles.push_back(triangle);
                    }
                    //我需要填入zBuffer里面的值到底是打到的点的z值还是三角形的插值？
                    float alpha, beta, gamma;
                    //这里我们把重心坐标的计算封装为一个函数
                    Transform::computeBarycentric2D(r.getIntersectionPoint(), point0, point1, point2, alpha, beta, gamma);
                    zBuffer[i][j] = alpha * point0.z() + beta * point1.z() + gamma * point2.z();
                    normalBuffer[i][j] = (alpha * triangle.vertices[0].normal
                                       + beta * triangle.vertices[1].normal
                                       + gamma * triangle.vertices[2].normal).normalized().array();
                    //以上是关于插值的一些推导，关键是把β和γ的二元一次方程写出来，消元就能拿到公式了
                    //除此之外你可以直接使用eigen库来求解未知数
                    /*
                    Matrix M;已知数矩阵
                    vector3f b;方程右边的数

                    vector3f w = M.colPivHouseholderQr().solve(b);就能得到解了
                    */
                }
            }
        }
    }
}

void rasterizer::rasterize(const objLoader& model, const Eigen::Vector3f& eye,const Eigen::Vector3f& target) {

    float _y = std::tan(fov_y / 2.0f) * z_near;
    float _x = _y * aspect_ratio;
     //如果我们能把offset,_x,_y都固定就好了，但是有点难，反正是用不了constexpr，要是全局变量或者私有数据还行
    float offset = _x/(SCREEN_WIDTH/2.0f);
    
    for(const auto& triangle : IntersectedTriangles) {

        std::vector<Eigen::Vector3f>screenTriangle;

        for(const auto& vertex : triangle.vertices) {
        //对点进行MVP变换
        Eigen::Vector3f screenPos = Transform::transform(vertex.position,eye,target);
        screenTriangle.push_back(screenPos);

        //计算着色
        }
        // 计算屏幕三角形的包围盒(实际坐标)
        float min_x = std::min(screenTriangle[0].x(), std::min(screenTriangle[1].x(), screenTriangle[2].x()));
        float max_x = std::max(screenTriangle[0].x(), std::max(screenTriangle[1].x(), screenTriangle[2].x()));
        float min_y = std::min(screenTriangle[0].y(), std::min(screenTriangle[1].y(), screenTriangle[2].y()));
        float max_y = std::max(screenTriangle[0].y(), std::max(screenTriangle[1].y(), screenTriangle[2].y()));

        // 计算屏幕三角形的包围盒(像素索引)
        int min_x_index = static_cast<int>(min_x/offset);
        int max_x_index = static_cast<int>(max_x/offset);
        int min_y_index = static_cast<int>(min_y/offset);
        int max_y_index = static_cast<int>(max_y/offset);

        const auto& material = model.materialFor(triangle);
        for(int i = min_y_index; i <= max_y_index; ++i) {
            for(int j = min_x_index; j <= max_x_index; ++j) {
                Eigen::Vector3f P(j + offset/2.0f, i + offset/2.0f, 0.0f);
                float alpha, beta, gamma;
                Transform::computeBarycentric2D(P, screenTriangle[0], screenTriangle[1], screenTriangle[2], alpha, beta, gamma);
                if(alpha >= 0 && beta >= 0 && gamma >= 0) {
                    float z_interpolated = alpha * screenTriangle[0].z() + beta * screenTriangle[1].z() + gamma * screenTriangle[2].z();
                    
                    if(z_interpolated > zBuffer[i][j]) {
                        zBuffer[i][j] = z_interpolated;
                        frameBuffer[i][j] = shadeModel.computeShading(material, eye, P, normalBuffer[i][j]);
                    }
                }
            }
        }
    }
}

void rasterizer::draw() {
    // 这里可以使用 OpenGL 或其他图形库将 frameBuffer 显示在屏幕上
    // 例如，使用 OpenGL 的 glDrawPixels 或者将 frameBuffer 转换为纹理并渲染到屏幕上
}

