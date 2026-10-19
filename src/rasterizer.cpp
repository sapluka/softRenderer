#include "rasterizer.h"
#include "parameter.h"
#include <Eigen/Core>
#include <Eigen/core>
#include <vector>


void rasterizer::initRasterizer() {
    // 初始化颜色缓冲区为黑色
    for (int i = 0; i < SCREEN_HEIGHT; ++i) {
        for (int j = 0; j < SCREEN_WIDTH; ++j) {
            frameBuffer[i][j] = Eigen::Array3f(0.0f, 0.0f, 0.0f);
            zBuffer[i][j] = std::numeric_limits<float>::infinity(); 
            normalBuffer[i][j] = Eigen::Array3f(0.0f, 0.0f, 0.0f);
        }
    }
}



void rasterizer::setGBuffers(const objLoader& model)
{
     float _y = std::tan(fov_y / 2.0f) * z_near;
     float _x = _y * aspect_ratio;
     
     float offset_x = _x/(SCREEN_WIDTH/2.0f);
     float offset_y = _y/(SCREEN_HEIGHT/2.0f);

     Eigen::Vector3f origin (EYE_POS_X,EYE_POS_Y,EYE_POS_Z);
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
                    IntersectedTriangles.push_back(triangle);
                    //我需要填入zBuffer里面的值到底是打到的点的z值还是三角形的插值？
                    float alpha ;
                    float beta ;
                    float gamma ;

                    Eigen::Vector3f P = r.getIntersectionPoint();
                    Eigen::Vector3f A = point0;
                    Eigen::Vector3f B = point1;
                    Eigen::Vector3f C = point2;
                    Eigen::Vector3f e_1 = B - A;
                    Eigen::Vector3f e_2 = C - A;
                    Eigen::Vector3f q = P - A;

                    float a = e_1.dot(e_1);
                    float b = e_1.dot(e_2);
                    float c = e_2.dot(e_2);
                    float u = e_1.dot(q);
                    float v = e_2.dot(q);

                    float denom = a * c - b * b;

                    beta = (c * u - b * v) / denom;
                    gamma = (a * v - b * u) / denom;
                    alpha = 1.0f - beta - gamma;
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

void rasterizer::rasterize() {
    std::vector<float>model_scales = {1.0f, 1.0f, 1.0f};
    std::vector<float>model_translates = {0.0f, 0.0f, 0.0f};
    Eigen::Vector3f model_rotation_axis(0.0f, 1.0f, 0.0f);
    float model_rotation_angle = 0.0f; // 旋转角度，单位为弧度
    Eigen::Matrix4f model = Transform::getModelMatrix(model_scales, model_translates, model_rotation_axis, model_rotation_angle);
    Eigen::Matrix4f view = Transform::getViewMatrix(model_translates, model_rotation_axis, model_rotation_angle);
    Eigen::Matrix4f projection = Transform::getProjectionMatrix();
    Eigen::Matrix4f viewport = Transform::getViewportMatrix();
    for(const auto& triangle : IntersectedTriangles) {

        std::vector<Eigen::Vector3f>screenTriangle;

        for(const auto& vertex : triangle.vertices) {
        //对点进行MVP变换
        Eigen::Vector4f worldPos(vertex.position.x(), vertex.position.y(), vertex.position.z(), 1.0f);
        Eigen::Vector4f clipSpacePos = projection * view * model * worldPos;
        Eigen::Vector3f ndcPos = Transform::getNDC(clipSpacePos);
        Eigen::Vector4f temp = viewport * Eigen::Vector4f(ndcPos.x(), ndcPos.y(), ndcPos.z(), 1.0f);
        Eigen::Vector3f screenPos(temp.x() , temp.y() , temp.z() );
        screenTriangle.push_back(screenPos);

        //计算着色
        }
        float min_x = std::min(screenTriangle[0].x(), std::min(screenTriangle[1].x(), screenTriangle[2].x()));
        float max_x = std::max(screenTriangle[0].x(), std::max(screenTriangle[1].x(), screenTriangle[2].x()));
        float min_y = std::min(screenTriangle[0].y(), std::min(screenTriangle[1].y(), screenTriangle[2].y()));
        float max_y = std::max(screenTriangle[0].y(), std::max(screenTriangle[1].y(), screenTriangle[2].y()));

    }
}
