#include "objLoader.h"
#include "rasterizer.h"
#include "shadeModel.h"
#include "scene.h"
#include <Eigen/Core>
#include <Eigen/core>
#include <algorithm>
#include <iostream>
int main()
{
    std::cout << "Build configuration: " << RENDERER_BUILD_CONFIG << std::endl;
    objLoader mod;
    if (!mod.loadObj(std::filesystem::path(RENDERER_ASSET_DIR) / "bunny/bunny.obj")) {
        std::cerr << mod.error() << std::endl;
        return 1;
    }

    std::vector<float>points_x;
    std::vector<float>points_y;
    std::vector<float>points_z;
    

    for(auto& tri : mod.triangles())
    {
        for(auto vertex:tri.vertices)
        {
            points_x.push_back(vertex.position.x());
            points_y.push_back(vertex.position.y());
            points_z.push_back(vertex.position.z());
        }

    }
    
    auto [min_x,max_x] = std::minmax_element(points_x.begin(),points_x.end());
    auto [min_y,max_y] = std::minmax_element(points_y.begin(),points_y.end());
    auto [min_z,max_z] = std::minmax_element(points_z.begin(),points_z.end());

    std::cout << "min_x: " << *(min_x) << " max_x: " << *(max_x) << std::endl;
    std::cout << "min_y: " << *(min_y) << " max_y: " << *(max_y) << std::endl;
    std::cout << "min_z: " << *(min_z) << " max_z: " << *(max_z) << std::endl;

    
    Eigen::Vector3f point_min(*(min_x),*(min_y),*(min_z));
    Eigen::Vector3f point_max(*(max_x),*(max_y),*(max_z));
    Eigen::Vector3f target = (point_min+point_max)*0.5f;

    light lig;
    shadeModel shademodel = shadeModel(lig);
    rasterizer ras = rasterizer(shademodel);
    ras.initRasterizer();
     
    camera cam(point_min, point_max);

    scene sce = scene(mod,ras,cam);

    sce.init();
    sce.render();

    sce.releaseResources();
}
