#include <vector>
#include <Eigen/core>
#include "objLoader.h"
#include "parameter.h"


class Scene {   
    private:
        int frameBuffer[SCREEN_HEIGHT][SCREEN_WIDTH]; // 颜色缓冲区
        float zBuffer[SCREEN_HEIGHT][SCREEN_WIDTH]; // 深度缓冲区
        std::vector<Eigen::Vector3f> normalBuffer; // 法线缓冲区        

        public:
        Scene() {};
        void init()
        {

        }

  
};