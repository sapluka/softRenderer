#include <vector>
#include <Eigen/core>

constexpr int SCREEN_WIDTH = 800;
constexpr int SCREEN_HEIGHT = 600;  
class Scene {   
    private:
        int frameBuffer[SCREEN_HEIGHT][SCREEN_WIDTH]; // 颜色缓冲区
        float zBuffer[SCREEN_HEIGHT][SCREEN_WIDTH]; // 深度缓冲区
        std::vector<Eigen::Vector3f> normalBuffer; // 法线缓冲区        

        public:
        Scene() {};
        void init();

    
};