#include <Eigen/Core>



class camera{
    private:
    Eigen::Vector3f position ;
    float fov_y;
    float aspect_ratio;
    public:
    camera() = default;
    void initCamera();//初始化位置，FOV等等

};