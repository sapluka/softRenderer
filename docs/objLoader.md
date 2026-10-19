# objLoader 使用说明

输出统一为三角形。每个三角形的三个角点都直接保存位置、法线、UV、顶点颜色；
OBJ 的独立索引、从 1 开始的编号和负索引都已在内部处理。

## 最小用法

```cpp
#include "objLoader.h"
#include <iostream>

int main() {
    objLoader model;
    if (!model.loadObj("D:/renderer/assets/monkey/monkey.obj")) {
        std::cerr << model.error() << '\n';
        return 1;
    }
    if (!model.warnings().empty()) {
        std::cerr << model.warnings(); // 有警告仍可能加载成功。
    }

    std::cout << "三角形数量：" << model.triangles().size() << '\n';
    for (const auto& triangle : model.triangles()) {
        const auto& a = triangle.vertices[0];
        const auto& b = triangle.vertices[1];
        const auto& c = triangle.vertices[2];
        const auto& material = model.materialFor(triangle);

        // a.position、b.position、c.position：三个顶点的位置。
        // a.normal、b.normal、c.normal：各角点的单位法线。
        // a.uv、b.uv、c.uv：各角点的 UV。
        // material.diffuse：这个三角形的 RGB 漫反射率 Kd。
        std::cout << material.name << ": "
                  << a.position.transpose() << " / "
                  << b.position.transpose() << " / "
                  << c.position.transpose() << '\n';
    }
}
```

这个类现在是实例类，不再使用 `objLoader::loadObj(...)` 静态调用，
也不再创建头文件里的全局 `vertices`、`faces`、`normals`、`uvs`。
多个实例可以同时保存不同模型。

## 接入现有 setGBuffers

`rasterizer::setGBuffers` 已改为接收 `const objLoader&`，内部也已经改用新的角点数据。
窗口或程序入口准备好以后，可以这样调用：

```cpp
#include "objLoader.h"
#include "rasterizer.h"
#include <memory>

// model 已经成功加载。
auto raster = std::make_unique<rasterizer>();
raster->initRasterizer();
raster->setGBuffers(model);
```

这里使用堆分配，因为 rasterizer 内含几块较大的屏幕缓冲区。
这次接入只处理模型数据访问；现有渲染器的相机、可见性判断、着色和窗口输出仍由渲染代码负责。

## 数据接口

| 接口或字段 | 含义 |
| --- | --- |
| `triangles()` | 全部三角形，只读 `vector<Triangle>` |
| `triangle.vertices[0..2]` | 三个完整角点，内部不需要再查 OBJ 索引 |
| `vertex.position` | 文件中记录的位置，不自动居中或缩放 |
| `vertex.normal` | 可直接使用的单位法线 |
| `vertex.normalFromFile` | 法线是否来自有效的 `vn`；否则由 loader 生成 |
| `vertex.uv`、`vertex.hasUV` | UV 值，以及文件是否确实提供了这个角点的 UV |
| `triangle.hasUVs()` | 三个角点是否都有 UV |
| `vertex.color` | 顶点颜色；没有提供时为白色 `(1,1,1)` |
| `materialFor(triangle)` | 获取这个三角形的材质，包括默认材质 |
| `materials()` | 材质列表，索引 0 始终是默认材质 |
| `objects()` | 命名 object/group 的片段及其三角形编号 |
| `objectFor(triangle)` | 获取这个三角形所属的 object/group |
| `error()` | 最近一次加载失败的原因 |
| `warnings()` | 缺失材质、跳过退化面等可恢复问题 |
| `clear()` | 清空模型和诊断信息，保留内置默认材质 |

每次 `loadObj` 都替换旧模型；失败返回 `false`，此时没有三角形或对象，
不会意外继续使用旧模型。重新加载或清空后，不要继续使用上一次获得的引用。

为方便使用，角点直接展开存储，不要求使用者管理去重后的顶点或属性索引；
相比共享顶点的紧凑表示，它会多占用一些内存。

## 材质与贴图

材质参数与 MTL 的对应关系：

| 字段 | MTL |
| --- | --- |
| `ambient` | `Ka` |
| `diffuse` | `Kd` |
| `specular` | `Ks` |
| `emission` | `Ke` |
| `shininess` | `Ns` |
| `opacity` | `d`，1 为不透明；也由 tinyobjloader 处理 `Tr` |
| `ior` | `Ni` |
| `illuminationModel` | `illum` |
| `roughness`、`metallic` | PBR 扩展 `Pr`、`Pm` |

`diffuseTexture`、`specularTexture`、`emissionTexture`、`normalTexture`、
`bumpTexture`、`opacityTexture` 分别保存对应贴图的路径和常用选项。
例如：

```cpp
const auto& material = model.materialFor(triangle);
if (triangle.hasUVs() && !material.diffuseTexture.path.empty()) {
    const auto& texture = material.diffuseTexture;
    // texture.path 是解析后的图片路径，不是已经加载到内存的图片。
    // 应用 texture.scale、texture.offset 后，再由纹理采样代码使用 UV。
    // texture.clamp 对应 -clamp，texture.strength 对应 bump 的 -bm。
}
```

Loader 解析材质参数与贴图引用，不解码图片、不采样纹理，也不计算最终像素颜色。
正常加载的 MTL 参数沿用 tinyobjloader 的解析值与缺省值；只有未关联到可用材质的面才使用默认材质。

默认材质的 `Kd=(0.8,0.8,0.8)`、`Ka=(0.1,0.1,0.1)`、`Ks=(0,0,0)`。
没有 `mtllib`、MTL 缺失或为空、`usemtl` 名字不存在时，面都会关联到默认材质。
也可以指定自己的默认 MTL，使用其中第一个材质：

```cpp
objLoader::Options options;
options.defaultMaterialFile = "fallback.mtl"; // 相对于 OBJ 所在目录。
if (!model.loadObj("D:/renderer/assets/monkey/monkey.obj", options)) {
    std::cerr << model.error() << '\n';
}
```

若自定义默认 MTL 也无法读取，则使用内置灰色，并报告警告。

## 按对象遍历

```cpp
for (const auto& object : model.objects()) {
    std::cout << object.name << '\n';
    for (std::size_t index : object.triangleIndices) {
        const auto& triangle = model.triangles()[index];
        const auto& material = model.materialFor(triangle);
        // 每个三角形可以使用不同材质。
    }
}
```

这里的 Object 对应 tinyobjloader 输出的命名 object/group 片段，
不保证与 Blender 场景对象一一对应；无名片段显示为 `default`。

## 支持范围和路径规则

- 支持 `v`、`v/vt`、`v//vn`、`v/vt/vn`，正索引和负索引。
- 三角形、四边形、一般简单平面多边形统一三角化；使用 Earcut 处理凹多边形。
- 保留独立的法线和 UV，所以硬边和 UV 接缝不会被错误合并。
- 缺少法线或法线为零时生成法线；`s off` 使用面法线，平滑组使用面积加权的共享法线。
- 缺少 UV 时值为零且 `hasUV=false`，不会伪造纹理展开。
- 支持常见 RGB MTL 参数、多个材质、同一行的多个 MTL 文件和顶点 RGB 扩展。
- 零面积三角形被跳过并警告；只含空面或退化面的模型加载失败。
- 输出面网格。点、线不会变成三角形；NURBS/自由曲面不属于此接口的支持范围。
- 不保证修复自交或非平面的无效多边形；此类模型应先在建模软件中修复。
- OBJ 相对路径以程序工作目录为基准；`mtllib` 以 OBJ 目录为基准；贴图以各自 MTL 目录为基准。
- 纹理路径解析为绝对路径，但不保证引用的图片实际存在。
- 中文 OBJ/MTL 路径支持 `std::filesystem::u8path(u8"中文路径")`；文件内容和名字按 UTF-8 处理。

项目的 `test.obj` 引用了不存在的 `未命名.mtl`，因此会成功读取几何、报告警告并使用默认材质。
可以修正资产中的引用，或将 `Options.defaultMaterialFile` 设为同目录的 `test.mtl`。

## 验证

项目现在显式指定 C++17，并提供根目录 `.clangd`：读取 `build` 中的编译数据库，
按当前工作区的 WinLibs/MinGW 工具链解析 `src` 和 `tests`。
这避免 Windows 版 clangd 自动选择 MSVC/C++14，导致 `std::filesystem` 等接口被误报为不存在。
如果以后改用其他工具链，应同步调整 `.clangd` 中的 target。

在项目根目录执行：

```text
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
```

测试包含五份现有模型，以及独立索引、负索引、凹多边形及其绕序、法线生成、
材质切换、贴图相对路径、中文路径、顶点颜色、默认材质、错误输入和重复加载。

还可以使用编辑器实际安装的 clangd 独立检查代码诊断，例如在 PowerShell 中：

```powershell
& 'D:/clangd_21.1.8/bin/clangd.exe' --check=D:/renderer/src/objLoader.cpp --tweaks= --log=info
```

这里的 `--tweaks=` 仅跳过 clangd 的重构动作自测，不关闭编译诊断；
不需要把这个参数加入编辑器设置。项目没有屏蔽诊断，也不需要修改全局 VS Code 配置。
