
一个软渲染器

OBJ 读取和渲染数据接口见 [objLoader 使用说明](docs/objLoader.md)。

## 构建和运行

当前 Windows/MinGW 环境中，在项目根目录执行：

```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j 6
.\build\renderer.exe
```

逐像素求交的 Debug 构建非常慢。`RelWithDebInfo` 同时保留优化和调试符号。
未指定构建类型的新构建目录默认使用 `RelWithDebInfo`；已有目录保留缓存中的选择。
本项目使用的 MinGW Makefiles 必须在配置阶段指定 `CMAKE_BUILD_TYPE`，不能靠 `cmake --build build --config Release` 切换。
程序启动时打印构建类型、实际工作线程数，首帧分别报告计算与显示耗时。
模型和显示着色器使用构建时记录的项目路径，不依赖启动目录。


## 测试和计时

```powershell
ctest --test-dir build --output-on-failure
.\build\rasterizer_trace_tests.exe --benchmark assets/monkey/monkey.obj 1
.\build\rasterizer_trace_tests.exe --benchmark assets/monkey/monkey.obj 0
```

基准会报告帧计算时间、非黑像素数和缓冲校验值；可在最后追加 PPM 输出路径保存首帧。
比较线程数时应保持模型、分辨率及构建配置一致。
