## specification

scene：负责组织和控制其他类（以及组织流程？）。

objLoader：.obj解析器，负责读取.obj文件，解析顶点、法线、纹理坐标等数据。

rasterizer：包含了光栅化。

transform: 包含了MVP,NDC,viewport变换矩阵的计算

camera：包含了 camera 的数据以及一些方法，比如处理鼠标事件。

shadeModel：着色模型，包含了着色方法（计算光照的方法）。

light：光源数据



理论上，这里应该还有 OpenGL 的类，因为我做完软渲染器之后，可能会多加一个后端。但是这个接口不太好设计，因为我现在还不怎么会用 OpenGL，GLSL 也没怎么写过。可能到时会重新设计这个后端。

