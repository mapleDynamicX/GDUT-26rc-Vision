# rootdir

想了解 CMake 中`add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")`这句指令的**核心作用、各部分拆解、实际代码使用方式**，还有使用时的关键注意事项和现代 CMake 的推荐写法，下面我会从基础到实战详细讲解，适配 CMake 新手的理解节奏。

## 一、核心作用总述

这句是**CMake 向 C/C++ 编译器传递预处理器宏定义**的指令，核心目的是：

在**当前 CMakeLists.txt 及其所有子目录**的所有编译单元（.c/.cpp/.h 文件）中，定义一个名为`ROOT_DIR`的预处理器宏，宏的**值**是**当前 CMakeLists.txt 所在目录的绝对路径**（末尾带斜杠）。

C/C++ 代码中可以**直接使用这个宏**，无需手动定义，从而在代码中动态获取工程目录路径，**彻底替代硬编码路径**（比如直接写`/home/xxx/project/`或`C:\xxx\project\`），让工程的路径处理跨平台（Windows/Linux/macOS）、跨机器、跨编译目录都能正常工作。

简单说：**CMake 把目录路径通过宏定义 “传给” C/C++ 代码，代码直接用这个路径，不用写死**。

## 二、指令各部分逐字拆解

这句指令由**CMake 命令**、**编译器参数**、**CMake 变量**、**转义字符**组成，逐个拆解后就能理解背后的逻辑，也是避免踩坑的关键：

```cmake
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
```

### 1. `add_definitions`：CMake 的宏定义传递命令

- 这是 CMake 的内置核心命令，作用是**向编译器传递「预处理器定义选项」**，本质是给 gcc/clang/MSVC 等编译器添加`-D`类参数（不同编译器的参数一致，CMake 自动做跨平台兼容）。
- 作用域：**当前 CMakeLists.txt 文件 + 该文件后续的所有子目录**（通过`add_subdirectory`引入的子目录）的所有编译目标（`add_executable`/`add_library`创建的可执行文件、库）。
- 注意：这是 CMake**较旧的全局命令**，CMake3.0 + 有更推荐的目标级写法，后面会讲。

### 2. `-DROOT_DIR`：编译器的预处理器宏定义语法

- `-D`是 C/C++ 编译器的**预处理器宏定义标志**（gcc/clang/MSVC 均支持），格式为`-D宏名=宏值`，如果宏值是字符串，需要加双引号。
- `ROOT_DIR`是**自定义宏名**，建议用「大写 + 下划线」命名，避免和代码中的变量 / 宏重名（比如`PROJ_ROOT_DIR`、`APP_ROOT_DIR`）。

### 3. `\"`：CMake 中的双引号转义符

- CMake 中，双引号`"`是**字符串定界符**（用来包裹 CMake 的字符串），如果直接写`"${xxx}"`，CMake 会把双引号解析为自身的语法，**不会传递给编译器**。
- 反斜杠`\`是 CMake 的转义字符，`\"`表示**让 CMake 把双引号原封不动地传递给编译器**，最终编译器拿到的宏定义是`ROOT_DIR="xxx/"`（带双引号）。
- ✅ 关键：**漏写反斜杠会直接导致 C/C++ 代码编译报错**（后续会讲坑点）。

### 4. `${CMAKE_CURRENT_SOURCE_DIR}`：CMake 内置路径变量

## 使用

```cpp
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }
    fout_out.close();
    fout_imu_pbp.close();
```
