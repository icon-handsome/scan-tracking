# Collision_Detection

## 介绍
兰铀容器防碰撞算法

## 代码架构说明

### 目录说明
- detection:三个工位碰撞检测算法的头文件
    - first:第一个工位碰撞检测算法(基本完成)
    - second:第二个工位碰撞检测算法（雏形）
    - third:第三个工位碰撞检测算法（雏形）
- log_manager:日志管理类的头文件
- model:模型类的头文件,涉及一些模型计算
- config:一些配置参数
- utils:工具类的头文件
    - Params.h:三个工位待求参数结构体
    - tic_toc.h:时间测量工具类
    - utils.h:一些通用工具函数

### 编译说明
- 依赖库
    - PCL 1.12
    - Eigen 3.4
- 编译步骤
  1. Arm编译
    - 编译环境:Docker,arm64(ARM64架构)
    - 构建Docker镜像
    - docker build -f Dockerfile.arm64 -t collision-arm64-builder .
    - 编译ARM64版本
    - docker run --rm -v $(pwd):/workspace collision-arm64-builder bash build-arm64.sh

  2. x86编译(本地宿主机)
   - bash build-x86.sh

- 运行说明
  -编译的分别在lib_arm64和lib_x86目录下
  - 本地运行
  - 调用demo可执行文件
  - 调用案例都在main.cpp中；可执行程序在build-x86目录下
  - 运行第一检测位检测算法(需要外表面顶部直边一圈点云，内表面点云和内表面开孔点云文件)
  - ./Collision_Detection fengtou_out_top.ply first_inliner.ply first_inliner_hole.ply