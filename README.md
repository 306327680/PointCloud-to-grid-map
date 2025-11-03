# PointCloud-to-grid-map

[English](#english) | [中文](#中文)

---

<a name="english"></a>
## English

### Introduction

PointCloud-to-grid-map is a ROS package that converts 3D point cloud data into 2D occupancy grid maps. This project leverages PCL (Point Cloud Library) and OpenCV to perform real-time point cloud processing, automatic ground detection, and generates navigation-ready occupancy grid maps suitable for mobile robot path planning and navigation.

### Features

- **3D to 2D Projection**: Converts 3D point cloud data to 2D occupancy grid maps
- **Automatic Ground Detection**: Automatically identifies ground plane without manual calibration
- **Surface Normal Analysis**: Uses PCL to calculate surface normals for obstacle classification
- **Dynamic Reconfiguration**: Supports real-time parameter adjustment via ROS dynamic_reconfigure
- **Multi-level Obstacle Classification**: Distinguishes between free space, low obstacles, and high obstacles
- **Real-time Processing**: Optimized for real-time performance with configurable processing rates

### Dependencies

#### System Requirements
- Ubuntu 16.04 or higher
- ROS Kinetic or higher

#### Required Packages
- **ROS Packages**:
  - `roscpp`
  - `nav_msgs`
  - `pcl_ros`
  - `dynamic_reconfigure`

- **Libraries**:
  - PCL 1.8 or higher
  - OpenCV 3.x
  - Boost

### Installation

1. **Create a catkin workspace** (skip if you already have one):
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

2. **Clone the repository**:
```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/PointCloud-to-grid-map.git
```

3. **Install dependencies**:
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. **Build the package**:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Usage

#### Quick Start

1. **Launch the node**:
```bash
roslaunch cloud_to_map cloud_to_map.launch
```

This will start:
- Point cloud reader node
- Voxel filter node
- Cloud-to-map conversion node
- RViz for visualization

#### Manual Launch

If you want to run the node separately:

```bash
rosrun cloud_to_map cloud_to_map_node
```

### Configuration

#### Dynamic Reconfigure Parameters

You can adjust these parameters in real-time using `rqt_reconfigure`:

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `frame` | string | "map" | - | Reference frame for the occupancy grid |
| `search_radius` | double | 0.06 | 0.0001 - 1.0 | Search radius for normal vector calculation (meters) |
| `deviation` | double | 0.785 | 0 - 1.571 | Max deviation angle from ground plane normal (radians, ~45°) |
| `buffer` | int | 0 | 0 - 100 | Number of points needed to classify cell as obstacle |
| `loop_rate` | double | 10.0 | 0 - 1000 | Node processing rate (Hz) |
| `cell_resolution` | double | 0.7 | 0 - 1.0 | Grid cell size (meters/cell) |

#### Launch with Custom Parameters

You can modify parameters in the launch file or pass them via command line:

```bash
rosrun cloud_to_map cloud_to_map_node _cell_resolution:=0.5 _search_radius:=0.08
```

### Topics

#### Subscribed Topics

- `/passthrough` (`sensor_msgs/PointCloud2`)
  - Input point cloud data (typically pre-filtered)

#### Published Topics

- `/map` (`nav_msgs/OccupancyGrid`)
  - Output 2D occupancy grid map
  - Values:
    - `-1`: Unknown/Free space
    - `0`: Definitely free space
    - `100`: Occupied/Obstacle

### Project Structure

```
PointCloud-to-grid-map/
├── launch/
│   ├── cloud_to_map.launch      # Main launch file
│   ├── 1.rviz                   # RViz configuration
│   ├── launch.rviz              # Alternative RViz config
│   └── map_navigation.launch    # Navigation launch file
├── src/
│   └── cloud_to_map/
│       ├── src/
│       │   └── cloud_to_map_node.cpp    # Main node implementation
│       ├── cfg/
│       │   └── cloud_to_map_node.cfg    # Dynamic reconfigure config
│       ├── CMakeLists.txt
│       └── package.xml
└── README.md
```

### Algorithm Overview

1. **Point Cloud Reception**: Receives filtered point cloud from `/passthrough` topic
2. **Surface Normal Calculation**: Computes normal vectors for each point using KD-tree search
3. **Ground Detection**: Automatically identifies ground plane by analyzing horizontal surfaces
4. **Height Analysis**: Calculates relative height of points above detected ground plane
5. **Grid Population**: Projects 3D points onto 2D grid cells
6. **Obstacle Classification**: Classifies cells based on:
   - Normal vector deviation from vertical
   - Point density in cell
   - Height above ground
7. **Map Publication**: Publishes resulting occupancy grid for navigation

### Troubleshooting

**Issue**: No map published
- Check if point cloud is being published to `/passthrough` topic
- Verify point cloud frame matches the configured frame

**Issue**: Map looks inverted or incorrect
- Adjust `deviation` parameter (angle threshold)
- Modify `search_radius` for normal calculation
- Check if ground plane is correctly detected

**Issue**: Performance issues
- Reduce `cell_resolution` for larger cells
- Increase voxel filter grid size to reduce point cloud density
- Lower `loop_rate` if real-time processing isn't critical

### Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

### License

This project is open source. Please check the LICENSE file for details.

### Acknowledgments

- Built with [ROS](http://www.ros.org/)
- Uses [PCL](http://pointclouds.org/) for point cloud processing
- Uses [OpenCV](https://opencv.org/) for image processing operations

---

<a name="中文"></a>
## 中文

### 项目简介

PointCloud-to-grid-map 是一个 ROS 功能包，用于将 3D 点云数据转换为 2D 占用栅格地图。该项目利用 PCL（点云库）和 OpenCV 进行实时点云处理、自动地面检测，并生成适用于移动机器人路径规划和导航的占用栅格地图。

### 功能特点

- **3D 到 2D 投影**：将 3D 点云数据转换为 2D 占用栅格地图
- **自动地面检测**：无需手动校准，自动识别地面平面
- **表面法线分析**：使用 PCL 计算表面法线进行障碍物分类
- **动态重配置**：支持通过 ROS dynamic_reconfigure 实时调整参数
- **多级障碍物分类**：区分自由空间、低障碍物和高障碍物
- **实时处理**：针对实时性能优化，处理速率可配置

### 依赖项

#### 系统要求
- Ubuntu 16.04 或更高版本
- ROS Kinetic 或更高版本

#### 必需的软件包
- **ROS 软件包**：
  - `roscpp`
  - `nav_msgs`
  - `pcl_ros`
  - `dynamic_reconfigure`

- **库文件**：
  - PCL 1.8 或更高版本
  - OpenCV 3.x
  - Boost

### 安装步骤

1. **创建 catkin 工作空间**（如果已有则跳过）：
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

2. **克隆代码仓库**：
```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/PointCloud-to-grid-map.git
```

3. **安装依赖项**：
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. **编译功能包**：
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 使用方法

#### 快速开始

1. **启动节点**：
```bash
roslaunch cloud_to_map cloud_to_map.launch
```

这将启动：
- 点云读取节点
- 体素滤波节点
- 点云到地图转换节点
- RViz 可视化工具

#### 手动启动

如果要单独运行节点：

```bash
rosrun cloud_to_map cloud_to_map_node
```

### 配置说明

#### 动态重配置参数

您可以使用 `rqt_reconfigure` 实时调整这些参数：

| 参数名 | 类型 | 默认值 | 范围 | 说明 |
|--------|------|--------|------|------|
| `frame` | string | "map" | - | 占用栅格的参考坐标系 |
| `search_radius` | double | 0.06 | 0.0001 - 1.0 | 法向量计算的搜索半径（米） |
| `deviation` | double | 0.785 | 0 - 1.571 | 与地面平面法向量的最大偏差角（弧度，约 45°） |
| `buffer` | int | 0 | 0 - 100 | 将单元格分类为障碍物所需的点数 |
| `loop_rate` | double | 10.0 | 0 - 1000 | 节点处理频率（Hz） |
| `cell_resolution` | double | 0.7 | 0 - 1.0 | 栅格单元大小（米/单元） |

#### 使用自定义参数启动

您可以在启动文件中修改参数，或通过命令行传递：

```bash
rosrun cloud_to_map cloud_to_map_node _cell_resolution:=0.5 _search_radius:=0.08
```

### 话题说明

#### 订阅的话题

- `/passthrough` (`sensor_msgs/PointCloud2`)
  - 输入点云数据（通常是预过滤的）

#### 发布的话题

- `/map` (`nav_msgs/OccupancyGrid`)
  - 输出 2D 占用栅格地图
  - 数值含义：
    - `-1`：未知/自由空间
    - `0`：确定的自由空间
    - `100`：占用/障碍物

### 项目结构

```
PointCloud-to-grid-map/
├── launch/
│   ├── cloud_to_map.launch      # 主启动文件
│   ├── 1.rviz                   # RViz 配置文件
│   ├── launch.rviz              # 备用 RViz 配置
│   └── map_navigation.launch    # 导航启动文件
├── src/
│   └── cloud_to_map/
│       ├── src/
│       │   └── cloud_to_map_node.cpp    # 主节点实现
│       ├── cfg/
│       │   └── cloud_to_map_node.cfg    # 动态重配置配置文件
│       ├── CMakeLists.txt
│       └── package.xml
└── README.md
```

### 算法概述

1. **点云接收**：从 `/passthrough` 话题接收过滤后的点云
2. **表面法线计算**：使用 KD 树搜索为每个点计算法向量
3. **地面检测**：通过分析水平表面自动识别地面平面
4. **高度分析**：计算点相对于检测到的地面平面的相对高度
5. **栅格填充**：将 3D 点投影到 2D 栅格单元
6. **障碍物分类**：基于以下条件对单元格进行分类：
   - 法向量与垂直方向的偏差
   - 单元格中的点密度
   - 相对于地面的高度
7. **地图发布**：发布生成的占用栅格地图用于导航

### 常见问题

**问题**：没有地图发布
- 检查是否有点云发布到 `/passthrough` 话题
- 验证点云坐标系是否与配置的坐标系匹配

**问题**：地图看起来颠倒或不正确
- 调整 `deviation` 参数（角度阈值）
- 修改法线计算的 `search_radius`
- 检查地面平面是否正确检测

**问题**：性能问题
- 降低 `cell_resolution` 以使用更大的单元格
- 增加体素滤波器网格大小以减少点云密度
- 如果不需要实时处理，降低 `loop_rate`

### 贡献指南

欢迎贡献！请随时提交问题或拉取请求。

### 许可证

本项目为开源项目。详细信息请查看 LICENSE 文件。

### 致谢

- 使用 [ROS](http://www.ros.org/) 构建
- 使用 [PCL](http://pointclouds.org/) 进行点云处理
- 使用 [OpenCV](https://opencv.org/) 进行图像处理操作

---

**Maintainer**: jake
**Status**: Active Development
**ROS Version**: Kinetic+
**License**: TODO
