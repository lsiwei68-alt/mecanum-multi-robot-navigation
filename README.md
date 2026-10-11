# mecanum_multi_nav

ROS 2 Humble + Gazebo Fortress 双麦克纳姆轮小车协同导航学习项目。

第一阶段采用方案 A：两辆车各运行一套带命名空间的 Nav2，共用同一张已知地图，
分别规划到终点的两个编队槽位。两车的局部代价地图会把对方作为动态障碍物，
终点保持相同朝向并排停放。

## 当前结构

- `/robot1`、`/robot2`：独立的速度、里程计、TF、扫描和 Nav2 接口
- 唯一 TF 帧：`robot1/odom -> robot1/base_link` 与
  `robot2/odom -> robot2/base_link`，共享 `map` 根坐标系
- `/tf`、`/tf_static`：汇总后的显示树，供单个 RViz 同时观察两辆车
- 全局规划：各自使用 NavFn，在同一张 `arena.yaml` 地图上规划
- 局部规划：支持 `linear.y` 的 DWB 全向运动模型
- 编队层：根据目标中心、朝向和间距生成两个终点槽位，并做最短距离分配
- 相互感知：`scan` 同时包含静态环境和另一辆小车，供局部代价地图避障
- 定位扫描：`scan_static` 只含静态环境，避免移动小车干扰 AMCL

Gazebo Fortress 的 Ogre 2 GPU 雷达在当前 WSL 的 NVIDIA / D3D12 路径上会触发
`GL3PlusTextureGpu::copyTo` 异常。Mesa CPU 软件渲染虽然可以稳定生成原生扫描，
但当前低矮车体低于水平扫描平面，原生雷达会从另一辆车上方扫过。因此第一阶段
固定使用确定性的二维射线节点生成同等格式的 `sensor_msgs/LaserScan`。以后换到
原生支持良好的 Gazebo 或实车雷达时，只需替换扫描来源，Nav2、代价地图与编队
逻辑无需改动。

## 构建

```bash
cd ~/multi_robot_nav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 一条命令启动完整演示（推荐）

下面一条命令依次启动 Gazebo、两套 Nav2、单窗口 RViz，并在 8 秒后启动编队
任务。编队节点即使先于 Nav2 完全就绪，也会继续等待生命周期节点激活：

```bash
source /opt/ros/humble/setup.bash
source ~/multi_robot_nav_ws/install/setup.bash
ros2 launch mecanum_multi_nav complete_demo.launch.py
```

`native_lidar` 默认就是 `false`。也可以在命令中显式写出，避免误用实验模式：

```bash
ros2 launch mecanum_multi_nav complete_demo.launch.py native_lidar:=false
```

无 Gazebo GUI 运行：

```bash
ros2 launch mecanum_multi_nav complete_demo.launch.py headless:=true
```

Gazebo 原生 GPU Lidar 只保留为实验选项。当前机器上应使用 Mesa CPU 软件渲染：

```bash
ros2 launch mecanum_multi_nav complete_demo.launch.py \
  native_lidar:=true software_rendering:=true
```

只验证雷达和导航、不打开 Gazebo GUI：

```bash
ros2 launch mecanum_multi_nav complete_demo.launch.py \
  native_lidar:=true software_rendering:=true headless:=true
```

该模式在 Gazebo 场景中直接生成两路 360 点、10 Hz 的扫描，并通过 ros_gz_bridge
发布到 `/robot1/scan` 和 `/robot2/scan`。它能正确扫描墙体和固定障碍物，但由于
本模型的雷达扫描平面高于另一辆车的车身，目前不能用于双车互相感知。设置
`software_rendering:=false` 会尝试 NVIDIA 硬件路径，但更新到 WSL 3.0.1 后复测仍会
触发 Ogre2 异常，因此当前不用于正式演示。

可以覆盖终点和编队参数：

```bash
ros2 launch mecanum_multi_nav complete_demo.launch.py \
  goal_x:=2.5 goal_y:=1.0 goal_yaw:=1.5708 spacing:=1.2
```

也可用 `rviz:=false` 或 `run_demo:=false` 关闭对应自动启动项。

## 分终端启动

终端 1（带 Gazebo GUI）：

```bash
source /opt/ros/humble/setup.bash
source ~/multi_robot_nav_ws/install/setup.bash
ros2 launch mecanum_multi_nav dual_robot_nav.launch.py
```

若只想测试而不打开 GUI：

```bash
ros2 launch mecanum_multi_nav dual_robot_nav.launch.py headless:=true
```

终端 2：

```bash
source /opt/ros/humble/setup.bash
source ~/multi_robot_nav_ws/install/setup.bash
ros2 run mecanum_multi_nav formation_demo --ros-args -p use_sim_time:=true
```

可在另一个终端启动单窗口双车 RViz，显示 robot1 和 robot2 的地图、全局路径、
局部路径、局部代价地图、激光扫描和车体 footprint：

```bash
source /opt/ros/humble/setup.bash
source ~/multi_robot_nav_ws/install/setup.bash
ros2 launch mecanum_multi_nav dual_rviz.launch.py
```

默认目标中心为 `(3.0, 0.0)`，朝向为 `0 rad`，两车中心距为 `1.2 m`。可覆盖参数：

```bash
ros2 run mecanum_multi_nav formation_demo --ros-args \
  -p use_sim_time:=true -p goal_x:=2.5 -p goal_y:=1.0 \
  -p goal_yaw:=1.5708 -p spacing:=1.2
```

## 单车运动冒烟测试

```bash
ros2 launch mecanum_multi_nav single_robot_sim.launch.py headless:=true
ros2 run mecanum_multi_nav motion_smoke_test --ros-args -p robot_name:=robot1
```

该测试依次验证前进、横移和旋转，证明底盘确实是全向运动，而不是差速底盘模型。

## 地图工作流

当前演示直接加载预制的 `maps/arena.yaml`。正式流程可以让 robot1 单独运行
SLAM Toolbox 建图并保存地图，然后 robot1 和 robot2 在导航阶段共同加载保存后的
同一份地图；这是合理且常用的“离线建图、在线多机器人定位导航”方式。
