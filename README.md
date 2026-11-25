# maze_control

基于 **ROS 2 Humble + TurtleBot3 Burger** 的「地老鼠（MicroMouse）走迷宫」简易仿真项目。机器人依靠激光雷达与里程计，采用**左墙跟随（left-hand rule）**策略在 Gazebo 自定义迷宫中自主通行，并在 RViz2 中实时显示行驶轨迹。

课程代码:(2026-2027-1)-S0610350-03


## ✨ 功能特性

- **自定义迷宫世界**：`worlds/z_maze.world`，蛇形通道布局，墙体均设置 `static=true` 与 `<collision>` 碰撞属性，小车不会穿墙；带蓝色起点、绿色终点标记。
- **一键启动**：`cm_launch.py` 同时拉起 Gazebo、机器人模型、RViz2 与左墙跟随节点，并用 `TimerAction` 延迟启动，等待传感器数据就绪。
- **左墙跟随算法**：分区域读取 `/scan`（前方 0°±15°、左侧 60°/75°/90°），过滤无效测距，四分支分段控制；订阅 `/odom` 累积轨迹并发布 `/robot_path`。
- **雷达分区节点**：使用 NumPy 掩码提取前 / 左 / 右三区域距离（前方取最小、左右取中值），打印调试日志。
- **航点跟踪节点**：基于里程计的 P 控制器依次跟踪预设航点，同样发布可视化轨迹。

## 🛠 环境要求

| 项目        | 版本 / 说明                                                            |
| ----------- | ---------------------------------------------------------------------- |
| 操作系统    | Ubuntu 22.04                                                           |
| ROS         | ROS 2 Humble                                                           |
| 仿真器      | Gazebo Classic（配合 `gazebo_ros`）                                  |
| 机器人      | TurtleBot3 Burger（`turtlebot3_gazebo`、`turtlebot3_description`） |
| 可视化      | RViz2                                                                  |
| Python 依赖 | `rclpy`、`numpy`、`geometry_msgs`、`nav_msgs`、`sensor_msgs` |

## 📁 目录结构

```
maze_control/
├── launch/
│   └── cm_launch.py              # 一键启动：Gazebo + 小车 + RViz2 + 左墙跟随
├── maze_control/
│   ├── __init__.py
│   ├── tb3_wall_follower.py      # ★ 左墙跟随主算法（含轨迹发布）
│   ├── scan_region_node.py       # 雷达前/左/右分区处理
│   └── tb3_waypoints.py          # 航点序列跟踪（P 控制）
├── worlds/
│   └── z_maze.world              # Gazebo 蛇形迷宫世界
├── resource/maze_control         # ament 包索引
├── test/                         # 模板测试文件
├── package.xml
├── setup.py                      # 节点入口注册（console_scripts）
└── setup.cfg
```

## 🚀 安装与编译

```bash
# 1. 放入你的 ROS2 工作空间
mkdir -p ~/ros2maze_ws/src
cd ~/ros2maze_ws/src
git clone https://github.com/Circle7222/maze_control.git

# 2. 安装依赖并编译
cd ~/ros2maze_ws
rosdep install -i --from-path src --rosdistro humble -y   # 若已装依赖可跳过
colcon build --symlink-install
source install/setup.bash
```

### ⚠️ 运行前必须修改一处路径

`launch/cm_launch.py` 中迷宫世界路径是**硬编码的绝对路径**，请改成你自己机器上的实际路径：

```python
world_path = "/home/<你的用户名>/<你的工作空间>/src/maze_control/worlds/z_maze.world"
```

> 例如工作空间名为 `maze_ws` 时，改为 `/home/<你的用户名>/maze_ws/src/maze_control/worlds/z_maze.world`。
> 更优雅的做法是用 `get_package_share_directory('maze_control')` 拼接，避免换机器就要改。

## ▶️ 运行

### 一键运行（推荐）

```bash
export TURTLEBOT3_MODEL=burger
source ~/<你的工作空间>/install/setup.bash
ros2 launch maze_control cm_launch.py
```

启动流程：`Gazebo`（加载迷宫）→ `robot_state_publisher`（发布 URDF）→ `spawn_entity.py`（生成小车于起点 `(-2.0, -1.6)`）→ 3 秒后启动 `RViz2` → 10 秒后启动 `tb3_wall_follower`。延迟是为了等待 `/scan`、`/odom` 数据稳定。

### 单独运行节点

```bash
ros2 run maze_control tb3_wall_follower    # 左墙跟随（需 Gazebo 已在运行）
ros2 run maze_control scan_region_node     # 雷达分区距离日志（走迷宫项目用不到）
ros2 run maze_control tb3_waypoints        # 航点跟踪（走迷宫项目用不到）
```

## 🧠 节点与话题说明

| 节点                  | 订阅                 | 发布                          | 功能                 |
| --------------------- | -------------------- | ----------------------------- | -------------------- |
| `tb3_wall_follower` | `/scan`、`/odom` | `/cmd_vel`、`/robot_path` | 左墙跟随主算法       |
| `scan_region_node`  | `/scan`            | （终端日志）                  | 前/左/右区域距离提取 |
| `tb3_waypoints`     | `/odom`            | `/cmd_vel`、`/robot_path` | 航点序列 P 跟踪      |

**左墙跟随控制逻辑**（`control_loop`，10 Hz）：

| 条件                                  | 动作                                  |
| ------------------------------------- | ------------------------------------- |
| 前方距离 < `front_danger`（0.35 m） | 线速度 0，角速度 `-w`，原地右转避障 |
| 左侧距离 > `target_left + 0.1`      | 前进并以 `+0.4` 左转靠近墙          |
| 左侧距离 < `target_left - 0.1`      | 前进并以 `-0.3` 右转远离墙          |
| 其余（距离合适）                      | 以 `v=0.15 m/s` 直行                |

主要参数在 `tb3_wall_follower.py` 的 `__init__` 中硬编码：`target_left=0.4`、`front_danger=0.35`、`v=0.15`、`w=0.6`，可按需调整。

## 🗺 迷宫参数

- 外墙范围：`x ∈ [-2.5, 2.5]`，`y ∈ [-2.0, 2.0]`，厚 0.1 m，高 0.5 m
- 4 条内墙交替留缺口，形成约 0.7 m 宽的蛇形通道
- 起点标记：`(-2.0, -1.6)`；终点标记：`(2.0, 1.6)`

## 📊 RViz2 查看轨迹

1. 将 **Fixed Frame** 设为 `odom`（设为 `map` 会因无变换而报错）；
2. `Add → By topic → /robot_path → Path`，即可看到小车走过的绿色轨迹；
3. 可再添加 `/scan`（LaserScan）查看雷达点云。

## ❓ 常见问题

- **报 `Address already in use` / `Entity already exists`**：上次 Gazebo 未退干净，执行

  ```bash
  pkill -f gzserver
  pkill -f gzclient
  pkill -f gazebo
  ```

  后再启动。
- **`/robot_path` 无轨迹**：确认 `/odom` 有数据（`ros2 topic echo /odom`）；轨迹依赖里程计。
- **TF / 机器人模型显示异常**：`cm_launch.py` 使用 `model.sdf` 作为 `robot_description`，如需正确显示 TF，可改用 `turtlebot3_description` 包下的 `turtlebot3_burger.urdf`。

## 📝 备注

`setup.py` 中另注册了 `tb3_point2point`、`tb3_avoid` 两个入口，但仓库当前未包含对应源文件；如需使用请自行补充，或删除这两行注册项。

课程代码:(2026-2027-1)-S0610350-03
