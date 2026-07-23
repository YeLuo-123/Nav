# S2 DREAM 激光雷达建图与 Nav2 导航

面向 S2 DREAM 移动机器人实机的 ROS 2 交付包。通过有线网络连接机器人后，可完成双激光雷达数据接收、二维占据栅格建图、RViz 可视化、键盘遥控、Nav2 路径规划与自主导航。

## 运行环境

- Ubuntu 22.04 x86_64
- ROS 2 Humble Desktop
- 有线网卡地址：`192.168.127.100/24`
- S2 控制器默认地址：`192.168.127.10`
- ROS Domain ID：`0`

仓库内已包含指定版本的 Navigation2 源码快照，安装脚本会自动解压、安装依赖并编译。

## 安全提示

> `preview` 模式只用于显示和规划，不会向底盘发送速度；`real` 模式会控制机器人运动。

- 首次调试优先使用 `preview`，确认地图、定位和规划正常后再启用 `real`。
- 实机运动前清空机器人周围区域，并确保有人手持物理急停。
- 第一个导航目标应位于 `0.3–0.5 m` 内的明确自由区域。
- 键盘控制与 Nav2 导航不要同时运行，也不要重复启动多套 Nav2。
- 电脑端驻车依赖网络，不能替代物理急停。

## 快速开始

### 1. 安装和编译

先按照 ROS 官方说明安装 ROS 2 Humble Desktop，然后在仓库根目录执行：

```bash
./setup/install_and_build.sh
```

如需使用点云转激光扫描和键盘遥控，再安装：

```bash
sudo apt install \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-teleop-twist-keyboard
```

### 2. 配置有线网络

连接机器人后查找网卡名称：

```bash
ip -br link
```

将网卡配置为 `192.168.127.100/24`，并把实际网卡名导出到当前终端：

```bash
export S2_NETWORK_INTERFACE=<网卡名>
./bin/s2_mapping network
```

检查安装、网卡、控制器和 ROS 话题：

```bash
./bin/s2_doctor
```

### 3. 建图

推荐使用交互式键盘建图：

```bash
./bin/s2_keyboard_mapping preview  # 仅预览，不驱动底盘
./bin/s2_keyboard_mapping real     # 实机键盘建图
```

实机模式下使用以下按键：

- `i` / `,`：前进 / 后退
- `j` / `l`：左转 / 右转
- `Shift+u/o/m/.`：横移
- `k`：停止
- `p`：保存地图
- `Ctrl+C`：退出

地图默认保存到 `output/keyboard_map_<时间戳>/map.yaml`。也可以使用以下入口：

```bash
./bin/s2_cartographer_keyboard_mapping  # Cartographer 键盘建图
./bin/s2_autonomous_mapping             # 自主探索建图（实机会运动）
```

仅接收并累计点云时，可使用：

```bash
./bin/s2_mapping start
./bin/s2_rviz
./bin/s2_mapping status
./bin/s2_mapping files
./bin/s2_mapping stop
```

### 4. Nav2 导航

指定已保存的地图：

```bash
export S2_MAP_YAML="$PWD/output/keyboard_map_<时间戳>/map.yaml"
```

先启动无运动预览：

```bash
./bin/s2_nav2 preview
```

在 RViz 中：

1. 使用 **2D Pose Estimate** 标记机器人的实际位置和朝向。
2. 等待实时点云与地图基本重合。
3. 使用 **2D Goal Pose** 下发附近的测试目标。

确认定位、规划和现场安全后，停止预览实例，再启用实机导航：

```bash
./bin/s2_nav2 real
```

### 5. 紧急驻车

任意终端执行：

```bash
export S2_NETWORK_INTERFACE=<网卡名>
./bin/s2_estop
```

该命令会停止本机导航/键盘速度源、发送零速度并请求底盘驻车。

## 常用命令

| 命令 | 作用 |
| --- | --- |
| `./bin/s2_doctor` | 检查环境、网络、构建结果和 ROS 话题 |
| `./bin/s2_keyboard preview\|real` | 单独进行键盘遥控 |
| `./bin/s2_keyboard_mapping preview\|real` | 键盘控制并同步建图 |
| `./bin/s2_nav2 preview\|real` | 启动 Nav2 预览或实机导航 |
| `./bin/s2_rviz` | 启动建图 RViz |
| `./bin/s2_estop` | 停止本机控制源并请求底盘驻车 |
| `python3 tests/smoke_test.py` | 执行静态冒烟测试 |
| `sha256sum -c SHA256SUMS` | 校验原始交付文件完整性 |

## 配置

常用环境变量：

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `S2_NETWORK_INTERFACE` | 自动探测，失败时为 `eno1` | 连接机器人的有线网卡 |
| `S2_CONTROLLER_IP` | `192.168.127.10` | 底盘控制器地址 |
| `S2_HOST_CIDR` | `192.168.127.100/24` | 主机有线地址 |
| `S2_ROS_DOMAIN_ID` | `0` | ROS 2 Domain ID |
| `S2_MAP_YAML` | 未设置 | Nav2 使用的地图 YAML |
| `S2_NAV2_PARAMS` | `configs/navigation/s2_nav2_params.yaml` | Nav2 参数文件 |

主要配置位于：

- `configs/navigation/`：Nav2 参数
- `configs/cartographer/`：Cartographer 参数
- `configs/robots/`：CycloneDDS 网络配置
- `configs/rviz/`：RViz 显示配置
- `robot_description/`：S2 URDF 与网格模型

## 项目结构

```text
bin/                用户命令入口
configs/            导航、建图、DDS 和 RViz 配置
docs/               产品资料与完整操作手册
robot_description/  机器人模型
setup/              环境初始化和安装脚本
tests/              冒烟测试与功能测试
third_party/         Navigation2 Humble 源码快照
tools/               ROS 2 节点、Launch 文件和底层脚本
output/              运行时地图与日志（运行后生成）
```

## 故障排查

### 找不到机器人或 ROS 话题

```bash
ip -br addr show <网卡名>
ping -c 2 192.168.127.10
export S2_NETWORK_INTERFACE=<网卡名>
./bin/s2_doctor
```

确认网卡已获得 `192.168.127.100/24`，且没有把 DDS 绑定到 Wi-Fi。

### RViz 能显示地图，但无法规划

每次启动导航后都需要用 **2D Pose Estimate** 重新设置初始位姿。未建立 `map -> odom` 变换时，规划器不会进入可用状态。

### 已下发目标，但机器人不移动

- 确认运行的是 `real` 而非 `preview`。
- 确认没有另一套 Nav2 或键盘控制进程占用速度输出。
- 确认底盘已进入软手动模式并退出驻车。
- 运行 `./bin/s2_doctor` 检查控制器与里程计话题。

## 已知限制

- 默认点云栅格地图依赖底盘里程计，不包含回环优化；长距离或闭环路线可能出现地图形变和重影。
- 玻璃等透明障碍主要由超声波作为实时本地障碍处理，不会写入静态地图。
- 网络中断时，电脑端驻车命令无法替代机器人的物理急停。

更完整的实机操作、状态检查和问题处理请阅读：

- [S2 有线建图与 Nav2 导航完整操作手册](docs/S2有线建图与Nav2导航完整操作手册_20260717.md)
- [S2 DREAM 激光雷达建图迁移与测试手册](docs/S2_DREAM激光雷达建图迁移与测试手册_20260716.md)
