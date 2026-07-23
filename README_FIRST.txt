S2 激光雷达建图与 Nav2 导航交付包
====================================

用途：在另一台 Ubuntu 22.04 笔记本上，通过网线连接 S2 底盘，完成双雷达点云
接收、二维占据图构建、RViz 三维显示、Nav2 路径规划和小范围实机导航。

首次使用按以下顺序：

1. 阅读 docs/S2有线建图与Nav2导航完整操作手册_20260717.md。
2. 安装 ROS 2 Humble Desktop。
3. 执行：./setup/install_and_build.sh
4. 插好网线，用 ip -br link 找到网卡名。
5. 执行：export S2_NETWORK_INTERFACE=<网卡名>
6. 执行：./bin/s2_mapping network
7. 执行：./bin/s2_doctor
8. 执行：./bin/s2_mapping start
9. 新终端执行：./bin/s2_rviz
10. 只规划测试：./bin/s2_nav2 preview
11. 明确准备好实机运动后：./bin/s2_nav2 real

键盘控制：

- 只预览速度命令：./bin/s2_keyboard preview
- 明确准备好实机运动后：./bin/s2_keyboard real
- i/, 前进/后退，j/l 旋转，Shift+u/o/m/. 横移，k 停车，Ctrl+C 退出。
- 键盘控制和 Nav2 导航不要同时运行。

电脑端紧急驻车：

- 任意终端执行：./bin/s2_estop
- 该命令会停止本机导航/键盘速度源，直接发送零速度并请求底盘驻车。
- 物理急停仍是最终安全保障；断网时电脑端命令无法替代物理急停。

安全说明：

- preview 模式不会把速度发给底盘。
- real 模式会切换软手动、退出驻车并通过 WebSocket 发送速度。
- real 模式必须有人手持急停，首个目标限制在 0.3-0.5 m 的明确自由区。
- 当前地图是底盘里程计约束下的点云二维融合，不包含回环优化；长距离运行可能重影。

完整性检查：

  sha256sum -c SHA256SUMS


指定地图的实机导航（当前推荐流程）
==================================

当前使用的地图：

  /home/fq/nav2/output/keyboard_map_20260720_175854/map.yaml

该流程会同时启动：静态地图、RViz、Nav2、AMCL、点云转二维扫描、碰撞监控和
超声波融合。不要同时启动第二套 Nav2 或键盘控制节点。

一、首次安装额外依赖并编译
--------------------------

  cd /home/fq/nav2
  sudo apt install ros-humble-pointcloud-to-laserscan
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  cd /home/fq/nav2/nav2_ws
  colcon build --symlink-install

二、确认网线和底盘连通
----------------------

  cd /home/fq/nav2
  ip -br addr show enp0s31f6
  ping -c 2 192.168.127.10

正常情况下，有线网卡应包含 192.168.127.100/24，底盘地址为
192.168.127.10。若本机有线网卡名称不是 enp0s31f6，请把后续命令中的名称
替换为实际名称。

三、检查是否已有导航进程
------------------------

  pgrep -af 's2_nav2_navigation.launch.py|s2_nav2_cmd_vel_bridge|rviz2'

若已经有完整导航实例在运行，不要重复启动。需要停止当前前台实例时，在启动它的
终端按 Ctrl+C。需要紧急驻车时执行：

  cd /home/fq/nav2
  export S2_NETWORK_INTERFACE=enp0s31f6
  ./bin/s2_estop

四、启动实机导航
----------------

确认有人手持物理急停，然后执行：

  cd /home/fq/nav2
  export S2_NETWORK_INTERFACE=enp0s31f6
  export S2_MAP_YAML=/home/fq/nav2/output/keyboard_map_20260720_175854/map.yaml
  ./bin/s2_nav2 real

该终端必须保持运行。Ctrl+C 会停止本次导航。

只想验证地图、路径和 RViz，不允许机器人运动时，把最后一行改为：

  ./bin/s2_nav2 preview

五、在 RViz 中完成重定位
------------------------

每次重新启动导航后，AMCL 都需要新的初始位姿：

1. 在 RViz 菜单 Tools 中选择 2D Pose Estimate（绿色箭头）。
2. 在地图上机器人实际位置按住鼠标左键。
3. 拖出机器人实际朝向后松开。
4. 等待约 5-10 秒，确认实时点云与地图边缘基本重合。
5. 再选择 2D Goal Pose 设置附近的导航目标。

不要把 2D Goal Pose 当成初始位姿工具。未完成重定位时，map -> odom 变换不存在，
规划器不会进入可用状态。

六、启动后的快速检查（另开终端）
--------------------------------

  cd /home/fq/nav2
  export S2_NETWORK_INTERFACE=enp0s31f6
  source setup/s2_bundle_env.sh "$S2_NETWORK_INTERFACE"
  set +u
  source tools/s2_nav2_source_env.sh
  set -u
  export ROS_DISABLE_DAEMON=1

  ros2 lifecycle get /amcl
  ros2 lifecycle get /controller_server
  ros2 lifecycle get /planner_server
  ros2 lifecycle get /bt_navigator
  ros2 lifecycle get /collision_monitor
  timeout 6 ros2 topic hz /s2_lidar_slam/scan
  timeout 6 ros2 topic hz /s2_ultrasonic/points

正常参考值：二维激光扫描约 2.5 Hz，超声波融合话题约 10 Hz。设置初始位姿后，
上述 Nav2 生命周期节点应为 active。

检查超声波硬件状态：

  ros2 topic echo /driver/radar/Data --once
  ros2 topic echo /s2_ultrasonic/points --once --field width

8 个超声波应显示 is_connected=true。若全部为 false 且融合点云 width 为 0，说明
底盘超声波硬件或 RS485/厂商驱动未上线；此时不能依赖超声波识别玻璃。

七、目标已下发但机器人不移动
----------------------------

先确认底盘控制接口能够切换软手动并退出驻车：

  curl -sS --max-time 10 -H 'Content-Type: application/json' \
    -d '{"id":1}' http://192.168.127.10:8080/api/AMR/SetManualMode

  curl -sS --max-time 10 -H 'Content-Type: application/json' \
    -d '{}' http://192.168.127.10:8080/api/AMR/StopParking

正常返回应包含 "ErrorCode":0；StopParking 若返回已经退出驻车的 Conflict 也可接受。
若 ping 正常、8080 端口可连接，但两个接口一直无响应，应先重启底盘控制服务或底盘，
不要反复下发导航目标。

八、当前方案的限制
------------------

- 当前静态地图由底盘里程计约束的点云累计生成，没有回环优化。
- 长距离或闭环路线可能存在地图形变，AMCL 即使粒子收敛仍可能有实际位置偏差。
- 玻璃由超声波作为实时本地障碍处理，不会写入静态地图。
- 首个实机目标应限制在 0.3-0.5 m 的明确自由区域。
