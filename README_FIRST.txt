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
