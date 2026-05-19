#! /bin/bash
set +e

sleep 3
echo "===== 机器人启动流程开始 ====="

# ================= ROS 环境 =================
source /opt/ros/humble/setup.bash || true
source /home/openarm/ros2_ws/install/setup.bash || true
source /home/openarm/svtrobo_ws/install/setup.bash || true

# ================= 启动 rosbridge =================
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash
source /home/openarm/ros2_ws/install/setup.bash
source /home/openarm/svtrobo_ws/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
exec bash" || true

sleep 1

# ================= CAN 配置（直接执行） =================
echo "=== 配置 CAN ==="
for can_if in can0 can1; do
    echo "配置 $can_if"

    echo "openarm" | sudo -S ip link set $can_if down || true
    echo "openarm" | sudo -S /home/openarm/ros2_ws/src/openarm_can/setup/configure_socketcan.sh $can_if -fd || echo "$can_if 参数设置失败"
    echo "openarm" | sudo -S ip link set $can_if up || echo "$can_if 启动失败"

    ip -details link show $can_if
done

# ================= 启动底盘控制 =================
echo "=== CAN OK，启动底盘 ==="
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash
source /home/openarm/ros2_ws/install/setup.bash
source /home/openarm/svtrobo_ws/install/setup.bash
ros2 launch classis_control svtrobo_bringup.launch.py
exec bash" || true

sleep 1

# ================= Node 服务（前台） =================
echo "=== 启动 Node API ==="
cd /home/openarm/ros_process_api || echo "目录不存在"

export PATH=/home/openarm/.nvm/versions/node/v20.20.0/bin:/usr/bin:/bin
export NODE_PATH=/home/openarm/.nvm/versions/node/v20.20.0/lib/node_modules

/home/openarm/.nvm/versions/node/v20.20.0/bin/pnpm start || echo "pnpm 启动失败"

echo "===== 启动流程结束 ====="
exit 0

