#!/usr/bin/env bash
# Read-only stage-0 baseline check. Safe to run on the robot host.
# Usage: ./scripts/check_stage0.sh | tee docs/00_stage0_auto_snapshot.md
set -u
set +e

now() { TZ=Asia/Shanghai date '+%Y-%m-%d %H:%M:%S %Z'; }
section() { printf '\n## %s\n\n' "$1"; }
kv() { printf -- '- %s: `%s`\n' "$1" "${2:-<empty>}"; }

printf '# Stage 0 自动检查快照\n\n'
printf -- '- 采集时间: `%s`\n' "$(now)"
printf -- '- 主机名: `%s`\n' "$(hostname)"
printf -- '- 执行用户: `%s`\n' "$(id -un)"
printf -- '- 执行目录: `%s`\n' "$PWD"

section '操作系统与运行环境'
if [ -r /etc/os-release ]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  kv '发行版' "${PRETTY_NAME:-unknown}"
  kv 'VERSION_ID' "${VERSION_ID:-unknown}"
else
  kv '发行版' '无法读取 /etc/os-release'
fi
kv '内核' "$(uname -r)"
kv '架构' "$(uname -m)"
if grep -qi microsoft /proc/version 2>/dev/null; then
  kv '运行平台' 'WSL2 (硬件直通需额外配置，不推荐作为真机实时控制主机)'
else
  kv '运行平台' '原生 Linux'
fi

section '计算与存储资源'
kv '逻辑 CPU' "$(nproc 2>/dev/null)"
kv '内存' "$(free -h 2>/dev/null | awk '/^Mem:/{print $2}')"
kv '工作区可用空间' "$(df -h "$PWD" 2>/dev/null | awk 'NR==2{print $4}')"

section 'ROS 2 / 工具链'
if [ -z "${ROS_DISTRO:-}" ] && [ -f /opt/ros/humble/setup.bash ]; then
  # ROS setup scripts are not nounset-safe.
  set +u
  # shellcheck disable=SC1091
  . /opt/ros/humble/setup.bash
  set -u
fi
kv 'ROS_DISTRO' "${ROS_DISTRO:-<未设置>}"
kv 'ros2 命令' "$(command -v ros2 || echo missing)"
kv 'Humble setup' "$([ -f /opt/ros/humble/setup.bash ] && echo present || echo missing)"
kv 'colcon' "$(command -v colcon || echo missing)"
kv 'rosdep' "$(command -v rosdep || echo missing)"
kv 'xacro' "$(command -v xacro || echo missing)"
kv 'rclpy' "$(ros2 pkg prefix rclpy 2>/dev/null || echo missing)"
kv 'pip3' "$(command -v pip3 || python3 -m pip --version 2>/dev/null || echo missing)"
dpkg-query -W -f='${Package} ${Version}\n' \
  ros-humble-ros-base ros-humble-desktop ros-humble-realsense2-camera \
  ros-humble-realsense2-description can-utils 2>/dev/null | sed 's/^/- apt: `/' | sed 's/$/`/'

section 'Python 与 Conda'
kv 'python3 路径' "$(command -v python3 || echo missing)"
kv 'python3 版本' "$(python3 --version 2>&1 || echo missing)"
if command -v conda >/dev/null 2>&1; then
  kv 'conda' "$(command -v conda)"
  kv 'conda 版本' "$(conda --version 2>&1)"
else
  kv 'conda' 'missing'
  kv 'conda 版本' 'missing'
fi
printf '\n> ROS 2 Humble 的 `rclpy`/C++ 节点应使用系统 Python 和 APT 依赖。Conda 建议只隔离 LeRobot/PyTorch，不替代 ROS 2 主环境。\n'

section 'USB 设备'
kv 'lsusb' "$(command -v lsusb || echo missing)"
if command -v lsusb >/dev/null 2>&1; then
  lsusb 2>&1 | sed 's/^/- `/; s/$/`/'
  printf '\n- 匹配到的 USB-CAN/CAN 适配器数量: `%s`\n' "$(lsusb 2>/dev/null | grep -Eic 'can|peak|gs_usb|candle|usbcan|inx|canable|slcan' || true)"
  printf -- '- 匹配到的 Intel RealSense 设备数量: `%s`\n' "$(lsusb 2>/dev/null | grep -Eic 'Intel|RealSense|8086' || true)"
  printf -- '- candleLight CAN 适配器数量: `%s`\n' "$(lsusb 2>/dev/null | grep -Eic '1d50:606f' || true)"
fi

section 'CAN 接口'
kv 'ip 命令' "$(command -v ip || echo missing)"
printf '\n### CAN link\n\n```text\n'
ip -details link show type can 2>&1
printf '```\n'
printf '\n### CAN 内核模块（已加载）\n\n```text\n'
lsmod 2>/dev/null | grep -E '(^|_)(can|can_raw|gs_usb|peak_usb|slcan|vcan)' || echo '(none)'
printf '```\n'
kv 'candump' "$(command -v candump || echo missing)"

section '串口设备与稳定映射'
if [ -d /dev/serial/by-id ]; then
  find /dev/serial/by-id -maxdepth 1 -type l -printf '%f -> %l\n' 2>/dev/null | sed 's/^/- `/; s/$/`/' || true
else
  echo '- `/dev/serial/by-id`: missing'
fi
ls -l /dev/ttyUSB* /dev/ttyACM* 2>&1 | sed 's/^/- `/; s/$/`/'

section '视频设备与稳定映射'
if [ -d /dev/v4l/by-id ]; then
  find /dev/v4l/by-id -maxdepth 1 -type l -printf '%f -> %l\n' 2>/dev/null | sed 's/^/- `/; s/$/`/' || true
else
  echo '- `/dev/v4l/by-id`: missing'
fi
ls -l /dev/video* 2>&1 | sed 's/^/- `/; s/$/`/'
kv 'v4l2-ctl' "$(command -v v4l2-ctl || echo missing)"
kv 'rs-enumerate-devices' "$(command -v rs-enumerate-devices || echo missing)"

section 'udev 规则'
for f in \
  /etc/udev/rules.d/99-realsense-libusb.rules \
  /etc/udev/rules.d/99-agx-can.rules \
  /etc/udev/rules.d/99-nero-can.rules; do
  kv "$f" "$([ -e "$f" ] && echo present || echo missing)"
done

section '网络与主机时间'
printf '\n```text\n'
ip -brief address 2>&1
printf '```\n'
kv '时间' "$(now)"
kv 'NTP 同步' "$(timedatectl show -p NTPSynchronized --value 2>/dev/null || echo unknown)"
