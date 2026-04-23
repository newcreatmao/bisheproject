#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

ROS_SETUP="${ROS_SETUP:-/home/mao/ros2_humble/install/setup.bash}"
ORBBEC_SETUP="${ORBBEC_SETUP:-/home/mao/sou/orbbec_ws/install/setup.bash}"
GPS_SETUP="${GPS_SETUP:-/home/mao/sou/gps_ws/install/setup.bash}"
IMU_SETUP="${IMU_SETUP:-/home/mao/sou/imu_ws/install/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-$PROJECT_ROOT/install/setup.bash}"

RUNTIME_LOG_DIR="${PROJECT_RUNTIME_LOG_DIR:-$PROJECT_ROOT/log/runtime}"
PID_DIR="$RUNTIME_LOG_DIR/pids"
STACK_LOG_FILE="$RUNTIME_LOG_DIR/project_stack.log"
WEB_LOG_FILE="$RUNTIME_LOG_DIR/project_web.log"
STACK_PID_FILE="$PID_DIR/project_stack.pid"
WEB_PID_FILE="$PID_DIR/project_web.pid"

PROJECT_HOST="${PROJECT_HOST:-0.0.0.0}"
PROJECT_PORT="${PROJECT_PORT:-8080}"
PROJECT_WEB_ROOT="${PROJECT_WEB_ROOT:-$PROJECT_ROOT/source/web}"
PROJECT_STORAGE_ROOT="${PROJECT_STORAGE_ROOT:-$PROJECT_ROOT/source/allfile}"

CAMERA_NAME="${CAMERA_NAME:-camera}"
IMU_PORT="${IMU_PORT:-/dev/imu}"
IMU_BAUD="${IMU_BAUD:-460800}"
IMU_RATE_HZ="${IMU_RATE_HZ:-50}"
LIDAR_PARAMS_FILE="${LIDAR_PARAMS_FILE:-$PROJECT_ROOT/source/config/lsn10_serial.yaml}"
RGB_YOLO_CAMERA_SOURCE="${RGB_YOLO_CAMERA_SOURCE:-/dev/video0}"
RGB_YOLO_CAPTURE_PERIOD_SEC="${RGB_YOLO_CAPTURE_PERIOD_SEC:-0.333}"
YOLO_MODEL_PATH="${YOLO_MODEL_PATH:-$PROJECT_ROOT/source/pt/best.onnx}"
YOLO_CLASSES_PATH="${YOLO_CLASSES_PATH:-$PROJECT_ROOT/source/pt/classes.txt}"
YOLO_CONF_THRESHOLD="${YOLO_CONF_THRESHOLD:-0.35}"
PHOTO_OUTPUT_DIR="${PHOTO_OUTPUT_DIR:-$PROJECT_ROOT/source/allfile/photos}"

SCRIPT_NAME="$(basename "$0")"

usage() {
    cat <<EOF
用法:
  $SCRIPT_NAME [start|stop|status|restart]

默认不带参数时执行 start。

可选环境变量:
  PROJECT_HOST                  Web 服务监听地址，默认 0.0.0.0
  PROJECT_PORT                  Web 服务端口，默认 8080
  CAMERA_NAME                   深度相机节点名，默认 camera
  IMU_PORT                      IMU 串口，默认 /dev/imu
  IMU_BAUD                      IMU 波特率，默认 460800
  IMU_RATE_HZ                   IMU 配置频率，默认 50
  LIDAR_PARAMS_FILE             雷达参数文件，默认 source/config/lsn10_serial.yaml
  RGB_YOLO_CAMERA_SOURCE        RGB+YOLO 视频源，默认 /dev/video0
  RGB_YOLO_CAPTURE_PERIOD_SEC   RGB+YOLO 采样周期，默认 0.333（约 3fps）
  YOLO_MODEL_PATH               YOLO 模型路径，默认 source/pt/best.onnx
  YOLO_CLASSES_PATH             YOLO 类别文件路径，默认 source/pt/classes.txt
  YOLO_CONF_THRESHOLD           YOLO 置信度阈值，默认 0.35
  PHOTO_OUTPUT_DIR              RGB+YOLO 输出基准目录，实时帧写到 source/allfile/live
  ORBBEC_SETUP                  Orbbec 工作区环境，默认 /home/mao/sou/orbbec_ws/install/setup.bash
  GPS_SETUP                     GPS 工作区环境，默认 /home/mao/sou/gps_ws/install/setup.bash
  IMU_SETUP                     IMU 工作区环境，默认 /home/mao/sou/imu_ws/install/setup.bash
EOF
}

ensure_env() {
    if [[ ! -f "$ROS_SETUP" ]]; then
        echo "缺少 ROS 环境: $ROS_SETUP" >&2
        exit 1
    fi
    if [[ ! -f "$WORKSPACE_SETUP" ]]; then
        echo "缺少项目环境: $WORKSPACE_SETUP" >&2
        echo "先完成 project 的构建和安装。" >&2
        exit 1
    fi
    if [[ ! -x "$PROJECT_ROOT/bin/project" ]]; then
        echo "缺少主程序: $PROJECT_ROOT/bin/project" >&2
        exit 1
    fi
}

ensure_dirs() {
    mkdir -p "$PID_DIR" "$RUNTIME_LOG_DIR" "$PROJECT_STORAGE_ROOT" "$PHOTO_OUTPUT_DIR"
}

is_pid_alive() {
    local pid="$1"
    kill -0 "$pid" 2>/dev/null
}

matches_expected_command() {
    local pid="$1"
    local needle="$2"
    local cmdline
    cmdline="$(ps -o args= -p "$pid" 2>/dev/null || true)"
    [[ -n "$cmdline" && "$cmdline" == *"$needle"* ]]
}

component_pid() {
    local pid_file="$1"
    local needle="$2"

    if [[ ! -f "$pid_file" ]]; then
        return 1
    fi

    local pid
    pid="$(tr -d '[:space:]' < "$pid_file")"
    if [[ ! "$pid" =~ ^[0-9]+$ ]]; then
        rm -f "$pid_file"
        return 1
    fi

    if is_pid_alive "$pid" && matches_expected_command "$pid" "$needle"; then
        printf '%s\n' "$pid"
        return 0
    fi

    rm -f "$pid_file"
    return 1
}

start_component() {
    local name="$1"
    local pid_file="$2"
    local log_file="$3"
    local needle="$4"
    local command="$5"

    local pid
    if pid="$(component_pid "$pid_file" "$needle")"; then
        echo "$name 已在运行，PID=$pid"
        return 0
    fi

    printf '\n[%s] starting %s\n' "$(date '+%F %T')" "$name" >> "$log_file"
    setsid bash -lc "$command" >>"$log_file" 2>&1 &
    pid=$!
    echo "$pid" > "$pid_file"
    sleep 1

    if ! component_pid "$pid_file" "$needle" >/dev/null; then
        echo "$name 启动失败，查看日志: $log_file" >&2
        tail -n 40 "$log_file" >&2 || true
        rm -f "$pid_file"
        exit 1
    fi

    echo "$name 已启动，PID=$pid"
}

stop_component() {
    local name="$1"
    local pid_file="$2"
    local needle="$3"

    local pid
    if ! pid="$(component_pid "$pid_file" "$needle")"; then
        echo "$name 未运行"
        return 0
    fi

    echo "停止 $name，PID=$pid"
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true

    for _ in $(seq 1 10); do
        if ! is_pid_alive "$pid"; then
            rm -f "$pid_file"
            echo "$name 已停止"
            return 0
        fi
        sleep 1
    done

    kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    rm -f "$pid_file"
    echo "$name 已强制停止"
}

print_status_component() {
    local name="$1"
    local pid_file="$2"
    local needle="$3"

    local pid
    if pid="$(component_pid "$pid_file" "$needle")"; then
        echo "$name: running (PID=$pid)"
    else
        echo "$name: stopped"
    fi
}

start_stack() {
    local command
    command="
source \"$ROS_SETUP\"
[[ -f \"$ORBBEC_SETUP\" ]] && source \"$ORBBEC_SETUP\"
[[ -f \"$GPS_SETUP\" ]] && source \"$GPS_SETUP\"
[[ -f \"$IMU_SETUP\" ]] && source \"$IMU_SETUP\"
source \"$WORKSPACE_SETUP\"
export PROJECT_ROOT=\"$PROJECT_ROOT\"
export PROJECT_WEB_ROOT=\"$PROJECT_WEB_ROOT\"
export PROJECT_STORAGE_ROOT=\"$PROJECT_STORAGE_ROOT\"
exec ros2 launch project project_stack.launch.py \
  camera_name:=\"$CAMERA_NAME\" \
  imu_port:=\"$IMU_PORT\" \
  imu_baud:=\"$IMU_BAUD\" \
  imu_rate_hz:=\"$IMU_RATE_HZ\" \
  lidar_params_file:=\"$LIDAR_PARAMS_FILE\" \
  rgb_yolo_camera_source:=\"$RGB_YOLO_CAMERA_SOURCE\" \
  rgb_yolo_capture_period_sec:=\"$RGB_YOLO_CAPTURE_PERIOD_SEC\" \
  yolo_model_path:=\"$YOLO_MODEL_PATH\" \
  yolo_classes_path:=\"$YOLO_CLASSES_PATH\" \
  yolo_conf_threshold:=\"$YOLO_CONF_THRESHOLD\" \
  photo_output_dir:=\"$PHOTO_OUTPUT_DIR\"
"

    start_component \
        "ROS 传感器栈" \
        "$STACK_PID_FILE" \
        "$STACK_LOG_FILE" \
        "project_stack.launch.py" \
        "$command"
}

start_web() {
    local command
    command="
source \"$ROS_SETUP\"
[[ -f \"$ORBBEC_SETUP\" ]] && source \"$ORBBEC_SETUP\"
[[ -f \"$GPS_SETUP\" ]] && source \"$GPS_SETUP\"
[[ -f \"$IMU_SETUP\" ]] && source \"$IMU_SETUP\"
source \"$WORKSPACE_SETUP\"
export PROJECT_ROOT=\"$PROJECT_ROOT\"
export PROJECT_WEB_ROOT=\"$PROJECT_WEB_ROOT\"
export PROJECT_STORAGE_ROOT=\"$PROJECT_STORAGE_ROOT\"
export PROJECT_HOST=\"$PROJECT_HOST\"
export PROJECT_PORT=\"$PROJECT_PORT\"
exec \"$PROJECT_ROOT/bin/project\"
"

    start_component \
        "Web 控制台" \
        "$WEB_PID_FILE" \
        "$WEB_LOG_FILE" \
        "$PROJECT_ROOT/bin/project" \
        "$command"
}

start_all() {
    ensure_env
    ensure_dirs
    start_stack
    start_web
    echo "启动完成"
    echo "页面地址: http://127.0.0.1:$PROJECT_PORT"
    echo "传感器日志: $STACK_LOG_FILE"
    echo "Web 日志: $WEB_LOG_FILE"
}

stop_all() {
    stop_component "Web 控制台" "$WEB_PID_FILE" "$PROJECT_ROOT/bin/project"
    stop_component "ROS 传感器栈" "$STACK_PID_FILE" "project_stack.launch.py"
}

status_all() {
    print_status_component "ROS 传感器栈" "$STACK_PID_FILE" "project_stack.launch.py"
    print_status_component "Web 控制台" "$WEB_PID_FILE" "$PROJECT_ROOT/bin/project"
    echo "传感器日志: $STACK_LOG_FILE"
    echo "Web 日志: $WEB_LOG_FILE"
}

ACTION="${1:-start}"

case "$ACTION" in
    start)
        start_all
        ;;
    stop)
        stop_all
        ;;
    status)
        status_all
        ;;
    restart)
        stop_all
        start_all
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        echo "未知参数: $ACTION" >&2
        usage >&2
        exit 1
        ;;
esac
