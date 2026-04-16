#!/usr/bin/env bash

_project_restore_nounset=0
case "$-" in
  *u*)
    _project_restore_nounset=1
    set +u
    ;;
esac

source ~/ros2_humble/install/setup.bash
source ~/sou/imu_ws/install/setup.bash
source ~/sou/gps_ws/install/setup.bash
source ~/sou/lidar_ws/install/setup.bash
source ~/sou/orbbec_ws/install/setup.bash
if [ -f ~/sou/slam_ws/install/setup.bash ]; then
  source ~/sou/slam_ws/install/setup.bash
fi
if [ -d ~/sou/local_deps/geographiclib/root/usr/lib/aarch64-linux-gnu ]; then
  export LD_LIBRARY_PATH="$HOME/sou/local_deps/geographiclib/root/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
fi
if [ -f ~/sou/robot_localization_ws/install/setup.bash ]; then
  source ~/sou/robot_localization_ws/install/setup.bash
fi
if [ -f ~/use/project/install/setup.bash ]; then
  source ~/use/project/install/setup.bash
fi

if [ "$_project_restore_nounset" -eq 1 ]; then
  set -u
fi
unset _project_restore_nounset
