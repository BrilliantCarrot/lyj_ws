#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${UAV_GNC_WS:-$HOME/uav_gnc_ws}"
EXT_DIR="$WS_DIR/external"
OV_SRC_DIR="$EXT_DIR/open_vins"
OV_INSTALL_DIR="$EXT_DIR/openvins_install"
OV_REPO="${OPENVINS_REPO:-https://github.com/rpng/open_vins.git}"
OV_BRANCH="${OPENVINS_BRANCH:-master}"

mkdir -p "$EXT_DIR"

if [[ ! -d "$OV_SRC_DIR/.git" ]]; then
  git clone --branch "$OV_BRANCH" "$OV_REPO" "$OV_SRC_DIR"
else
  git -C "$OV_SRC_DIR" fetch origin "$OV_BRANCH"
  git -C "$OV_SRC_DIR" checkout "$OV_BRANCH"
  git -C "$OV_SRC_DIR" pull --ff-only origin "$OV_BRANCH"
fi

set +u
source /opt/ros/humble/setup.bash
set -u

colcon --log-base "$EXT_DIR/openvins_log" build \
  --base-paths "$OV_SRC_DIR" \
  --install-base "$OV_INSTALL_DIR" \
  --build-base "$EXT_DIR/openvins_build" \
  --merge-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5

cat <<EOF

OpenVINS ROS2 build complete.
Source it with:
  source $OV_INSTALL_DIR/setup.bash

Then run the UAV GNC VIO shadow launch with:
  ros2 launch uav_bringup px4_vio_shadow.launch.py start_openvins:=true
EOF
