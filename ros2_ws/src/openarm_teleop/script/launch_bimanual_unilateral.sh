#!/bin/bash
#
# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ========= Configuration =========
RIGHT_CAN=can0   # 物理右臂
LEFT_CAN=can1    # 物理左臂
# 允许用户传参覆盖物理can接口
if [ -n "$1" ]; then
    RIGHT_CAN=$1
fi
if [ -n "$2" ]; then
    LEFT_CAN=$2
fi
# 可选第三参数：如果为"left_lead"，则主左从右，否则主右从左
RIGHT_IS_LEADER=1
if [ "$3" = "left_lead" ]; then
    RIGHT_IS_LEADER=0
fi

if [ "$RIGHT_IS_LEADER" = "1" ]; then
    LEADER_CAN_IF=$RIGHT_CAN
    FOLLOWER_CAN_IF=$LEFT_CAN
else
    LEADER_CAN_IF=$LEFT_CAN
    FOLLOWER_CAN_IF=$RIGHT_CAN
fi
ARM_TYPE="v10"                  # Fixed for now
TMPDIR="/tmp/openarm_urdf_gen"
BIMANUAL_URDF_NAME="${ARM_TYPE}_bimanual.urdf"
XACRO_FILE="${ARM_TYPE}.urdf.xacro"
WS_DIR=~/ros2_ws
XACRO_PATH="$WS_DIR/src/openarm_description/urdf/robot/$XACRO_FILE"
URDF_OUT="$TMPDIR/$BIMANUAL_URDF_NAME"
BIN_PATH="$WS_DIR/build/openarm_teleop/bimanual_unilateral_control"
# ================================

echo "[INFO] Starting bimanual unilateral control script..."
echo "[INFO] Workspace directory: $WS_DIR"

# Check workspace
if [ ! -d "$WS_DIR" ]; then
    echo "[ERROR] Could not find workspace at: $WS_DIR" >&2
    echo "Please update WS_DIR in this launch script if using a different workspace." >&2
    exit 1
fi

# Check openarm_description package
if [ ! -d "$WS_DIR/src/openarm_description" ]; then
    echo "[ERROR] Could not find package: $WS_DIR/src/openarm_description" >&2
    echo "Please make sure to clone openarm_description into $WS_DIR/src/" >&2
    exit 1
fi

# Check xacro
if [ ! -f "$XACRO_PATH" ]; then
    echo "[ERROR] Could not find ${XACRO_FILE} under $WS_DIR/src/openarm_description/urdf/robot/" >&2
    exit 1
fi

# Check build binary
if [ ! -f "$BIN_PATH" ]; then
    echo "[ERROR] Compiled binary not found at: $BIN_PATH"
    echo "Please build the project first:"
    echo "  cd $WS_DIR"
    echo "  colcon build --packages-select openarm_teleop"
    exit 1
fi

# Generate URDF (bimanual with both left and right arms)
echo "[INFO] Generating bimanual URDF using xacro..."
# shellcheck source=/dev/null
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo "[WARN] setup.bash not found at $WS_DIR/install/setup.bash, trying to continue..."
fi

mkdir -p "$TMPDIR"
if ! xacro "$XACRO_PATH" bimanual:=true -o "$URDF_OUT"; then
    echo "[ERROR] Failed to generate URDF."
    exit 1
fi

# Run bimanual unilateral control binary

echo "[INFO] Launching bimanual unilateral control..."
echo "[INFO] Configuration:"
if [ "$RIGHT_IS_LEADER" = "1" ]; then
    echo "  Leader (Right Arm)  : $LEADER_CAN_IF"
    echo "  Follower (Left Arm) : $FOLLOWER_CAN_IF"
    echo "[INFO] Control Mode: Right arm leads, Left arm follows"
else
    echo "  Leader (Left Arm)   : $LEADER_CAN_IF"
    echo "  Follower (Right Arm): $FOLLOWER_CAN_IF"
    echo "[INFO] Control Mode: Left arm leads, Right arm follows"
fi
"$BIN_PATH" "$URDF_OUT" "$LEADER_CAN_IF" "$FOLLOWER_CAN_IF" $RIGHT_IS_LEADER

# Cleanup
echo "[INFO] Cleaning up tmp dir..."
rm -rf "$TMPDIR"
