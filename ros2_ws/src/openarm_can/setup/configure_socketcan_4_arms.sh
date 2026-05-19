#!/bin/sh
#
# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -eu

# Simple CAN Interface Setup Script
# Usage: script/configure_socketcan.sh <interface> [options]

# Default values
BITRATE=1000000
DBITRATE=5000000
FD_MODE=true

# Show usage
usage() {
    echo "Usage: $0 <can_interface> [options]"
    echo ""
    echo "Options:"
    echo "  -fd                    Enable CAN FD mode (default: enabled)"
    echo "  -b <bitrate>           Set bitrate (default: 1000000)"
    echo "  -d <dbitrate>          Set CAN FD data bitrate (default: 5000000)"
    echo "  -h                     Show help"
    echo ""
    echo "Examples:"
    echo "  $0                          # CAN FD at 1Mbps/5Mbps (default)"
    echo "  $0 -b 500000                # CAN FD at 500kbps/5Mbps"
    echo "  $0 -d 8000000               # CAN FD with 8Mbps data rate"
}

# Parse options
while [ $# -gt 0 ]; do
    case $1 in
    -fd)
        FD_MODE=true
        shift
        ;;
    -b)
        BITRATE="$2"
        shift 2
        ;;
    -d)
        DBITRATE="$2"
        shift 2
        ;;
    -h)
        usage
        exit 0
        ;;
    *)
        echo "Error: Unknown option '$1'"
        usage
        exit 1
        ;;
    esac
done

# for each device, check if it exists and configure it
for CAN_IF in can0 can1 can2 can3; do
    echo "Configuring $CAN_IF..."

    # Check if interface exists
    if ! ip link show "$CAN_IF" >/dev/null 2>&1; then
        echo "Error: CAN interface '$CAN_IF' not found"
        echo "Available interfaces:"
        ip link show | grep -E "can[0-9]" | cut -d: -f2 | tr -d ' ' || echo "  No CAN interfaces found"
        exit 1
    fi

    # Configure CAN interface
    echo "Configuring $CAN_IF..."

    if ! sudo ip link set "$CAN_IF" down; then
        echo "Error: Failed to bring down $CAN_IF"
        exit 1
    fi

    if [ "$FD_MODE" = true ]; then
        if ! sudo ip link set "$CAN_IF" type can bitrate "$BITRATE" dbitrate "$DBITRATE" fd on; then
            echo "Error: Failed to configure CAN FD mode"
            exit 1
        fi
        echo "$CAN_IF is now set to CAN FD mode (${BITRATE} bps / ${DBITRATE} bps)"
    else
        if ! sudo ip link set "$CAN_IF" type can bitrate "$BITRATE"; then
            echo "Error: Failed to configure CAN 2.0 mode"
            exit 1
        fi
        echo "$CAN_IF is now set to CAN 2.0 mode (${BITRATE} bps)"
    fi

    if ! sudo ip link set "$CAN_IF" up; then
        echo "Error: Failed to bring up $CAN_IF"
        exit 1
    fi

    echo "✓ $CAN_IF is active"

done
