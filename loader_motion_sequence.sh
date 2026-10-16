#!/usr/bin/env bash

set -euo pipefail

# Usage:
#   ./loader_motion_sequence.sh --help
#   ./loader_motion_sequence.sh [raised_arm] [raised_bucket_cylinder] \
#       [lowered_arm] [lowered_bucket_cylinder] [wait_seconds]
#
# The bucket is commanded through its hydraulic-cylinder joint because
# bucket_cylinder_follower continuously calculates and publishes bucket_pitch.

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  sed -n '4,10p' "$0"
  exit 0
fi

raised_arm="${1:-0.8}"
raised_bucket_cylinder="${2:-0.20}"
lowered_arm="${3:--0.15}"
lowered_bucket_cylinder="${4:-0.55}"
wait_seconds="${5:-3.0}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "Error: ros2 was not found. Source ROS 2 Humble and this workspace first." >&2
  exit 1
fi

publish_position() {
  local topic="$1"
  local target="$2"

  echo "Publishing ${topic} -> ${target}"
  ros2 topic pub --once \
    "$topic" \
    std_msgs/msg/Float64MultiArray \
    "{data: [${target}]}"
}

echo "Stage 1: raising the loader arm"
publish_position \
  /bucket_arm_position_controller_act/commands \
  "$raised_arm"
sleep 2

echo "Stage 1: adjusting the bucket cylinder after 2 seconds"
publish_position \
  /tube_position_controller_act/commands \
  "$raised_bucket_cylinder"
sleep "$wait_seconds"

echo "Stage 2: moving the bucket cylinder to ${lowered_bucket_cylinder}"
publish_position \
  /tube_position_controller_act/commands \
  "$lowered_bucket_cylinder"
sleep 2

echo "Stage 2: lowering the loader arm after 2 seconds"
publish_position \
  /bucket_arm_position_controller_act/commands \
  "$lowered_arm"

echo "Motion sequence completed."
