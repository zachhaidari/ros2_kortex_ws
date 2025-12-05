#!/usr/bin/env bash
# Generates a Markdown report of active ROS2 nodes, topics, services, actions,
# and controller state focused on control & feedback.

set -euo pipefail
OUT="$(pwd)/control_feedback_report.md"
: > "$OUT"

# Source workspace if available. Some setup scripts use unset variables
# which cause errors when this script runs with `set -u` enabled.
if [ -f "install/setup.bash" ]; then
  # Temporarily disable "unbound variable" checking while sourcing
  set +u
  # shellcheck disable=SC1091
  source "install/setup.bash" || true
  set -u
fi

echo "# ROS2 Control & Feedback Report" >> "$OUT"
echo "Generated: $(date -u)" >> "$OUT"
echo "" >> "$OUT"

echo "## Environment" >> "$OUT"
echo "- ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}" >> "$OUT"
echo "" >> "$OUT"

# Nodes
echo "## Nodes (all)" >> "$OUT"
ros2 node list >> "$OUT" 2>&1 || true
echo "" >> "$OUT"

# Topics
echo "## Topics (all)" >> "$OUT"
ros2 topic list >> "$OUT" 2>&1 || true
echo "" >> "$OUT"

# Services
echo "## Services (all)" >> "$OUT"
ros2 service list >> "$OUT" 2>&1 || true
echo "" >> "$OUT"

# Actions
echo "## Actions (all)" >> "$OUT"
ros2 action list >> "$OUT" 2>&1 || true
echo "" >> "$OUT"

# Controller manager (if available)
if ros2 service list | grep -q "/controller_manager/list_controllers"; then
  echo "## Controller Manager - list_controllers" >> "$OUT"
  ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}' >> "$OUT" 2>&1 || true
  echo "" >> "$OUT"
fi

# Topic details (type + info) for control & feedback related topics
echo "## Topic details (filtered: control & feedback)" >> "$OUT"
# patterns for control/feedback
PATTERN='controller|joint|gripper|cmd|feedback|effort|twist|trajectory|follow_joint|tf|clock'

ros2 topic list 2>/dev/null | grep -E -i "$PATTERN" || true | while read -r TOPIC; do
  if [ -z "$TOPIC" ]; then continue; fi
  echo "### Topic: $TOPIC" >> "$OUT"
  echo '```' >> "$OUT"
  ros2 topic info "$TOPIC" >> "$OUT" 2>&1 || true
  echo '```' >> "$OUT"
  TYP=$(ros2 topic type "$TOPIC" 2>/dev/null || true)
  if [ -n "$TYP" ]; then
    echo "Type: $TYP" >> "$OUT"
    echo "Message definition:" >> "$OUT"
    echo '```' >> "$OUT"
    ros2 interface show "$TYP" >> "$OUT" 2>&1 || echo "(failed to show interface $TYP)" >> "$OUT"
    echo '```' >> "$OUT"
  fi
  echo "" >> "$OUT"
done

# Node details for nodes matching patterns
echo "## Node details (filtered: control & feedback)" >> "$OUT"
ros2 node list 2>/dev/null | grep -E -i "$PATTERN" || true | while read -r NODE; do
  if [ -z "$NODE" ]; then continue; fi
  echo "### Node: $NODE" >> "$OUT"
  echo '```' >> "$OUT"
  ros2 node info "$NODE" >> "$OUT" 2>&1 || true
  echo '```' >> "$OUT"
  echo "" >> "$OUT"
done

# Services filtered
echo "## Service details (filtered: control & feedback)" >> "$OUT"
ros2 service list 2>/dev/null | grep -E -i "$PATTERN" || true | while read -r SVC; do
  if [ -z "$SVC" ]; then continue; fi
  echo "### Service: $SVC" >> "$OUT"
  echo '```' >> "$OUT"
  ros2 service type "$SVC" >> "$OUT" 2>&1 || true
  echo '```' >> "$OUT"
  echo "" >> "$OUT"
done

echo "Report saved to: $OUT"

exit 0
