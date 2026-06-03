#!/usr/bin/env bash
set -euo pipefail

SESSION="${SESSION:-amr}"
WS="/ros2_ws"

# Map was originally generated with the robot spawned at (-9.0, -7.5).
# If we change the Gazebo spawn, AMCL initial pose must shift by the same delta.
ORIGINAL_SPAWN_X="-9.0"
ORIGINAL_SPAWN_Y="-7.5"
SPAWN_X="-9.0"
SPAWN_Y="-7.5"
SPAWN_Z="0.1"
SPAWN_YAW="0.0"
INITIAL_POSE_X="0.0"
INITIAL_POSE_Y="0.0"
INITIAL_POSE_YAW="0.0"
read -r INITIAL_POSE_QZ INITIAL_POSE_QW < <(python3 - <<PY
import math
yaw = float(${INITIAL_POSE_YAW})
print(math.sin(yaw * 0.5), math.cos(yaw * 0.5))
PY
)

safe_source() {
  # ROS setup scripts may reference optional vars that are unset under `set -u`.
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

cleanup_stale_sim() {
  echo "Cleaning stale Gazebo/sim processes..."
  pkill -f "ign gazebo" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "ros_gz_sim create" 2>/dev/null || true
  pkill -f "ros_gz_bridge.*parameter_bridge" 2>/dev/null || true
  ros2 daemon stop >/dev/null 2>&1 || true
}

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required. Install tmux and retry."
  exit 1
fi

safe_source /opt/ros/humble/setup.bash

if [[ -f "$WS/.env" ]]; then
  set -a
  safe_source "$WS/.env"
  set +a
  echo "Loaded $WS/.env"
fi

if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "No install setup found, running a quick build..."
  cd "$WS"
  colcon build --symlink-install
fi

safe_source "$WS/install/setup.bash"

echo "Initializing inventory DB..."
python3 "$WS/src/cognitive_amr/scripts/init_database.py" \
  --db "$WS/src/cognitive_amr/config/inventory.db" >/dev/null

tmux kill-session -t "$SESSION" 2>/dev/null || true
cleanup_stale_sim

tmux new-session -d -s "$SESSION" -n gazebo \
  "source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && \
   ros2 launch linorobot2_gazebo gazebo.launch.py \
     world_name:=tugbot_style_ai_warehouse \
     spawn_x:=$SPAWN_X spawn_y:=$SPAWN_Y spawn_z:=$SPAWN_Z spawn_yaw:=$SPAWN_YAW \
     gui:=false; exec bash"

tmux new-window -t "$SESSION" -n navigation \
  "source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && \
   ros2 launch linorobot2_navigation navigation.launch.py \
     sim:=true \
     map:=$WS/src/linorobot2/linorobot2_navigation/maps/warehouse_map.yaml \
     initial_pose_x:=$INITIAL_POSE_X initial_pose_y:=$INITIAL_POSE_Y initial_pose_yaw:=$INITIAL_POSE_YAW & \
   NAV_PID=\$!; \
   for i in \$(seq 1 45); do \
     if ros2 node list 2>/dev/null | grep -q '^/amcl$'; then \
       ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \"{header:{frame_id: map, stamp:{sec: 0, nanosec: 0}}, pose:{pose:{position:{x: $INITIAL_POSE_X, y: $INITIAL_POSE_Y, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: $INITIAL_POSE_QZ, w: $INITIAL_POSE_QW}}}}\"; \
       break; \
     fi; \
     sleep 1; \
   done; \
   wait \$NAV_PID; exec bash"

tmux new-window -t "$SESSION" -n cognitive \
  "source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && \
   ros2 launch cognitive_amr cognitive_amr.launch.py; exec bash"

tmux new-window -t "$SESSION" -n foxglove \
  "source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && \
   ros2 run foxglove_bridge foxglove_bridge \
     --ros-args --params-file $WS/src/cognitive_amr/config/foxglove_params.yaml; exec bash"

tmux new-window -t "$SESSION" -n web_ui \
  "source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && \
   ros2 run cognitive_amr operator_web_ui; exec bash"

echo
echo "Stack started in tmux session: $SESSION"
echo "Attach with: tmux attach -t $SESSION"
echo "Windows: gazebo | navigation | cognitive | foxglove | web_ui"
echo "Spawn world pose: ($SPAWN_X, $SPAWN_Y, yaw=$SPAWN_YAW)"
echo "AMCL initial pose on saved map: ($INITIAL_POSE_X, $INITIAL_POSE_Y, yaw=$INITIAL_POSE_YAW)"
echo "AMCL seeding: /initialpose auto-published after nav startup"
