#!/bin/bash
# Usage:
#   source run.sh          → enable tab completion in current shell
#   ./run.sh build         → colcon build
#   ./run.sh source        → source install/setup.bash
#   ./run.sh launch        → ros2 launch

# ── Tab Completion ────────────────────────────────────────────────────────────
_run_completion() {
  local cur opts
  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"
  opts="build launch nmpc_controller"
  if [[ $COMP_CWORD -eq 1 ]]; then
    COMPREPLY=( $(compgen -W "${opts}" -- "$cur") )
  fi
}
complete -F _run_completion run.sh ./run.sh

# ── If script is being sourced, only register completion (don't execute) ──────
[[ "${BASH_SOURCE[0]}" != "${0}" ]] && return 0

# ── Main logic ────────────────────────────────────────────────────────────────
ACTION="$1"

if [[ "$ACTION" == b* ]]; then
  echo "[BUILD] Running colcon build..."
#   colcon build --symlink-install
    colcon build

elif [[ "$ACTION" == l* ]]; then
  echo "[LAUNCH] Launching vehicle.launch.py..  ."
  source install/setup.bash
  ros2 launch gazebo_ackermann_steering_vehicle vehicle.launch.py

elif [[ "$ACTION" == n* ]]; then
  echo "[RUN] Running nmpc_controller node..."
  source install/setup.bash
  ros2 run gazebo_ackermann_steering_vehicle nmpc_controller
else
  echo "Usage: $0 [build|launch|nmpc_controller]"
  exit 1
fi
