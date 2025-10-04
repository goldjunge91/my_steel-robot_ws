#!/usr/bin/env bash
# Simple helper to diagnose controller_manager and (optionally) spawn controllers
# Usage: ./scripts/check_and_spawn_controllers.sh [--spawn controller1 controller2 ...] [--test-cmd]

set -euo pipefail

WORKDIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WORKDIR"

SPAWN=false
TEST_CMD=false
CONTROLLERS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --spawn)
      SPAWN=true
      shift
      while [[ $# -gt 0 && "$1" != "--test-cmd" && "$1" != "--spawn" ]]; do
        CONTROLLERS+=("$1")
        shift
      done
      ;;
    --test-cmd)
      TEST_CMD=true
      shift
      ;;
    -h|--help)
      echo "Usage: $0 [--spawn controller1 controller2 ...] [--test-cmd]"
      exit 0
      ;;
    *)
      echo "Unknown arg: $1" >&2
      exit 2
      ;;
  esac
done

echo "Waiting for /controller_manager/list_controllers service..."
if ! ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers >/dev/null 2>&1; then
  # wait for up to 30s
  timeout=30
  until ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers >/dev/null 2>&1; do
    sleep 1
    ((timeout--)) || { echo "Timeout waiting for controller_manager" >&2; exit 1; }
  done
fi

echo "Listing controllers via service..."
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers

if $SPAWN && [[ ${#CONTROLLERS[@]} -gt 0 ]]; then
  CM="controller_manager"
  echo "Spawning controllers: ${CONTROLLERS[*]}"
  for c in "${CONTROLLERS[@]}"; do
    echo "Spawning $c..."
    # If controller already exists, attempt configure -> switch; otherwise use spawner
    LIST_OUT=$(ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers 2>/dev/null || true)
    if echo "$LIST_OUT" | grep -q "name='${c}'"; then
      # Controller present; check state
      if echo "$LIST_OUT" | grep -q "name='${c}'.*state='active'"; then
        echo "Controller ${c} already active; skipping."
        elif echo "$LIST_OUT" | grep -q "name='${c}'.*state='inactive'"; then
        echo "Controller ${c} present and inactive: attempting configure + start"
        # configure_controller (use JSON payload) with retries for transient failures
        payload_config="{\"name\": \"${c}\"}"
        max_retries=5
        attempt=0
        configured=false
        while [[ $attempt -lt $max_retries ]]; do
          ((attempt++))
          echo "Configure attempt ${attempt}/${max_retries} for ${c}..."
          CONF_OUT=$(ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController "${payload_config}" 2>&1 || true)
          if echo "$CONF_OUT" | grep -q "ConfigureController_Response(ok=True)"; then
            configured=true
            break
          fi
          sleep 1
        done
        if $configured; then
          echo "Configured ${c} successfully on attempt ${attempt}. Attempting to start."
          payload_switch="{\"start_controllers\": [\"${c}\"], \"stop_controllers\": [], \"strictness\": 2}"
          SWITCH_OUT=$(ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "${payload_switch}" 2>&1 || true)
          if echo "$SWITCH_OUT" | grep -q "SwitchController_Response(ok=True)"; then
            echo "Started ${c}."
          else
            echo "Failed to start ${c} after configure. Response:" >&2
            echo "$SWITCH_OUT" >&2
          fi
        else
          echo "Failed to configure ${c} after ${max_retries} attempts; last response:" >&2
          echo "$CONF_OUT" >&2
          echo "Collecting diagnostics to /tmp..." >&2
          TS=$(date +%Y%m%d_%H%M%S)
          OUTF="/tmp/controller_diag_${c}_${TS}.log"
          {
            echo "===== list_controllers =====";
            ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers || true;
            echo "\n===== node info /controller_manager =====";
            ros2 node info /controller_manager || true;
            echo "\n===== params /controller_manager =====";
            ros2 param list /controller_manager || true;
            echo "\n===== topic list -v =====";
            ros2 topic list -v || true;
            echo "\n===== CONF_OUT =====";
            echo "$CONF_OUT";
          } > "$OUTF" 2>&1 || true
          echo "Diagnostics written to $OUTF" >&2
          echo "Falling back to spawner." >&2
          ros2 run controller_manager spawner "$c" -c "$CM" || echo "Spawner returned non-zero for $c" >&2
        fi
        
      else
        # present but unknown state - try spawner as fallback
        ros2 run controller_manager spawner "$c" -c "$CM" || echo "Spawner returned non-zero for $c" >&2
      fi
    else
      # Controller not present, use spawner
      ros2 run controller_manager spawner "$c" -c "$CM" || echo "Spawner returned non-zero for $c" >&2
    fi
  done
fi

if $TEST_CMD; then
  echo "Publishing a short test /cmd_vel..."
  # publish on /cmd_vel for 1s at 10Hz
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once || true
  echo "Published test cmd_vel (one message)."
fi

echo "Done."
