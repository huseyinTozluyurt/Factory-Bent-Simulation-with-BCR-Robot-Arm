#!/usr/bin/env bash
# Factory simulation in one command.
#
#   1. The Product Order window opens: choose the products for bent 1 (or the world's own
#      products) and press "Start simulation".
#   2. Gazebo + both bents + BCR arm + attacher + cameras + pick-and-place start together
#      (factory_full.launch.py); your products are put on bent 1 the same distance apart and
#      the arm starts picking them. The Product Order window opens again for live orders.
#   Ctrl+C in this terminal stops everything.
#
# Usage:  ./run_factory.sh [options] [launch arguments]
#   --no-select        skip the selection window, start with the last selection
#   --world-products   skip the selection window, start with the world's own products
#   --build            colcon build factory_bringup first
#   --world NAME       dual (default): two circular bents; circular: one circular bent;
#                      straight: the straight conveyor. The last two have no bent 2: the arm
#                      picks only the products set to a bin, the others stay on the belt.
#                      Experimental two-conveyor worlds (bent 1 -> arm -> bent 2): straight-circular,
#                      circular-straight, straight-straight (circular-circular = dual).
#   launch arguments are passed on, e.g.  show_window:=false  order_gui:=false  motion_type:=cycle
#
# The separate-terminal way (T1 / T2 / T3 in COMMANDS.md) still works as before.

set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$PROJECT_DIR/../.." && pwd)"
[ -f "$WS_DIR/install/setup.bash" ] || WS_DIR="$HOME/ros2_ws"
ROS_DISTRO_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
WORLDS_DIR="$PROJECT_DIR/my_factory_worlds/worlds"
ORDER_FILE="${FACTORY_ORDER_FILE:-$HOME/.ros/factory_order.json}"

select=true; world_products=false; build=false; launch_args=(); world=dual
while [ $# -gt 0 ]; do
  case "$1" in
    --no-select) select=false ;;
    --world-products) select=false; world_products=true ;;
    --build) build=true ;;
    --world) shift; world="${1:-}" ;;
    --world=*) world="${1#--world=}" ;;
    -h|--help) sed -n '2,23p' "$0"; exit 0 ;;
    *) launch_args+=("$1") ;;
  esac
  shift
done
# world file and Gazebo world name (as in product_catalog.WORLD_FILES); every node reads the
# name from FACTORY_WORLD_NAME and sets its belt layout from it
case "$world" in
  dual) WORLD="$WORLDS_DIR/dual_circular_factory_conveyor.sdf"; FACTORY_WORLD_NAME=dual_circular_factory_world ;;
  circular) WORLD="$WORLDS_DIR/circular_factory_conveyor.sdf"; FACTORY_WORLD_NAME=circular_factory_world ;;
  straight) WORLD="$WORLDS_DIR/factory_conveyor.sdf"; FACTORY_WORLD_NAME=factory_world ;;
  straight-circular) WORLD="$WORLDS_DIR/exp_straight_circular.sdf"; FACTORY_WORLD_NAME=exp_straight_circular_world ;;
  circular-straight) WORLD="$WORLDS_DIR/exp_circular_straight.sdf"; FACTORY_WORLD_NAME=exp_circular_straight_world ;;
  straight-straight) WORLD="$WORLDS_DIR/exp_straight_straight.sdf"; FACTORY_WORLD_NAME=exp_straight_straight_world ;;
  circular-circular) WORLD="$WORLDS_DIR/dual_circular_factory_conveyor.sdf"; FACTORY_WORLD_NAME=dual_circular_factory_world ;;
  *) echo "Unknown world '$world' (use dual, circular, straight, straight-circular, circular-straight,"
     echo "straight-straight or circular-circular)."; exit 1 ;;
esac
export FACTORY_WORLD_NAME

set +e                                   # ROS setup scripts are not written for set -e
source "$ROS_DISTRO_SETUP"
set -e
if $build; then
  (cd "$WS_DIR" && colcon build --packages-select factory_bringup --symlink-install)
fi
set +e
source "$WS_DIR/install/setup.bash"
set -e
share="$(ros2 pkg prefix factory_bringup 2>/dev/null)/share/factory_bringup"
if [ ! -f "$share/launch/factory_full.launch.py" ] || ! ros2 pkg executables factory_bringup | grep -q order_gui; then
  echo "factory_bringup is not built completely (factory_full.launch.py or order_gui missing). Rebuild it:"
  echo "  cd $WS_DIR && rm -rf build/factory_bringup install/factory_bringup"
  echo "  colcon build --packages-select factory_bringup --symlink-install"
  exit 1
fi

if pgrep -f "gz sim" > /dev/null; then
  echo "Gazebo is already running (another simulation?)."
  read -r -p "Close it and continue? [y/N] " answer
  if [[ "$answer" =~ ^[Yy] ]]; then
    pkill -f "gz sim" || true
    sleep 2
  else
    echo "Stopped. Close the other simulation first."
    exit 1
  fi
fi

if $select; then
  echo "Choose the products in the Product Order window, then press Start simulation ..."
  if ! ros2 run factory_bringup order_gui --select --output "$ORDER_FILE"; then
    echo "Selection window closed without starting. Nothing was started."
    exit 0
  fi
  order_arg="order_file:=$ORDER_FILE"
elif $world_products; then
  order_arg="order_file:=none"
elif [ -f "$ORDER_FILE" ]; then
  order_arg="order_file:=$ORDER_FILE"
else
  echo "No earlier selection ($ORDER_FILE): starting with the world's own products."
  order_arg="order_file:=none"
fi

# arm movement chosen in the window ("motion_type" in the file's settings); a motion_type:=...
# given on the command line comes later and wins
motion_arg=()
if [ "$order_arg" != "order_file:=none" ]; then
  motion="$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1])).get("settings", {}).get("motion_type", ""))' \
            "$ORDER_FILE" 2>/dev/null || true)"
  [ -n "$motion" ] && motion_arg=("motion_type:=$motion")
fi

echo "Starting the simulation in the $world world ($order_arg ${motion_arg[*]}). Ctrl+C stops everything."
exec ros2 launch factory_bringup factory_full.launch.py "$order_arg" \
  "factory_world_path:=$WORLD" "${motion_arg[@]}" "${launch_args[@]}"
