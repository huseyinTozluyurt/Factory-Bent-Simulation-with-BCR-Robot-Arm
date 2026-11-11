#!/usr/bin/env bash
# Vision lab: develop OpenCV / YOLO detection and drive the BCR arm (bent to bent, trash, recycling)
# from one main(lab) function, in every world.
#
#   ./run_vision_lab.sh sim [--world W] [run_factory.sh options]   terminal 1: the simulation, started once
#   ./run_vision_lab.sh run [LAB] [vision_lab options]             terminal 2: a lab file (labs/lab_LAB.py) or
#                                                                  vision_lab_main.py; edit, run again
#   ./run_vision_lab.sh new NAME [--template T] [--world W]        create labs/lab_NAME.py from a template
#   ./run_vision_lab.sh list | templates                           the lab files / the templates
#   ./run_vision_lab.sh [--world W] [vision_lab options]           both: simulation in the background, then the lab
#   ./run_vision_lab.sh offline [options]                          no simulation: images or synthetic scenes
#   ./run_vision_lab.sh worlds | detectors                         what is available
#
# Worlds (W): dual (default), circular, straight, straight-circular, circular-straight, straight-straight,
#             pipeline (pipeline_factory_lab.sdf: 12 arm stations; the lab works at --station A1 ... A12)
# Pipeline world:
#   ./run_vision_lab.sh sim --world pipeline        Gazebo with the lab version of pipeline_factory.sdf
#   ./run_vision_lab.sh run LAB --station A2        a lab at arm A2 (found running; or add --world pipeline)
#   ./run_vision_lab.sh run LAB --fake --station A6 the same without Gazebo (drawn camera images)
#   ./run_vision_lab.sh --world pipeline [LAB ...]  both: Gazebo in the background, then the lab
# "sim" is run_factory.sh without the standard detector and pick-and-place controller (the lab brings its
# own): run_factory.sh --world W ... run_detector:=false run_controller:=false. Its order window, --no-select,
# --world-products and --build work as there.
#
# vision_lab options (run / both):
#   --script FILE        Python file with main(lab) (default: vision_lab_main.py in this folder),
#                        or an installed example: bent_to_bent, sort_into_bins, custom_detector, yolo_sort
#   --detector NAME      opencv_contours (default), opencv_color, opencv_bgsub, yolo, file.py:Class, module:Class
#   --opt KEY=VALUE      detector option, repeatable (e.g. --opt model=best.pt --opt conf=0.4)
#   --arm KEY=VALUE      vision_pick_place parameter, repeatable (e.g. --arm motion_type=cycle)
#   --record DIR         save auto-labelled YOLO training frames into DIR
#   --dry-run            plan and log, do not move the arm      --no-window   no Vision Lab window
#   --station A1..A12    pipeline: the arm station      --fake   pipeline without Gazebo
#   --no-feed            pipeline: no automatic products --belt-speed V   pipeline belt speed (m/s)
# Examples:
#   ./run_vision_lab.sh sim --world straight-circular --world-products
#   ./run_vision_lab.sh new color_sorting --template sort && ./run_vision_lab.sh run color_sorting
#   ./run_vision_lab.sh run 02_bent_to_bent
#   ./run_vision_lab.sh offline --lab 05_my_opencv_detector --synthetic 10 --world all
#   ./run_vision_lab.sh run --detector yolo --opt model=$HOME/runs/detect/train/weights/best.pt
#   ./run_vision_lab.sh offline --synthetic 20 --world all --detector opencv_contours,opencv_color
#   ./run_vision_lab.sh run 08_pipeline_sorting --fake

set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$PROJECT_DIR/../.." && pwd)"
[ -f "$WS_DIR/install/setup.bash" ] || WS_DIR="$HOME/ros2_ws"

ros_env() {
  set +e
  source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
  source "$WS_DIR/install/setup.bash"
  set -e
  if ! ros2 pkg executables factory_bringup 2>/dev/null | grep -q vision_lab; then
    echo "factory_bringup has no vision_lab executable yet. Rebuild it:"
    echo "  cd $WS_DIR && colcon build --packages-select factory_bringup --symlink-install"
    exit 1
  fi
}

export VISION_LAB_DIR="${VISION_LAB_DIR:-$PROJECT_DIR/labs}"   # lab files: labs/lab_<name>.py

lab_run() {          # vision_lab run: a lab by name, --script FILE, or this folder's vision_lab_main.py
  local args=("$@") has_script=false
  for a in "${args[@]}"; do [[ "$a" == --script || "$a" == --script=* ]] && has_script=true; done
  [[ ${#args[@]} -gt 0 && "${args[0]}" != -* ]] && has_script=true          # run <lab name>
  $has_script || args=(--script "$PROJECT_DIR/vision_lab_main.py" "${args[@]}")
  ros2 run factory_bringup vision_lab run "${args[@]}"
}

is_pipeline() { [[ "$1" == pipeline* ]]; }

pipeline_sim() {     # Gazebo with pipeline_factory_lab.sdf (no ROS nodes: the lab drives it over gz-transport)
  local world_file="$PROJECT_DIR/my_factory_worlds/worlds/pipeline_factory_lab.sdf"
  set +e
  source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
  source "$WS_DIR/install/setup.bash"
  set -e
  local share
  share="$(ros2 pkg prefix bcr_arm_description 2>/dev/null)/share"
  if [ ! -d "$share/bcr_arm_description/meshes" ]; then
    echo "bcr_arm_description is not built in $WS_DIR (needed for the arm meshes)."
    exit 1
  fi
  if [ ! -f "$world_file" ]; then
    echo "Building $world_file ..."
    (cd "$PROJECT_DIR/my_factory_worlds/tools" && python3 make_pipeline_lab_world.py)
  fi
  if ! python3 -c "import gz.transport13, gz.msgs10" 2>/dev/null; then
    echo "Note: the lab needs the gz-transport Python bindings: sudo apt install python3-gz-transport13"
  fi
  export GZ_SIM_RESOURCE_PATH="$share${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
  if pgrep -f "gz sim" > /dev/null; then
    echo "Gazebo is already running (another simulation?). Close it first."
    exit 1
  fi
  echo "Pipeline lab world in Gazebo; start a lab in another terminal:  $0 run LAB --station A1"
  exec gz sim -r "$world_file" "$@"
}

cmd="${1:-}"
case "$cmd" in
  -h|--help) sed -n '2,42p' "$0"; exit 0 ;;
  sim)
    shift
    world=dual; rest=()
    while [ $# -gt 0 ]; do
      case "$1" in
        --world) shift; world="${1:-dual}" ;;
        --world=*) world="${1#--world=}" ;;
        *) rest+=("$1") ;;
      esac
      shift
    done
    is_pipeline "$world" && pipeline_sim "${rest[@]}"
    export FACTORY_ORDER_FILE="${FACTORY_ORDER_FILE:-$HOME/.ros/factory_order_${world//-/_}.json}"
    exec bash "$PROJECT_DIR/run_factory.sh" --world "$world" "${rest[@]}" run_detector:=false run_controller:=false
    ;;
  run)
    shift; ros_env; lab_run "$@" ;;
  offline|worlds|detectors|new|list|templates)
    shift; ros_env; exec ros2 run factory_bringup vision_lab "$cmd" "$@" ;;
  *)
    # both: the simulation (world's own products, no order window) in the background, then the lab
    world=dual; rest=()
    while [ $# -gt 0 ]; do
      case "$1" in
        --world) shift; world="${1:-dual}" ;;
        --world=*) world="${1#--world=}" ;;
        *) rest+=("$1") ;;
      esac
      shift
    done
    ros_env
    if pgrep -f "gz sim" > /dev/null; then
      echo "Gazebo is already running: use  $0 run ...  to start only the lab."
      exit 1
    fi
    log="${TMPDIR:-/tmp}/vision_lab_sim.log"
    echo "Starting the $world simulation in the background (log: $log) ..."
    if is_pipeline "$world"; then
      bash "$0" sim --world pipeline > "$log" 2>&1 &
    else
      bash "$0" sim --world "$world" --world-products order_gui:=false > "$log" 2>&1 &
    fi
    sim_pid=$!
    trap 'echo "Stopping the simulation ..."; kill -INT $sim_pid 2>/dev/null; sleep 3; pkill -f "gz sim" 2>/dev/null || true' EXIT
    for _ in $(seq 1 90); do
      if is_pipeline "$world"; then
        gz topic -l 2>/dev/null | grep -q '^/pipeline/A1/camera$' && break
      else
        ros2 topic list 2>/dev/null | grep -q '^/overhead_camera/image$' && break
      fi
      kill -0 $sim_pid 2>/dev/null || { echo "The simulation stopped; see $log"; exit 1; }
      sleep 1
    done
    sleep 5                                  # the attacher and the bridges come up a little later
    lab_run --world "$world" "${rest[@]}"
    ;;
esac
