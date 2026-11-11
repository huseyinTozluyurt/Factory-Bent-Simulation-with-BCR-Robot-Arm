#!/usr/bin/env bash
# Pipeline factory (layout only): opens pipeline_factory.sdf in Gazebo. 13 conveyors (straight
# transport lines S1-S9, circular selection loops C1-C4), 12 BCR arms (A1-A12) posed over their
# pick points and 5 dock boxes. Nothing moves yet: this is for reviewing the layout.
# The layout is defined in factory_bringup/pipeline_layout.py. Ctrl+C closes Gazebo.
#
# Usage:  ./run_pipeline_factory.sh [extra gz sim options]

set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$PROJECT_DIR/../.." && pwd)"
[ -f "$WS_DIR/install/setup.bash" ] || WS_DIR="$HOME/ros2_ws"
WORLD="$PROJECT_DIR/my_factory_worlds/worlds/pipeline_factory.sdf"

set +e                                   # ROS setup scripts are not written for set -e
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "$WS_DIR/install/setup.bash"
set -e
# the arms' meshes are model://bcr_arm_description/meshes/...: Gazebo finds them under the share folder
share="$(ros2 pkg prefix bcr_arm_description 2>/dev/null)/share"
if [ ! -d "$share/bcr_arm_description/meshes" ]; then
  echo "bcr_arm_description is not built in $WS_DIR (needed for the arm meshes)."
  exit 1
fi
export GZ_SIM_RESOURCE_PATH="$share${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
if pgrep -f "gz sim" > /dev/null; then
  echo "Gazebo is already running (another simulation?). Close it first."
  exit 1
fi
echo "Opening the pipeline factory layout in Gazebo. Ctrl+C closes it."
exec gz sim "$WORLD" "$@"
