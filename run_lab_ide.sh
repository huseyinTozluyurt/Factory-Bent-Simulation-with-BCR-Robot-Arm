#!/usr/bin/env bash
# Vision Lab IDE: a desktop app to create a lab project for a world, edit its script, build and run it.
#
#   ./run_lab_ide.sh                    open the IDE (the last project, or none)
#   ./run_lab_ide.sh ~/vision_lab_projects/my_lab    open that lab project
#
# In the IDE: File > New project (pick a world: pipeline, pipeline-tree, dual, straight, ...; it writes a
# starter lab_<name>.py for that world), edit it, Build (Ctrl+B), Run (F5: Gazebo + the lab), Stop (Shift+F5).
# Run = ./run_vision_lab.sh sim --world W --drawn-cameras  and then  ./run_vision_lab.sh run lab_<name>.py --world W
# The IDE runs from this folder's source (no colcon build needed to start it); after a build also:
#   ros2 run factory_bringup lab_ide
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FACTORY_PROJECT_DIR="$PROJECT_DIR"
if ! python3 -c "import PyQt5.QtWidgets" 2>/dev/null; then
  echo "PyQt5 is missing:  sudo apt install python3-pyqt5"
  exit 1
fi
export PYTHONPATH="$PROJECT_DIR/factory_bringup${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m factory_bringup.lab_ide "$@"
