#!/usr/bin/env bash
# Single circular conveyor factory + BCR arm, in one command.
#   1. The PyQt Product Order window opens for this world's single circular bent: choose the products and
#      where each one goes, then press "Start simulation".
#   2. Gazebo (circular_factory_conveyor.sdf), the conveyor, the BCR arm, the cameras, the attacher and
#      vision pick-and-place start; the Product Order window opens again for live orders.
#   Ctrl+C stops everything.
# One bent: the arm picks the products set to a bin; the others stay on the belt.
#
# Usage:  ./run_single_circular.sh [options] [launch arguments]
#   --no-select        skip the window, start with this world's last selection
#   --world-products   skip the window, start with the world's own products
#   --build            colcon build factory_bringup first
#   launch arguments are passed on, e.g.  show_window:=false  order_gui:=false  motion_type:=cycle

# each world keeps its own last selection, so --no-select never mixes worlds
export FACTORY_ORDER_FILE="${FACTORY_ORDER_FILE:-$HOME/.ros/factory_order_circular.json}"
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/run_factory.sh" --world circular "$@"
