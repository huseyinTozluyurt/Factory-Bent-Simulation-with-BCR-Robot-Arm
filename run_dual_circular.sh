#!/usr/bin/env bash
# Dual circular conveyor factory + BCR arm, in one command.
#   1. The PyQt Product Order window opens for this world's two bents (tabs Bent 1 - pick, Bent 2 - place): choose the products and
#      where each one goes, then press "Start simulation".
#   2. Gazebo (dual_circular_factory_conveyor.sdf), the conveyor, the BCR arm, the cameras, the attacher and
#      vision pick-and-place start; the Product Order window opens again for live orders.
#   Ctrl+C stops everything.
# Bent 1 (pick) -> BCR arm -> bent 2 (place) or the trash / recycling bins.
#
# Usage:  ./run_dual_circular.sh [options] [launch arguments]
#   --no-select        skip the window, start with this world's last selection
#   --world-products   skip the window, start with the world's own products
#   --build            colcon build factory_bringup first
#   launch arguments are passed on, e.g.  show_window:=false  order_gui:=false  motion_type:=cycle

# each world keeps its own last selection, so --no-select never mixes worlds
export FACTORY_ORDER_FILE="${FACTORY_ORDER_FILE:-$HOME/.ros/factory_order_dual.json}"
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/run_factory.sh" --world dual "$@"
