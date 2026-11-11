#!/usr/bin/env bash
# Experimental world: straight bent 1 -> BCR arm -> circular bent 2, in one command.
#   1. The PyQt Product Order window opens with a tab for each bent (Bent 1 - pick, Bent 2 - place):
#      choose the products and where each one goes, then press "Start simulation".
#   2. Gazebo (exp_straight_circular.sdf), both conveyors, the BCR arm, the cameras, the attacher and
#      vision pick-and-place start; the Product Order window opens again for live orders.
#   Ctrl+C stops everything.
# The arm picks from bent 1 at the catch station and places on bent 2 or drops into a bin.
# Straight bent 1 ends over a collection box: products the arm does not pick fall into it.
#
# Usage:  ./run_exp_straight_to_circular.sh [options] [launch arguments]
#   --no-select        skip the window, start with this world's last selection
#   --world-products   skip the window, start with the world's own products
#   --build            colcon build factory_bringup first
#   launch arguments are passed on, e.g.  show_window:=false  order_gui:=false  motion_type:=cycle

# each world keeps its own last selection, so --no-select never mixes worlds
export FACTORY_ORDER_FILE="${FACTORY_ORDER_FILE:-$HOME/.ros/factory_order_straight_circular.json}"
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/run_factory.sh" --world straight-circular "$@"
