"""Vision Lab IDE: a small PyQt5 desktop app to create, edit, build and run vision-lab scripts.

    ./run_lab_ide.sh                      (project folder; no colcon build needed to start it)
    ros2 run factory_bringup lab_ide      (after colcon build)

Modules (world_info and generator have no Qt or ROS imports and are usable on their own):
    world_info   the worlds the lab knows and what is in each (stations, arms, bents, bins, docks)
    generator    writes a starter lab script for a world
    app          the window: projects, world picker, editor, build / simulation / lab panes
"""
