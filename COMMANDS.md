# Factory Bent Project: Terminal Commands

ROS 2 Jazzy + Gazebo Harmonic. Workspace: `~/ros2_ws`, project: `~/ros2_ws/src/factory_bent_project`.

## 0. Everything with one command (optional)

`run_factory.sh` in the project folder replaces terminals T1, T2, T3 and T4:

1. The Product Order window opens first (nothing else is running yet). Choose the products for
   bent 1, the distance between them, and so on, the same way as in section 6c. Then press
   **Start simulation with this order**, or **Start simulation with the world's own products**.
   The window opens with your last selection. Closing it without pressing Start starts nothing.
2. Gazebo, both bents, the BCR arm, the attacher, the cameras and the pick-and-place controller
   start together (`factory_full.launch.py`). As soon as Gazebo is up, your products are put on
   bent 1, the same distance apart, and then the arm starts picking them.
3. After ~8 s the Product Order window opens again for live orders and bent stop / start.
   Ctrl+C in that terminal stops everything.

One-time setup:

```bash
cd ~/ros2_ws
colcon build --packages-select factory_bringup --symlink-install   # installs factory_full.launch.py
chmod +x ~/ros2_ws/src/factory_bent_project/run_factory.sh
```

Run:

```bash
~/ros2_ws/src/factory_bent_project/run_factory.sh                    # choose products, then start
~/ros2_ws/src/factory_bent_project/run_factory.sh --no-select        # start again with the last selection
~/ros2_ws/src/factory_bent_project/run_factory.sh --world-products   # the world's 8 products, no window
~/ros2_ws/src/factory_bent_project/run_factory.sh --build            # colcon build first
~/ros2_ws/src/factory_bent_project/run_factory.sh show_window:=false order_gui:=false motion_type:=cycle
```

Launch arguments it passes on: `show_window`, `order_gui` (true), `wait_at`, `motion_type`, and
`vision_delay` (8.0 s before the cameras and controller start; raise it if Gazebo loads slowly).
The selection is saved in `~/.ros/factory_order.json` (change it with `FACTORY_ORDER_FILE=...`).
If Gazebo is already running, the script asks before it closes it.

The same without the script (after choosing in the window yourself):

```bash
ros2 run factory_bringup order_gui --select --output ~/.ros/factory_order.json
ros2 launch factory_bringup factory_full.launch.py order_file:=$HOME/.ros/factory_order.json \
  factory_world_path:=$HOME/ros2_ws/src/factory_bent_project/my_factory_worlds/worlds/dual_circular_factory_conveyor.sdf
```

Optional app-menu entry ("Factory Simulation", opens a terminal and runs the script):

```bash
cd ~/ros2_ws/src/factory_bent_project
sed "s|@PROJECT@|$PWD|" factory_simulation.desktop > ~/.local/share/applications/factory_simulation.desktop
```

The separate terminals below still work as before. Backups for this change are named `.bak_before_onecmd`
(the attacher, `order_gui.py` and `bcr_endpoint_object_attacher.launch.py`).

## 1. Environment setup (every new terminal)

```bash
export GZ_VERSION=harmonic
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Optional: put this in `~/.bashrc` as a shortcut, then just type `fb` in each terminal.

```bash
echo 'alias fb="export GZ_VERSION=harmonic && source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && cd ~/ros2_ws"' >> ~/.bashrc
source ~/.bashrc
```

## 2. Build

```bash
cd ~/ros2_ws

# Only the bringup package (fast, use after editing the attacher/UI)
colcon build --packages-select factory_bringup
# Python edits picked up without rebuilding each time:
colcon build --packages-select factory_bringup --symlink-install

# Whole workspace (BCR arm packages + factory_bringup)
colcon build --symlink-install

# Install missing dependencies listed in package.xml files
rosdep install --from-paths src --ignore-src -r -y

# Clean rebuild of one package (optional, only if a build gets stuck).
# This deletes ONLY colcon's generated output in ~/ros2_ws/build and ~/ros2_ws/install,
# NOT your source code in ~/ros2_ws/src/factory_bent_project/factory_bringup.
# The next colcon build recreates both folders.
rm -rf ~/ros2_ws/build/factory_bringup ~/ros2_ws/install/factory_bringup
colcon build --packages-select factory_bringup

source install/setup.bash   # always re-source after building
```

## 3. Launch the worlds (Gazebo only)

```bash
W=~/ros2_ws/src/factory_bent_project/my_factory_worlds/worlds

gz sim $W/factory_conveyor.sdf                 # single straight bent (world: factory_world)
gz sim $W/circular_factory_conveyor.sdf        # single circular bent (world: circular_factory_world)
gz sim $W/dual_circular_factory_conveyor.sdf   # dual circular bent (world: dual_circular_factory_world)

gz sim -r $W/factory_conveyor.sdf              # -r = start running immediately
```

Gazebo GUI not appearing (multicast fix):

```bash
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4
```

## 4. BCR arm + dual bent (pick-and-place setup)

Run each in its own terminal (after step 1).

```bash
# T1: Gazebo + dual circular world + BCR arm + camera bridge
ros2 launch factory_bringup factory_with_bcr_arm.launch.py
# a different world path:
ros2 launch factory_bringup factory_with_bcr_arm.launch.py \
  factory_world_path:=$HOME/ros2_ws/src/factory_bent_project/my_factory_worlds/worlds/dual_circular_factory_conveyor.sdf

# T2: endpoint object attacher (attach/release services)
ros2 launch factory_bringup bcr_endpoint_object_attacher.launch.py

# T3: arm control UI (PyQt5)
ros2 launch factory_bringup bcr_arm_simple_ui.launch.py
# or: ros2 run factory_bringup bcr_arm_simple_ui
```

Note: `factory_with_bcr_arm.launch.py` defaults to `/home/huseyin/...`; pass `factory_world_path:=` if your home directory differs.

## 5. Attacher services (without the UI)

```bash
ros2 service call /move_object_to_endpoint_pick std_srvs/srv/Trigger    # put pickup_box at the pick pose
ros2 service call /attach_object_to_endpoint std_srvs/srv/Trigger       # snap box top to gripper_tip
ros2 service call /release_object_from_endpoint std_srvs/srv/Trigger    # drop from the carried pose
ros2 service call /get_endpoint_attach_status std_srvs/srv/Trigger      # attached? last pose?
```

Tune at runtime:

```bash
ros2 param list /bcr_endpoint_object_attacher
ros2 param get  /bcr_endpoint_object_attacher contact_gap
ros2 param set  /bcr_endpoint_object_attacher contact_gap 0.01
ros2 param set  /bcr_endpoint_object_attacher attach_offset_x 0.02
ros2 param set  /bcr_endpoint_object_attacher base_yaw 1.5708
ros2 param set  /bcr_endpoint_object_attacher endpoint_frame gripper_tip
```

Check the fast gz-transport path is available (otherwise the attacher falls back to the slower CLI):

```bash
python3 -c "from gz.transport13 import Node; from gz.msgs10.pose_pb2 import Pose; print('gz python bindings OK')"
# if missing:
sudo apt install python3-gz-transport13 python3-gz-msgs10
```

## 6. Camera and object detection

```bash
# Camera bridge (only needed when not using factory_with_bcr_arm.launch.py)
ros2 run ros_gz_bridge parameter_bridge /overhead_camera/image@sensor_msgs/msg/Image@gz.msgs.Image

cd ~/ros2_ws/src/factory_bent_project
python3 camera_viewer.py          # raw camera window
python3 test_image_sub.py         # prints image size/encoding (bridge check)
python3 object_detection_base.py  # simple threshold + contours
python3 object_detection.py       # tracking, counting, measurements -> measurement_logs/

# Alternative image view
ros2 run rqt_image_view rqt_image_view /overhead_camera/image
```

## 6b. Vision pick and place (ring 1 -> ring 2)

Default is **station mode with two cameras**:

1. The arm goes to the middle of the ring 1 section nearest to it (-2.58, 0) and stops there.
2. The overhead camera tracks the products riding the bent (top view: x, y).
3. The side camera (new `side_camera` model, low at (-1.50, 0, 1.80), looking straight at the
   station) must see an object standing on the belt at the station, at the same place where the
   overhead camera has a track (side view: y along the belt and the object's height). That is the
   cross-check. If the cameras disagree, the object is let through.
4. The product is stopped on the bent (`/hold_object_for_pick`), the arm comes down until the
   gripper is 2 cm above the product's top, then attaches, lifts and puts it on ring 2.
   If anything fails after the stop, the arm goes back up and the product continues along the bent.

After pulling this version, rebuild once (adds the `side_camera_checker` and `release_zone_monitor` executables):

```bash
cd ~/ros2_ws
colcon build --packages-select factory_bringup --symlink-install
source install/setup.bash
ros2 pkg executables factory_bringup | grep -E "side_camera_checker|release_zone_monitor"   # both listed
```

Restart Gazebo (T1) so the world loads with the side camera.

```bash
# T1: Gazebo + dual bent + BCR arm + overhead camera bridge
ros2 launch factory_bringup factory_with_bcr_arm.launch.py \
  factory_world_path:=$HOME/ros2_ws/src/factory_bent_project/my_factory_worlds/worlds/dual_circular_factory_conveyor.sdf

# T2: attacher (hold / resume / attach / release services)
ros2 launch factory_bringup bcr_endpoint_object_attacher.launch.py

# T3: side camera bridge + side checker + overhead detector + controller.
#     Opens two windows: "Object Detection" (top view, ROI, track) and "Side Camera"
#     (side view, objects with y / height, CROSS-CHECK verdict).
ros2 launch factory_bringup vision_pick_place.launch.py
#   wait_at:=station        arm hovers over the first bent between catches (default home: it waits at its
#                           start position; when both cameras confirm an object the first bent stops,
#                           the arm comes to grab it, the bent runs again, and after placing it goes back)
#   motion_type:=auto       movement type per cycle (default auto = chosen by object height); also
#                           cycle | random | standard | mirror | high_carry | low_fast | full_turn |
#                           reverse_return  (each one is described in MOTION_TYPES in vision_pick_place.py)
#   motion_variety:=false   always the "standard" movement
#   use_side_camera:=false  previous behaviour (overhead camera only, attach from the hover point)
#   dry_run:=true           detect, cross-check and solve IK, log the moves, do not move the arm
#   loop:=false             catch one object and stop
#   auto_start:=false       wait for the start call below
#   show_window:=false      no windows (views still on /vision/debug_image and /vision/side_debug_image)
#   mode:=pick_zone         old behaviour: pick a still object near the arm

# T4 (optional)
ros2 service call /vision_pick_place/start std_srvs/srv/Trigger
ros2 service call /vision_pick_place/stop  std_srvs/srv/Trigger
ros2 topic echo /vision_pick_place/status       # CROSS-CHECK OK, HOLDING, descend, CAUGHT, PLACED
ros2 topic echo /vision_pick_place/cross_check  # why it is not grabbing yet
ros2 topic echo /vision/side_check              # side camera: present, y, z_top, height
ros2 topic hz /side_camera/image                # side camera bridged? (~15 Hz)
```

Second bent (ring 2) moves too: an object put down on it rides it like the products on the first
bent. A camera above the release zone (`release_camera`, window "Release Zone") checks the zone:
after a grab the arm waits 0.8 m before the zone, holding the object, until the zone is EMPTY,
then ring 2 is stopped, the object is put down, the arm moves back up and ring 2 runs again.
`use_release_camera:=false` on T3 skips the camera check (ring 2 is still stopped for the release).

Separate stop / start for each bent (T2 running):

```bash
ros2 service call /stop_bent1  std_srvs/srv/Trigger   # first bent (same as /stop_bent)
ros2 service call /start_bent1 std_srvs/srv/Trigger
ros2 service call /stop_bent2  std_srvs/srv/Trigger   # second bent: objects that were put on it
ros2 service call /start_bent2 std_srvs/srv/Trigger
ros2 topic echo /vision/release_zone                  # release camera: ready, empty, blobs
ros2 service call /release_zone_monitor/relearn std_srvs/srv/Trigger   # re-learn the empty zone (zone must be empty)
```
From code: `VisionPickPlace.stop_bent()`, `start_bent()`, `stop_bent2()`, `start_bent2()`.

Stop / start the whole first bent (T2 running). The controller does this by itself: the bent
stops when the product is held at the station and runs again as soon as the arm has it.
Also usable on their own, and from code via `VisionPickPlace.stop_bent()` / `start_bent()`:

```bash
ros2 service call /stop_bent  std_srvs/srv/Trigger   # every product on the bent stands still
ros2 service call /start_bent std_srvs/srv/Trigger   # all move on (a product the arm holds stays with it)
```

Testing the new attacher services by hand (T2 running, products moving):

```bash
# stop the product nearest the station on the bent
ros2 param set /bcr_endpoint_object_attacher select_near_point true
ros2 param set /bcr_endpoint_object_attacher select_x -2.58
ros2 param set /bcr_endpoint_object_attacher select_y 0.0
ros2 param set /bcr_endpoint_object_attacher auto_select_radius 3.0
ros2 service call /hold_object_for_pick std_srvs/srv/Trigger     # reply: model, pose, top_z
ros2 service call /resume_held_object std_srvs/srv/Trigger       # it carries on along the bent
```

Debug images saved once at start-up in `~/ros2_ws/src/factory_bent_project/debug_frames/`:
`overhead_raw.png`, `overhead_detections.png`, `side_raw.png`, `side_detections.png`.

Tuning (in `launch/vision_pick_place.launch.py`, then relaunch T3):
- arm speed: `move_duration` (1.0 s), `swing_duration` (2.0 s), `descend_duration` (0.8 s)
- where it grabs: `catch_window` (side camera y within 0.25 m of the station)
- camera agreement: `cross_check_tol` (0.20 m); `side_confirm_timeout` (2.0 s)
- how far it comes down: `descend_gap` (0.02 m above the product top)
- lift right after the grab, with one joint only: `lift_joint` (`joint2` = shoulder; `ik` = all joints straight up), `lift_height` (0.50 m), capped at gripper tip z `lift_max_z` (2.35 m, the arm reaches ~2.4 m over ring 1), then one swing round to ring 2:
  `swing_direction` (`left` = counter-clockwise seen from above, or `right`), `carry_swing_duration` (3.0 s)
- wait before release so the carried object catches up: `release_settle_sec` (0.6 s)
- where it puts the object on ring 2: `place_x`, `place_y` (slot 0, now 2.2171, -0.3736), `place_yaw` (-1.2781 rad);
  `place_exact` true = the object is put down exactly there, resting on the belt (`place_surface_z` 1.375);
  slots 1 and 2 are 0.3 m either side along the ring (`place_slots`, `place_slot_spacing`)
- hover height over the products: `hover_z` (2.0 m; tallest product top is ~1.84 m)
- side camera sensitivity: `mog_var_threshold` (25), `min_area_px` (250), `bottom_tol` (0.08 m)
- overhead detector sensitivity: `gray_margin`, `canny_low`, `canny_high`

## 6c. Product orders from the PyQt5 GUI

The "Product Order" window lets you choose which products go on the first bent and in which
order. The chosen products are put on bent 1 the same distance apart along the belt; the top one
of the list reaches the catch station first, and the vision pick and place takes them one by one.

There are 30 product types (`factory_bringup/product_catalog.py`): the 8 products of the world
plus 22 new ones. The new boxes are cereal box, detergent bottle, soda six-pack, book parcel, toy box,
pizza box, tissue box, gift box, tool case, laptop box, pet food bag, kettle box and electronics box.
The cylinders are paint can, glass jar, poster tube, water jug, coffee can, flower pot and candle jar.
The balls (spheres) are a basketball and a football. To add your own type, add one line to
`PRODUCT_TYPES` there. It needs a size, an RGB colour, `box`, `cylinder` or `sphere`, and a label.
Keep the top at least about 8 x 8 cm and the height at least 8 cm, so both cameras see it.

**Pictures.** The GUI shows each type as a picture (`factory_bringup/product_images/<type>.png`).
Pictures from the simulation come from a photo camera in the world (`product_photo_camera` at (0, 11, 1.6),
away from the bents). `capture_product_images` puts each product in front of it, one after the
other, and saves what the camera sees. The package ships with pre-made previews in the same
place, drawn with the same camera angle and colours, so the GUI has pictures before the first
capture. A capture overwrites them with the real Gazebo images.

```bash
# Gazebo (T1) must be running with the new world (it has the photo camera); takes ~1-2 s per product
ros2 run factory_bringup capture_product_images
ros2 run factory_bringup capture_product_images --ros-args -p types:="['basketball', 'gift_box']"   # only these
```
In the GUI, **Take pictures in Gazebo** runs the same command and reloads the pictures when it is done.
**Reload pictures** reloads them from disk. Run the capture again after you add a new type.

One-time setup (PyQt5, and a rebuild for the new `order_gui` and `capture_product_images` commands):

```bash
sudo apt install python3-pyqt5
cd ~/ros2_ws
colcon build --packages-select factory_bringup --symlink-install
source install/setup.bash
ros2 pkg executables factory_bringup | grep -E "order_gui|capture_product_images"
```
Restart Gazebo (T1) once so the world loads with the photo camera.

Run T1, T2 and T3 as in 6b. Restart T2 (the attacher), because it now serves the orders. Then:

```bash
# T4: Product Order GUI
source ~/ros2_ws/install/setup.bash
ros2 run factory_bringup order_gui
```

In the window:
- **Left grid:** a picture of every product type. Clicking one shows it large below, with its
  shape and size. Double-click or use **Add >>** to add one (the `x N` box adds N at once).
  **Add one of each** adds all 30; at 1.0 m only 19 fit, so lower the distance or remove some.
- **Right list:** the order, with the top item first. **Up** / **Down** change the order. **Remove** and
  **Clear** take items out.
- **Distance between products:** the arc length along the belt (default 1.0 m, minimum 0.5 m;
  the longest product is 0.48 m).
- **First product before the station:** how far before the catch point the first one starts (default 1.5 m).
- The label shows how many products fit on the ring at that distance. For example, 19 fit at 1.0 m
  and 39 at 0.5 m. The Spawn button is off when the order is too long.
- **Remove the products on bent 1 first** (on by default): the world's products on bent 1 are
  deleted, so only your order rides it. Objects already on bent 2 and the one the arm holds stay.
- **Spawn order on bent 1:** bent 1 stops, the products are created standing still at their places,
  and bent 1 runs again. The log at the bottom shows the attacher's answer.
- **Stop / Start bent 1 / 2:** these call the same services as `/stop_bent1` ... `/start_bent2`.

Without the GUI (same thing from a terminal, T2 running):

```bash
ros2 topic pub --once /spawn_order std_msgs/msg/String \
  "{data: '{\"items\": [\"paint_can\", \"cereal_box\", \"milk_carton\"], \"spacing\": 1.0, \"start_offset\": 1.5, \"clear\": true}'}"
ros2 topic echo /spawn_order/result
```

Spawned models are named `order_<type>_<n>`, for example `order_paint_can_1`. Their numbers start
again at 1 when T2 restarts, so restart T1 together with T2 to start from a clean world.
Attacher parameters: `ring1_center_x/y` (-6, 0), `ring1_radius` (3.42), `ring1_product_bottom_z` (1.42),
`spawn_reference_angle` (0 = the catch station on the +x side of ring 1).
Backups made for this change: `bcr_endpoint_object_attacher.py.bak_before_order` and `setup.py.bak_before_order`.
Backups for the pictures change: `product_catalog.py`, `order_gui.py` and `setup.py` `.bak_before_pictures`, and
`dual_circular_factory_conveyor.sdf.bak_before_photocam`.

### Bent 2 orders, one belt speed, simulation time (GUI)

- **Bent 2 tab:** order objects for the second bent the same way as for bent 1. They are put on bent 2
  the same distance apart, and the first one arrives at the release point first. The distance, the
  first offset and "Remove the objects on bent N first" are kept separately for each tab. While
  bent 2 objects pass the release zone, the release camera sees the zone as occupied, so the arm
  waits until it is free (as before).
- **Simulation group:**
  - **Belt speed, both bents** (m/s): one speed for every product on both bents, at any radius.
    It also holds after stop / start, hold / resume, and release on bent 2. Before, bent 2 ran
    about 11 % faster (0.266 against 0.239 m/s). The default 0.2394 m/s is the world file's speed.
  - **Simulation speed** (x real time): Gazebo's real-time factor. 2.0 runs twice as fast, if your
    PC can keep up; the cameras make high values hard to reach.
  - **Run time** (minutes, "no limit" = 0): when it is over, the arm stops after its current move
    and both bents stop. It counts real time, starting from Apply (or from the start of the arm
    with run_factory.sh).
  - **Apply** sends these three to the attacher. In the start-up selection window (run_factory.sh)
    they are saved with the orders and applied at the start.

From the terminal (T2 running):

```bash
ros2 topic pub --once /spawn_order std_msgs/msg/String \
  "{data: '{\"bent\": 2, \"items\": [\"water_jug\", \"football\"], \"spacing\": 1.5, \"start_offset\": 1.0, \"clear\": true}'}"
ros2 topic pub --once /factory_settings std_msgs/msg/String \
  "{data: '{\"belt_speed\": 0.3, \"real_time_factor\": 1.0, \"run_minutes\": 10}'}"
ros2 param set /bcr_endpoint_object_attacher belt_speed 0.3   # only the value; publish on /factory_settings to apply it
```

The start-up file `~/.ros/factory_order.json` is now `{"bent1": {...}, "bent2": {...}, "settings": {...}}`.
Older files with a single order still work as a bent 1 order. Backups: `.bak_before_speed`
(attacher and `order_gui.py`).

## 7. Move the arm from the terminal

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once

ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7],
  points: [{positions: [0.0, 0.45, 0.0, -0.65, 0.0, 0.35, 0.0], time_from_start: {sec: 2}}]}"
```

## 8. Gazebo object commands

Replace `dual_circular_factory_world` with `factory_world` for the single bent.

```bash
# Spawn an object from an SDF file
gz service -s /world/dual_circular_factory_world/create \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 2000 \
  --req 'sdf_filename: "/tmp/box.sdf", name: "box_1", pose: {position: {x: 0, y: 0, z: 1.5}}'

# Move an object
gz service -s /world/dual_circular_factory_world/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 \
  --req 'name: "pickup_box", position: {x: -2.175, y: 1.195, z: 2.0}'

# Delete an object
gz service -s /world/dual_circular_factory_world/remove \
  --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 1000 \
  --req 'name: "box_1", type: MODEL'

# Pause / resume
gz service -s /world/dual_circular_factory_world/control \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 1000 --req 'pause: true'
```

## New look: order GUI and factory world

**Order GUI** (`order_gui.py`): it has a dark factory theme and a header with a status light (green "ATTACHER ONLINE" once the simulation listens). Other additions:

- a search box and a shape filter for the 30 products;
- numbered order lists that you can sort by dragging (Up / Down still work);
- coloured Stop / Start buttons, plus "Stop both" and "Start both";
- times in front of each message.

Everything it sends is unchanged.

**World** (`dual_circular_factory_conveyor.sdf`) now has decor: a factory hall with windows, lamps and columns, epoxy floor and dark pads under the bents, a yellow safety fence with a gate and stack lights, a hazard frame around the robot, walkways, pallet racks, an operator desk, a photo studio behind the photo camera, and "BENT 1 PICK" / "BENT 2 PLACE" signs. All of it is static and visual only (no collisions). None of it is over the belts or inside the arm's reach. The products, cameras, station, release point and lighting of the belts are unchanged.

The old plain world is kept as `dual_circular_factory_conveyor_plain.sdf`. If Gazebo runs slowly with the decor, start with that one:

```bash
ros2 launch factory_bringup factory_full.launch.py \
  factory_world_path:=$HOME/ros2_ws/src/factory_bent_project/my_factory_worlds/worlds/dual_circular_factory_conveyor_plain.sdf
```

With the one-command launcher: `./run_factory.sh factory_world_path:=$HOME/ros2_ws/src/factory_bent_project/my_factory_worlds/worlds/dual_circular_factory_conveyor_plain.sdf`

Backups: `*.bak_before_style`. Previews and the decor generator are in `style_previews/` in the project files.

## Close grasp and all arm movements

**Close grasp.** The arm now comes down in two steps: quickly to 10 cm above the object, then slowly onto it. It grips only once the gripper tip, computed from the measured joints, is within 1.5 cm of the contact point, 1 cm above the product top (`descend_gap` 0.01). Before, it gripped as soon as every joint was within 0.05 rad. On this long arm that can leave the tip about 10 cm above the object, so the object jumped up to the gripper. If the tip doesn't get there within 3 s, the log shows "gripper tip still … mm from the contact point" and it grips anyway. Related parameters: `descend_slow_height`, `descend_final_duration`, `contact_tolerance`, `contact_timeout_sec`, `descend_ik_tolerance`.

**Arm movements.** `motion_type:=auto` (the default) now uses all six movements:

- tall objects get `high_carry`;
- flat objects get `low_fast`;
- all others go round `standard`, `mirror`, `full_turn` and `reverse_return` in turn.

You can also choose the movement in the GUI ("Simulation" > "Arm movement"):

- In the start window, the choice goes into the order file, and `run_factory.sh` passes it on.
- While the simulation runs, "Apply settings" sets it on `vision_pick_place`, and it is used from the next pick.

From the command line:

```bash
./run_factory.sh motion_type:=cycle        # or random, standard, mirror, high_carry, low_fast, full_turn, reverse_return
ros2 param set /vision_pick_place motion_type random
```

Backups: `*.bak_before_grasp`.

## Trash and recycling bins

There are two open bins next to the arm: **RECYCLE** (green) at (0, +2.2) and **TRASH** (dark grey) at (0, -2.2). They have collisions, so a dropped product lands inside.

In the order GUI, pick a product and set **Goes to** to *Bent 2*, *Recycling bin* or *Trash bin*. Two buttons set many at once:

- "Auto-sort into bins": cardboard, paper, plastic, glass and metal go to recycling, the rest to trash.
- "All to bent 2" undoes everything.

Pictures show an R or T badge. The choice is saved at once in `~/.ros/factory_destinations.json`, and `vision_pick_place` reads it for every pick, so a change counts from the next object. It works in the start window and while the simulation runs.

For a bin product, the arm swings right to the recycling bin or left to the trash bin, lets the product fall in, and logs "BINNED: … (recycling N, trash M so far)". Everything else goes to bent 2 as before.

- `use_bins:=false` (vision parameter) turns the bins off.
- If the bin centre is just out of reach, the tip stops up to 0.25 m short (`bin_max_reach_shift`), or goes higher, and the log warns. The product still drops into the bin.
- The bins only fill up. Products stay in them until you restart.

Backups: `*.bak_before_bins`.

## 9. Debugging

```bash
ros2 node list
ros2 topic list
ros2 topic hz /overhead_camera/image
ros2 service list | grep endpoint

gz topic -l                                       # Gazebo topics
gz service -l | grep world                        # Gazebo services (find world names)
gz model --list                                   # models in the running world
gz model -m pickup_box -p                         # pose of a model

# TF: confirm gripper_tip exists and where it is
ros2 run tf2_ros tf2_echo base_link gripper_tip
ros2 run tf2_tools view_frames                    # writes frames.pdf

# Check a world file parses
gz sdf -k ~/ros2_ws/src/factory_bent_project/my_factory_worlds/worlds/factory_conveyor.sdf
```

## 10. Stop everything

```bash
pkill -f "gz sim"; pkill -f ros_gz_bridge; pkill -f parameter_bridge
ros2 daemon stop && ros2 daemon start   # if ros2 node/topic list looks stale
```

## 11. Git

```bash
cd ~/ros2_ws/src/factory_bent_project
git status
git add factory_bringup my_factory_worlds object_detection*.py
git commit -m "Add BCR arm bringup, circular worlds, gripper-top attacher"
git push origin main
```

## Held product stays still (pin)

While the arm comes down onto a product, the attacher now keeps the product at the pose it had when it was held, re-setting it on every timer tick until the gripper attaches it. Before, the gripper's touch pushed it (products have no gravity or damping), so it drifted, flickered and turned, and was released off-place on bent 2.

- On by default. Turn off: `ros2 param set /bcr_endpoint_object_attacher pin_held_object false` (or `pin_held_object:=false` in the launch).
- If you still see contact jitter, raise `descend_gap` from 0.01 back to 0.02 in `launch/vision_pick_place.launch.py`.
- Backup: `bcr_endpoint_object_attacher.py.bak_before_pin`.

## Trash and recycle functions (objects 2 and 3)

`vision_pick_place.py` has two new functions: `put_in_trash(...)` (swings left, drops into the trash bin) and `drop_in_recycle_bin(...)` (swings right, drops into the recycling bin). Every caught object goes through `deliver(...)`, which counts catches and, by default, calls `put_in_trash` for the 2nd caught object and `drop_in_recycle_bin` for the 3rd. All other objects follow the GUI "Goes to" choice (bent 2 unless set to a bin). Failed catches are not counted.

- Change the mapping: `numbered_destinations:="2:recycling,3:trash"` (any `N:trash` / `N:recycling` list). Turn off: `numbered_destinations:=""`.
- Backup: `vision_pick_place.py.bak_before_binfuncs`.

## Bin drops: high over the bin centre, then into it

A product going to a bin now travels like this: lift off bent 1, swing round high (tip z `bin_travel_z`, 2.1 m) to straight over the bin centre, go down to just above the rim, then release. On release the attacher lets the product go exactly over the bin centre (`release_snap_xy`), so it falls into the middle even if the arm stopped a little short. If it is still hanging in the air 1.5 s later, it is put on the bin floor. If a bin point can't be reached, the arm uses whichever of the two points it can reach and the product is still dropped into the bin, so it never stays hanging off the gripper.

- Travel height: `bin_travel_z` (vision_pick_place). Fall check: `release_fall_check_sec`, `release_fall_floor_z` (attacher).
- Backups: `*.bak_before_bindrop` for both files.
- Checked against the real BCR arm model: both bins are in reach at the travel height (2.1 m) and the drop height (1.54 m). Some IK postures put the elbow against a bin wall, so postures with any joint or link middle within `bin_arm_clearance` (0.15 m) of a bin below its rim are now replaced by a clear one (`urdf_ik.frame_positions`). Backups: `*.bak_before_binclear`.
- Fall into the bin (update): Gazebo gravity left the released product hanging over the bin, so the attacher now moves it down to the bin floor itself, in set_pose steps timed like a real free fall (~0.5 s), standing upright. Only then does it become a normal body. The log shows "Dropped <name> into the bin: z ... -> ...". The floor height is `release_fall_floor_z` (0.04). The fall check (`release_fall_check_sec`) is removed. Backup: `bcr_endpoint_object_attacher.py.bak_before_fall`.

## Other worlds: single circular bent and straight conveyor

The BCR arm now picks and drops into the trash and recycling bins in all three worlds. Pick the world with `--world` (the default is `dual`, so the dual world is unchanged):

```bash
cd ~/ros2_ws && colcon build --packages-select factory_bringup my_factory_worlds && source install/setup.bash
~/ros2_ws/src/factory_bent_project/run_factory.sh --world dual       # two circular bents (as before)
~/ros2_ws/src/factory_bent_project/run_factory.sh --world circular   # circular_factory_conveyor.sdf
~/ros2_ws/src/factory_bent_project/run_factory.sh --world straight   # factory_conveyor.sdf
```

Rebuild once after this update so the new world files are installed.

- Running T1/T2/T3 by hand instead: run `export FACTORY_WORLD_NAME=circular_factory_world` (or `factory_world`) in every terminal before `ros2 launch`, and pass the matching world file. Every node reads this name and sets its belt layout from `WORLD_LAYOUTS` in `product_catalog.py`. If it is not set, the dual layout is used.
- Both worlds were rebuilt from the dual world so the arm, the catch station at (-2.58, 0), the cameras and the two bins sit in exactly the same places. `circular_factory_conveyor.sdf` is ring 1 only. `factory_conveyor.sdf` is the old straight conveyor turned to run along x = -2.58 in the +y direction, through the catch station, ending in a collection box. The worlds are generated by `my_factory_worlds/tools/make_worlds.py` in this shared copy (not on your machine): run it next to `decor2.py`, a copy of the dual world named `dual.sdf` and the original straight world named `factory_conveyor_orig.sdf`.
- Without bent 2 there is nowhere to place a product, so only products going to a bin are picked. The vision first checks which product it would grab; a product that should "stay on the belt" is left alone. In the order window that choice is called "Stays on the belt (not picked)", the bent-2 tab is hidden, and the conveyor buttons are Stop belt / Start belt.
- Objects 2 and 3 (trash, recycling) are now counted per distinct product in arrival order, also in the dual world. Before, only successful catches were counted.
- Straight conveyor: products that pass the end of the belt (y 5.4) are moved into the collection box on a 3 × 4 grid and drop out of the simulation.
- Not yet tested in Gazebo. Backups on your machine: `*.bak_before_worlds` next to each changed file (5 Python files, `run_factory.sh`, both world files).

## One launcher per world

Three scripts next to `run_factory.sh` each start one world with the BCR arm. First the PyQt Product Order window opens for that world's bents, then Gazebo, the conveyor, the arm, the cameras and pick-and-place start:

```bash
cd ~/ros2_ws/src/factory_bent_project
./run_dual_circular.sh       # two circular bents: the window has Bent 1 - pick and Bent 2 - place tabs
./run_single_circular.sh     # one circular bent: one tab, products go to a bin or stay on the belt
./run_straight_conveyor.sh   # straight belt: one tab, the rest ride on into the collection box
```

- They take the same options as `run_factory.sh` (`--no-select`, `--world-products`, `--build`, launch arguments).
- Each world keeps its own last selection (`~/.ros/factory_order_dual.json`, `..._circular.json`, `..._straight.json`), so `--no-select` never reuses another world's products.

## Straight conveyor: products fall into the collection box and stay there

When a product's centre passes the end of the belt (y 5.4), the attacher stops its belt motion, and the product falls off the end in free fall (g = 9.81, about 0.5 s). It clears the box's front wall and lands on the box floor. Slots fill from the front wall backwards, the middle column first, 3 across and 6 rows deep, then a layer higher. Products already in the box are checked every 0.1 s: if anything moves one, it is stopped and put back. The terminal shows "... fell off the end of the belt into the collection box".

- `factory_full.launch.py` now sets `FACTORY_WORLD_NAME` from the `<world name>` in `factory_world_path`. So the straight-belt layout and the box drop are switched on even when you launch without `run_factory.sh` / the `run_*.sh` launchers. Before, without that variable the nodes used the dual layout, and products rode on into the back wall.
- Rebuild once: `cd ~/ros2_ws && colcon build --packages-select factory_bringup --symlink-install && source install/setup.bash`.
- Backups: `bcr_endpoint_object_attacher.py.bak_before_boxdrop` and `launch/factory_full.launch.py.bak_before_worldname`.
- Not tested in Gazebo yet.

## Experimental worlds: two conveyors linked by the BCR arm

Three new worlds combine a straight and a circular conveyor. In each one the arm picks from bent 1 at the catch station (-2.58, 0) and places on bent 2 at (2.22, -0.37), or drops into the trash / recycling bin. Circular to circular is the dual world.

```bash
cd ~/ros2_ws/src/factory_bent_project
./run_exp_straight_to_circular.sh    # straight bent 1 -> circular bent 2   (exp_straight_circular.sdf)
./run_exp_circular_to_straight.sh    # circular bent 1 -> straight bent 2   (exp_circular_straight.sdf)
./run_exp_straight_to_straight.sh    # straight bent 1 -> straight bent 2   (exp_straight_straight.sdf)
./run_exp_circular_to_circular.sh    # circular bent 1 -> circular bent 2   (the dual world)
# or: ./run_factory.sh --world straight-circular | circular-straight | straight-straight | circular-circular
```

- Each launcher first opens the PyQt Product Order window with a Bent 1 and a Bent 2 tab. Each one also keeps its own last selection (`~/.ros/factory_order_<bent1>_<bent2>.json`).
- A straight bent 1 runs +y along x = -2.58 (as in `factory_conveyor.sdf`) and ends over a collection box at the north end. A straight bent 2 runs -y along x = 2.22 and ends over a second box at the south end. Products reaching either end fall into that box and stay there.
- Orders on a straight bent fit 14 products at 1 m spacing (the belt length before the station or place point). On a circular bent it is 19.
- Layouts are in `WORLD_LAYOUTS` (`product_catalog.py`). The worlds are generated by `my_factory_worlds/tools/make_exp_worlds.py` in this shared copy. Top-view previews are in `exp_world_previews/`.
- Backups: `*.bak_before_exp` next to the 4 changed Python files and `run_factory.sh`. The dual, single circular and straight worlds are unchanged.
- Not tested in Gazebo yet. Rebuild once: `cd ~/ros2_ws && colcon build --packages-select factory_bringup --symlink-install`.

## Pipeline factory: 13 conveyors and 12 arms (layout only, no motion yet)

A larger experimental world joins the straight and circular conveyors into one pipeline. The straight conveyors carry products between stations, and the circular conveyors are where products get sorted. It has 4 sorting rings (C1 to C4), 9 transport lines (S1 to S9), 12 BCR arms (A1 to A12) on pedestals, and 5 loading docks. Every conveyor, arm and dock has a label painted on the floor.

```bash
cd ~/ros2_ws/src/factory_bent_project
./run_pipeline_factory.sh            # opens my_factory_worlds/worlds/pipeline_factory.sdf in Gazebo
```

Flow (A = arm, rule = what that arm takes off the ring; everything else rides on):

| Arm | From | To | Rule |
|---|---|---|---|
| A1 | S1 inbound | C1 sort recyclable | all |
| A2 | C1 | S2 to recycling | recyclable |
| A3 | C1 | S3 to general | rest |
| A4 | S2 | C2 sort cardboard | all |
| A6 | C2 | S4, dock 1 cardboard | cardboard |
| A7 | C2 | S5, dock 2 containers | rest |
| A5 | S3 | C3 sort by size | all |
| A9 | C3 | S7, dock 3 large | large |
| A8 | C3 | S6 to shipping | rest |
| A10 | S6 | C4 sort shipping | all |
| A11 | C4 | S8, dock 4 express | small |
| A12 | C4 | S9, dock 5 standard | rest |

- **Layout only.** Belts don't move, there are no products, and the arms are static models posed over their pick points. The station coordinates, routing rules and belt speed are in `factory_bringup/factory_bringup/pipeline_layout.py`, ready for the automation step.
- **Coordinates.** All belt tops are at 1.375 m, and each arm's pick and place points are 2.45 m from its base. Arm reach was checked with IK at all 12 stations.
- **Arm meshes.** The arms use the BCR meshes through `model://bcr_arm_description/...`. The launcher adds `$(ros2 pkg prefix bcr_arm_description)/share` to `GZ_SIM_RESOURCE_PATH`, so `bcr_arm_description` must be built and sourced.
- **Regenerating.** Run `my_factory_worlds/tools/make_pipeline_world.py` next to a copy of the dual world named `dual.sdf`. Previews are in `pipeline_previews/`.
- **Changes.** All files are new and no existing file was changed. No colcon rebuild is needed to open the world.
- **Not tested in Gazebo yet.**

## Straight conveyor: products hanging in the air past the belt end

Products have no gravity, because VelocityControl moves them along the belt. Only the attacher node moves them into the collection box. If that node stops, products leave the belt end and hang in the air.

- **Likely cause (a guess, not confirmed from your logs).** The belt-end check starts as soon as the attacher does, often before Gazebo publishes poses. Its fallback `gz topic -e` then waited 3 s, raised a timeout, and that error stopped the whole attacher node.
- **Fix.** The check now only reads the gz-transport pose stream and logs errors instead of raising them. It also retries `set_pose` into the box until it succeeds. After 30 s with no poses, it warns that Gazebo may be paused or running another world.
- **Rebuild and run.** `./run_straight_conveyor.sh --build`, or `colcon build --packages-select factory_bringup --symlink-install` in `~/ros2_ws`.
- **What to check in the terminal.** Look for "Straight bent 1: products past y = 5.40 fall into its collection box" at start, then "... fell off the end of bent 1 into its collection box" for each product.
- **Backup.** `bcr_endpoint_object_attacher.py.bak_before_hangfix`.
- **Not tested in Gazebo yet.**

## Vision lab: OpenCV / YOLO development with the arm and the bins

A development environment for detection algorithms: one `main(lab)` function in
`vision_lab_main.py` drives the arm from bent 1 to bent 2 and into the trash / recycling bins in every
world, with a detector you choose or write (OpenCV or YOLO). Full guide:
`factory_bringup/factory_bringup/vision_lab/README.md`.

```bash
cd ~/ros2_ws && colcon build --packages-select factory_bringup --symlink-install && source install/setup.bash
chmod +x ~/ros2_ws/src/factory_bent_project/run_vision_lab.sh
cd ~/ros2_ws/src/factory_bent_project

./run_vision_lab.sh sim --world dual                     # T1: simulation without the standard detector/controller
./run_vision_lab.sh run                                  # T2: main(lab) in vision_lab_main.py (edit, run again)
./run_vision_lab.sh run --script sort_into_bins          # installed examples: bent_to_bent, sort_into_bins,
                                                         #   custom_detector, yolo_sort
./run_vision_lab.sh run --detector opencv_color --opt "targets=['paint_can','glass_jar']"
./run_vision_lab.sh run --detector yolo --opt model=$HOME/best.pt --record ~/vision_lab_dataset
./run_vision_lab.sh --world straight-circular            # both in one terminal
./run_vision_lab.sh offline --synthetic 20 --world all   # score detectors without Gazebo
./run_vision_lab.sh worlds ; ./run_vision_lab.sh detectors
```

Worlds: dual, circular, straight, straight-circular, circular-straight, straight-straight (circular and
straight have no bent 2: bins only). YOLO is optional: `pip install ultralytics`.
New launch arguments of `factory_full.launch.py` / `vision_pick_place.launch.py`: `run_detector` and
`run_controller` (both default true, so the older commands behave as before).
Backups for this change are named `.bak_before_lab` (both launch files, `setup.py`, this file).

### Vision lab files: one experiment per file

Experiments are separate files in `labs/` (`lab_01_detection_survey` ... `lab_07_record_dataset`), each
with its own detector and arm calls (`lab.move_to_bent(2)`, `lab.move_to_trash()`,
`lab.move_to_recycle_bin()`, `lab.run(decide)`). Guide: `labs/README.md`.

```bash
./run_vision_lab.sh list                               # the lab files
./run_vision_lab.sh new my_lab --template sort         # create labs/lab_my_lab.py (see: templates)
./run_vision_lab.sh run my_lab                         # run it on the running simulation (sim first)
./run_vision_lab.sh offline --lab my_lab --synthetic 10 --world all
```

## Vision labs in the pipeline world

```bash
sudo apt install python3-gz-transport13                    # once: the lab drives Gazebo over gz-transport
./run_vision_lab.sh sim --world pipeline                    # Gazebo with pipeline_factory_lab.sdf (cameras, bins, movable arms)
./run_vision_lab.sh run 02_bent_to_bent --station A2        # any lab at one arm station (A1 ... A12)
./run_vision_lab.sh run 08_pipeline_sorting                 # every station sorts by vision
./run_vision_lab.sh run 08_pipeline_sorting --fake          # without Gazebo
./run_vision_lab.sh run 09_pipeline_demo                    # demo: detection, one arm, the whole factory
./run_vision_lab.sh run 10_pipeline_tests                   # 8 pipeline test cases, PASS / FAIL per product
PIPELINE_TESTS=branch_on_C1 ./run_vision_lab.sh run 10_pipeline_tests --fake   # one case, no Gazebo
./run_vision_lab.sh offline --world pipeline --synthetic 5  # detector metrics on the 12 station cameras
cd my_factory_worlds/tools && python3 make_pipeline_lab_world.py   # rebuild the lab world after editing pipeline_layout.py
```
