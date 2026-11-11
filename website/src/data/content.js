// All text and image references for the site live here, so the project
// description can be updated without touching the layout code.

export const img = (name) => `${import.meta.env.BASE_URL}images/${name}`
export const report = (name) => `${import.meta.env.BASE_URL}reports/${name}`

export const REPO_URL = 'https://github.com/huseyinTozluyurt/Factory-Bent-Simulation-with-BCR-Robot-Arm'

export const nav = [
  { id: 'overview', label: 'Overview' },
  { id: 'architecture', label: 'Architecture' },
  { id: 'worlds', label: 'Worlds' },
  { id: 'vision', label: 'Vision' },
  { id: 'arm', label: 'Pick & place' },
  { id: 'products', label: 'Products' },
  { id: 'pipeline', label: 'Pipeline factory' },
  { id: 'lab', label: 'Vision lab' },
  { id: 'timeline', label: 'Timeline' },
  { id: 'run', label: 'Run it' },
]

export const stats = [
  { value: '7-axis', label: 'BCR robot arm' },
  { value: '3', label: 'OpenCV cameras' },
  { value: '10', label: 'Gazebo worlds' },
  { value: '100+', label: 'product types' },
  { value: '12', label: 'arms in the pipeline factory' },
  { value: '10', label: 'vision lab experiments' },
]

export const tech = ['ROS 2 Jazzy', 'Gazebo Harmonic', 'Python 3.12', 'OpenCV 4', 'PyQt5', 'YOLO (optional)', 'Ubuntu 24.04']

export const features = [
  {
    icon: '👁️',
    title: 'Three-camera OpenCV vision',
    text: 'An overhead camera detects and tracks products, a side camera cross-checks them and measures their height, and a release camera checks that the drop zone is free.',
  },
  {
    icon: '🦾',
    title: 'Closed-loop pick and place',
    text: 'URDF-based inverse kinematics, catching moving products, pinning the product while the gripper descends, a precise grasp, and six movement styles.',
  },
  {
    icon: '♻️',
    title: 'Sorting into bins',
    text: 'A trash bin and a recycling bin, a destination for every product type, and auto-sort by material.',
  },
  {
    icon: '📦',
    title: 'Product generator',
    text: '100 realistic product types, your own .stl, .obj or .dae models, and orders spawned on any bent.',
  },
  {
    icon: '🖥️',
    title: 'Order GUI',
    text: 'A PyQt5 window for orders, destinations, belt and simulation speed, run time, and the arm movement style.',
  },
  {
    icon: '🌍',
    title: '10 worlds',
    text: 'Circular and straight bents in every combination, plus a factory with 13 conveyors and 12 arms.',
  },
  {
    icon: '🧪',
    title: 'Vision lab',
    text: 'Plug-in OpenCV or YOLO detectors, 10 example labs, scoring without Gazebo, auto-labelled YOLO datasets and pipeline tests.',
  },
  {
    icon: '🚀',
    title: 'One-command start',
    text: './run_factory.sh starts everything: the order window, Gazebo, the arm, the cameras and the controller.',
  },
]

export const nodes = [
  {
    name: 'ring_object_detector',
    role: 'Overhead OpenCV: grayscale, blur, threshold or Canny, contours in each belt lane, minAreaRect, pixel to world. The arm is masked out of the image using TF.',
  },
  {
    name: 'side_camera_checker',
    role: 'MOG2 background subtraction and Canny. Finds products standing on the belt at the catch station and measures their height.',
  },
  {
    name: 'release_zone_monitor',
    role: 'Compares each frame with a learned empty belt and reports whether the drop zone on bent 2 is empty.',
  },
  {
    name: 'vision_pick_place',
    role: 'Runs the pick cycle: camera cross-check, URDF IK (damped least squares), movement styles and destinations.',
  },
  {
    name: 'bcr_endpoint_object_attacher',
    role: 'Grasp and release, holding the product still, stopping and starting bents, spawning orders, and simulated falls into bins and boxes.',
  },
  {
    name: 'order_gui',
    role: 'Orders, destinations, belt speed, simulation speed and run time, arm movement style, product pictures and custom products.',
  },
]

export const cycle = [
  { id: 'Home', text: 'The arm waits at its home pose while the overhead camera tracks products on bent 1 and predicts when one will reach the catch station at (-2.58, 0).' },
  { id: 'Cross-check', text: 'A product reaches the station. The side camera must see a blob standing on the belt within 0.20 m of the overhead track. If the cameras disagree, the product simply rides on.' },
  { id: 'Hold', text: 'Both cameras agree. Bent 1 stops and the product is pinned in place, so the descending gripper cannot push it.' },
  { id: 'Descend', text: 'The arm drops quickly to 10 cm above the product top measured by the side camera, then slowly. Forward kinematics is checked every step.' },
  { id: 'Attach', text: 'The grip happens only when FK puts the tip within 1.5 cm of the product top. The attacher snaps the top face to the gripper tip and bent 1 starts again.' },
  { id: 'Lift', text: 'The product is lifted 0.30 to 0.60 m, depending on the movement style chosen from its height.' },
  { id: 'Swing', text: 'One smooth swing toward the destination: bent 2, the trash bin or the recycling bin.' },
  { id: 'Wait zone', text: 'For bent 2, the release camera must report the drop zone EMPTY. Bent 2 is stopped during the release.' },
  { id: 'Place / drop', text: 'The product is placed at (2.22, -0.37) on bent 2, or dropped from a high travel pose over the bin centre with a simulated free fall. Then the arm returns home.' },
]

export const worlds = [
  {
    key: 'dual',
    name: 'Dual circular',
    cmd: './run_dual_circular.sh',
    bent1: 'circular',
    bent2: 'circular',
    delivers: 'bent 2, trash, recycling',
    image: 'screenshot_2026-10-31_dual_circular_factory.png',
    text: 'The main cell: two circular bents with the BCR arm between them, a trash bin, a recycling bin, and factory decor (hall, fences, stack lights, racks, a control desk).',
  },
  {
    key: 'straight-circular',
    name: 'Straight → circular',
    cmd: './run_exp_straight_to_circular.sh',
    bent1: 'straight',
    bent2: 'circular',
    delivers: 'bent 2, trash, recycling',
    image: 'exp_straight_to_circular.png',
    text: 'Products arrive on a straight conveyor and are placed onto a circular bent.',
  },
  {
    key: 'circular-straight',
    name: 'Circular → straight',
    cmd: './run_exp_circular_to_straight.sh',
    bent1: 'circular',
    bent2: 'straight',
    delivers: 'bent 2, trash, recycling',
    image: 'exp_circular_to_straight.png',
    text: 'The arm picks from a circular bent and places onto a straight outbound conveyor.',
  },
  {
    key: 'straight-straight',
    name: 'Straight → straight',
    cmd: './run_exp_straight_to_straight.sh',
    bent1: 'straight',
    bent2: 'straight',
    delivers: 'bent 2, trash, recycling',
    image: 'exp_straight_to_straight.png',
    text: 'Two straight conveyors: a classic line-to-line transfer cell.',
  },
  {
    key: 'straight',
    name: 'Single straight',
    cmd: './run_straight_conveyor.sh',
    bent1: 'straight',
    bent2: 'none',
    delivers: 'trash and recycling (the rest fall into a collection box)',
    image: 'screenshot_2026-10-31_order_gui_straight_detection.png',
    text: 'The singular factory bent with the object generator. Products that the arm leaves on the belt fall into a collection box at its end. This world was tested on the real PC on Nov 1.',
  },
  {
    key: 'single-circular',
    name: 'Single circular',
    cmd: './run_single_circular.sh',
    bent1: 'circular',
    bent2: 'none',
    delivers: 'trash and recycling (the rest stay on the belt)',
    image: null,
    text: 'One circular bent. The arm sorts products into the bins and the rest keep circling.',
  },
]

export const cameras = [
  {
    name: 'Overhead',
    where: '13 m above the cell, looking straight down',
    steps: ['grayscale', 'blur', 'threshold or filled Canny', 'morphology', 'contours per lane', 'minAreaRect', 'pixel ray → product top plane'],
    question: 'Where is each product, and when will it reach the arm?',
    image: 'overhead_detections.png',
  },
  {
    name: 'Side',
    where: 'at belt height, facing the catch station',
    steps: ['MOG2 background subtraction', 'Canny', 'contours whose bottom is on the belt', 'height in metres'],
    question: 'Is it really standing on the belt, and how tall is it?',
    image: 'side_camera_detections.png',
  },
  {
    name: 'Release',
    where: 'above the drop zone on bent 2',
    steps: ['difference from learned empty belt', 'threshold', 'contours in the zone'],
    question: 'Is the drop zone free?',
    image: 'release_zone_camera.png',
  },
]

export const motionStyles = [
  { style: 'standard / mirror', lift: '0.50 m', swing: '3.0 s, left / right', auto: 'medium-height objects, taking turns' },
  { style: 'high_carry', lift: '0.60 m', swing: '4.0 s', auto: 'tall objects (≥ 0.30 m), to clear ring 1' },
  { style: 'low_fast', lift: '0.30 m', swing: '2.5 s', auto: 'flat objects (≤ 0.15 m), shortest cycle' },
  { style: 'full_turn / reverse_return', lift: '0.45 m', swing: '3.0 / 3.5 s', auto: 'medium-height objects, taking turns' },
]

export const labs = [
  ['01', 'Detection survey', 'Detection only: counts per bent and label, snapshots'],
  ['02', 'Bent → bent', 'Bent 1 to bent 2 (recycling in worlds without a bent 2)'],
  ['03', 'Trash and recycle', 'Products into both bins'],
  ['04', 'Sort by label', 'A decide(ctx) rule on the detector’s label'],
  ['05', 'My OpenCV detector', 'Your own detector class inside the lab file'],
  ['06', 'YOLO sorting', 'YOLO labels choose the destination'],
  ['07', 'Record dataset', 'An auto-labelled YOLO dataset while the arm works'],
  ['08', 'Pipeline sorting', 'All 12 pipeline stations sort by vision'],
  ['09', 'Pipeline demo', 'Three acts: detection at A1, arm A1, the whole factory'],
  ['10', 'Pipeline tests', '8 test cases with PASS/FAIL for each product'],
]

export const labApi = [
  ['lab.detect_objects(bent=1)', 'Detections in world coordinates: label, score, x, y, yaw, size, bent'],
  ['lab.move_to_bent(2) · move_to_trash() · move_to_recycle_bin()', 'The next product at the station goes there'],
  ['lab.run(decide, count=N)', 'Keep catching; decide(ctx) returns bent2, trash, recycling or skip'],
  ['lab.spawn_products([...]) · stop_bent(n) · set_motion(...)', 'Belts, orders and arm style'],
  ['lab.snapshot() · start_recording(dir)', 'Annotated images, auto-labelled YOLO dataset'],
  ['lab.report()', 'Where each product went, and whether the label matched the simulation’s truth'],
]

export const pipelineTests = [
  ['one_hop_A1', 'S1 → C1', 'PASS (3/3)', '96 s'],
  ['bins_at_A1', 'A1 into its trash bin, recycling bin and C1', 'PASS (3/3)', '84 s'],
  ['two_hops_to_S3', 'S1 → C1 → S3', 'PASS (2/2)', '119 s'],
  ['chain_to_large_dock', 'S1 → C1 → S3 → C3 → S7 → dock 3', 'PASS (2/2)', '244 s'],
  ['branch_on_C1', 'A2 → dock 2, A3 → dock 3', 'PASS', '328 s'],
  ['bins_along_the_chain', 'A3 recycles pizza boxes, A5 trashes coffee cans, rest → dock 3', 'PASS', '299 s'],
  ['longest_route_to_S9', 'S1 → C1 → S3 → C3 → S6 → C4 → S9 → dock 5', 'PASS', '307 s'],
]

export const eras = [
  {
    n: 1,
    title: 'Simulation scaffold',
    when: 'Mar – Oct 2026',
    text: 'Worlds, cameras and the arm were built. Detection was a standalone script with no link to ROS coordinates, and the arm was moved by hand.',
    images: [
      ['2026-03_straight_conveyor_first_version.png', 'March 2026: the straight conveyor and camera view'],
      ['ring_detection_tuned.png', 'Oct 29: tuned ring detection'],
    ],
  },
  {
    n: 2,
    title: 'Closing the loop',
    when: 'Oct 29',
    text: 'In one evening the project went from detection only to a camera-driven arm: first picking still objects, then catching moving products, then cross-checking with a second camera and stopping the bent while grasping.',
    images: [
      ['side_camera_detections.png', 'Side camera cross-check'],
      ['overhead_detections.png', 'Overhead detections in world coordinates'],
    ],
  },
  {
    n: 3,
    title: 'Usability',
    when: 'Oct 30',
    text: 'With the loop working, focus moved to the operator: the order GUI, product pictures, one-command start, speed control, the dark theme and factory decor, and bins as a second kind of destination.',
    images: [
      ['order_gui_first_version.png', 'Order GUI, first version'],
      ['order_gui_dark_theme_bins.png', 'Order GUI, dark theme with bins'],
    ],
  },
  {
    n: 4,
    title: 'Generalisation',
    when: 'Oct 31 night',
    text: 'The cell was rebuilt so that any bent shape works: six cell worlds from one layout table, and then a whole factory of 12 cells.',
    images: [
      ['exp_straight_to_circular.png', 'Straight → circular'],
      ['pipeline_top.png', 'The pipeline factory layout'],
    ],
  },
  {
    n: 5,
    title: 'Research platform',
    when: 'Oct 31 – Nov 1',
    text: 'The fixed detector became a slot. The vision lab lets anyone plug in an OpenCV or YOLO detector, run it in any world, score it offline, and test multi-arm routes. Products became realistic, and users can add their own.',
    images: [
      ['screenshot_2026-10-31_pipeline_factory_hall.png', 'The pipeline factory hall'],
      ['products_100_realistic.png', '100 realistic products'],
    ],
  },
]

export const phases = [
  ['Mar 16–17', 'Beginning', 'Repository created. A straight conveyor world, an overhead camera bridged to ROS, a camera viewer, and the first threshold + contour detection scripts with tracking and counting.'],
  ['Apr – Oct', 'Arm cell', 'Circular and dual circular worlds, BCR arm integration, a PyQt5 arm UI with preset poses, and an endpoint attacher that fakes the grasp with set_pose.'],
  ['Oct 26', 'Exact attach', 'The attacher was rewritten: the product top snaps to the gripper tip, follows the arm through gz-transport bindings with no CLI lag, and is released from its exact pose.'],
  ['Oct 29 20:43', 'Command sheet', 'COMMANDS.md: one sheet for building, launching every world, and the attacher, UI, detection and debugging commands.'],
  ['Oct 29 21:05', 'Vision pick and place', 'ring_object_detector (grayscale, Canny, contours, pixel to world) and vision_pick_place (URDF IK, pick still objects, place on ring 2).'],
  ['Oct 29 21:34', 'Catch station', 'Catching moving products. A ROI tracker predicts when a product enters the station circle, accounting for catch latency.'],
  ['Oct 29 22:32', 'Side camera', 'A second camera at belt height must confirm the product; it is then held and the arm descends onto its measured top.'],
  ['Oct 29 22:51', 'Carry', 'Lift after the grab, capped at the arm’s reach, then one swing to ring 2 with a short settle before release.'],
  ['Oct 29 23:24', 'Bent stop', 'Bent 1 stops while the product is held and restarts once the arm has it. Exact place slots on ring 2.'],
  ['Oct 30 00:11', 'Moving ring 2', 'Bent 2 moves too. The release-zone camera waits for an empty zone; bent 2 stops during release.'],
  ['Oct 30 01:03', 'Home & styles', 'The arm waits at home. Six movement styles chosen by the object’s height.'],
  ['Oct 30 01:42', 'Order GUI', 'Product Order window (PyQt5) and the /spawn_order topic with 16 product types.'],
  ['Oct 30 14:36', 'Pictures', '30 product types including spheres, and a photo camera that renders a picture grid for the GUI.'],
  ['Oct 30 21:36', 'One command', 'run_factory.sh: the order window first, then one launch file starts everything. A .desktop launcher.'],
  ['Oct 30 21:50', 'Speed', 'Bent 2 orders, one belt speed for both bents, simulation speed and a run-time limit.'],
  ['Oct 30 22:27', 'Style & decor', 'A dark factory theme for the GUI and world decor: hall, fence, stack lights, racks, desk, signs.'],
  ['Oct 30 23:45', 'Close grasp', 'Two-step descent; grip only when FK says the tip is within 1.5 cm of the contact point.'],
  ['Oct 30 23:57', 'Bins', 'Trash and recycling bins with collisions, a “Goes to” choice per product, and auto-sort by material.'],
  ['Oct 31 00:08', 'Bin drop', 'Pinned product during descent, high travel over the bin centre, bin-wall-safe IK and a simulated free fall.'],
  ['Oct 31 01:46', 'Worlds', 'Single circular and straight worlds rebuilt around the same cell; WORLD_LAYOUTS tells every node the belt geometry.'],
  ['Oct 31 03:34', 'Mixed worlds', 'Free fall into the collection box. Three experimental worlds that mix straight and circular bents.'],
  ['Oct 31 04:21', 'Pipeline factory', '4 sorting rings, 9 lines, 12 arms, 5 docks, generated from pipeline_layout.py.'],
  ['Oct 31 17:32', 'Hang fix', 'Products no longer hang in the air past the belt end; the attacher reads the pose stream and logs errors.'],
  ['Oct 31 18:00', 'Vision lab', 'OpenCV/YOLO detectors plugged into the controller in every world, main(lab) scripts and offline scoring.'],
  ['Oct 31 19:00', 'Pipeline lab', 'A camera and bins at each arm, whole-factory sorting, and a demo in three acts.'],
  ['Nov 1 00:00', 'Realistic & custom', 'Straight conveyor tested on the real PC. 100 realistic products and custom STL/OBJ/DAE products.'],
  ['Nov 1 05:19', 'Pipeline tests', '8 pipeline test cases with PASS/FAIL for each product along chains of up to 6 arms.'],
]

export const reports = [
  ['01_Project_Overview.pdf', '01 · Project Overview', 'Goals, architecture, worlds, how to run'],
  ['02_Evolution_Timeline.pdf', '02 · Evolution Timeline', 'Every phase with its date and reason'],
  ['03_Vision_Pipeline_Evolution.pdf', '03 · Vision Pipeline', 'From threshold scripts to cross-checked cameras'],
  ['04_Robot_Arm_Pick_and_Place_Evolution.pdf', '04 · Robot Arm Pick & Place', 'Attacher, IK, grasp, movement styles, bins'],
  ['05_Worlds_and_Product_Generator_Evolution.pdf', '05 · Worlds & Product Generator', 'From 1 belt to 13, from 7 products to 100+'],
  ['06_Vision_Lab_and_Testing.pdf', '06 · Vision Lab & Testing', 'Lab API, demo, pipeline test results'],
]

export const roadmap = [
  [true, 'Vision-driven pick and place between two bents'],
  [true, 'Side-camera and release-camera checks'],
  [true, 'Trash and recycling bins, and sorting by material'],
  [true, 'Six single-cell worlds and the 12-arm pipeline factory'],
  [true, 'Vision lab with OpenCV and YOLO detectors'],
  [true, '100 realistic products and custom 3D products'],
  [false, 'Test every world and the pipeline lab in Gazebo on hardware'],
  [false, 'Train a YOLO model on a lab 07 dataset and compare it with the OpenCV detectors'],
  [false, 'Run the whole pipeline factory with ROS controllers'],
]
