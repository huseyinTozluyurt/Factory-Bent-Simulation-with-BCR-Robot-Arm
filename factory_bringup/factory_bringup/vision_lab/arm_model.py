"""
BCR arm kinematics for the pipeline lab (no ROS imports): the arm's URDF (a copy of
my_factory_worlds/tools/arm.urdf, the one the pipeline world was built from), the transform of every
link for given joint values (to pose the arm's link models in Gazebo), and IK for the gripper tip.
"""
import math
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from factory_bringup.urdf_ik import UrdfChain

JOINTS = [f"joint{i}" for i in range(1, 8)]
SEED = [1.30, -0.75, 0.0, -0.80, 0.0, -1.20, 0.0]
MESH_SCALE_DEFAULT = "1 1 1"


def rpy(r, p, y) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def tf(xyz="0 0 0", r="0 0 0") -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy(*[float(v) for v in r.split()])
    T[:3, 3] = [float(v) for v in xyz.split()]
    return T


def axis_rot(axis, a) -> np.ndarray:
    k = np.array(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K
    return T


def euler(R) -> Tuple[float, float, float]:
    return (math.atan2(R[2, 1], R[2, 2]), math.asin(max(-1.0, min(1.0, -R[2, 0]))), math.atan2(R[1, 0], R[0, 0]))


def quat(R) -> Tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation matrix."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)


class ArmModel:
    """Kinematics of one BCR arm type (shared by all 12 pipeline arms)."""

    def __init__(self, urdf: str = None):
        self.urdf = urdf or ARM_URDF
        self.root = ET.fromstring(self.urdf)
        self.chain = UrdfChain(self.urdf, "base_link", "gripper_tip")
        self.joints = self.root.findall("joint")
        self.colors = {}
        for g in self.root.findall("gazebo"):
            d = g.find("material/diffuse")
            if g.get("reference") and d is not None:
                self.colors[g.get("reference")] = tuple(float(v) for v in d.text.split()[:3])
        self.links = [l.get("name") for l in self.root.findall("link")
                      if l.get("name") in self.link_frames({}) and l.findall("visual")]

    def link_frames(self, q: Dict[str, float]) -> Dict[str, np.ndarray]:
        """base_link-frame transform of every link for joint values q."""
        frames = {"base_link": np.eye(4)}
        changed = True
        while changed:
            changed = False
            for j in self.joints:
                parent, child = j.find("parent").get("link"), j.find("child").get("link")
                if parent in frames and child not in frames:
                    o = j.find("origin")
                    T = frames[parent] @ (tf(o.get("xyz", "0 0 0"), o.get("rpy", "0 0 0")) if o is not None else np.eye(4))
                    if j.get("type") in ("revolute", "continuous"):
                        T = T @ axis_rot([float(v) for v in j.find("axis").get("xyz").split()], q.get(j.get("name"), 0.0))
                    frames[child] = T
                    changed = True
        return frames

    def visuals(self, link: str) -> List[Dict]:
        """Mesh visuals of a link in its own frame (uri as model://...)."""
        out = []
        for l in self.root.findall("link"):
            if l.get("name") != link:
                continue
            for k, v in enumerate(l.findall("visual")):
                mesh = v.find("geometry/mesh")
                if mesh is None:
                    continue
                o = v.find("origin")
                out.append({"name": f"{link}_{k}", "T": tf(o.get("xyz", "0 0 0"), o.get("rpy", "0 0 0")) if o is not None else np.eye(4),
                            "uri": mesh.get("filename").replace("package://", "model://"),
                            "scale": mesh.get("scale", MESH_SCALE_DEFAULT),
                            "color": self.colors.get(link, (0.6, 0.6, 0.6))})
        return out

    def tip(self, q: Dict[str, float]) -> np.ndarray:
        return self.chain.tip_position(q)

    def solve(self, target_base: Sequence[float], seeds: List[Dict[str, float]]) -> Tuple[Dict[str, float], float]:
        return self.chain.solve_position(np.asarray(target_base, float), seeds, active=JOINTS)

    def seeds_towards(self, target_base: Sequence[float], extra: Optional[List[Dict[str, float]]] = None) -> List[Dict[str, float]]:
        bearing = math.atan2(target_base[1], target_base[0])
        seeds = list(extra or [])
        for j1 in (bearing, bearing + math.pi / 2, bearing - math.pi / 2):
            for j2 in (-0.75, 0.0, 0.75):
                s = list(SEED)
                s[0], s[1] = j1, j2
                seeds.append(dict(zip(JOINTS, s)))
        return seeds


ARM_URDF = r'''<?xml version="1.0" ?>
<!-- =================================================================================== -->
<!-- |    This document was autogenerated by xacro from bcr_arm_description/urdf/robots/bcr_arm.urdf.xacro | -->
<!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 | -->
<!-- =================================================================================== -->
<robot name="bcr_arm">
  <!-- Material definitions for RViz -->
  <material name="black">
    <color rgba="0.165 0.165 0.165 1.0"/>
  </material>
  <material name="dark_grey">
    <color rgba="0.376 0.376 0.376 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.565 0.565 0.565 1.0"/>
  </material>
  <!-- Connect to world frame -->
  <link name="world"/>
  <joint name="world_to_base" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>
  <!-- RViz Material Definitions -->
  <material name="color_black">
    <color rgba="0.165 0.165 0.165 1"/>
  </material>
  <material name="color_dark_grey">
    <color rgba="0.376 0.376 0.376 1"/>
  </material>
  <material name="color_grey">
    <color rgba="0.565 0.565 0.565 1"/>
  </material>
  <material name="color_light_grey">
    <color rgba="0.690 0.690 0.690 1"/>
  </material>
  <material name="color_white">
    <color rgba="0.961 0.961 0.961 1"/>
  </material>
  <!-- Ignition Gazebo Material Definitions -->
  <gazebo reference="base_link">
    <material>
      <ambient>0.165 0.165 0.165 1</ambient>
      <diffuse>0.165 0.165 0.165 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <emissive>0 0 0 0</emissive>
    </material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>
  <gazebo reference="link1">
    <material>
      <ambient>0.565 0.565 0.565 1</ambient>
      <diffuse>0.565 0.565 0.565 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <emissive>0 0 0 0</emissive>
    </material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>
  <gazebo reference="link2">
    <material>
      <ambient>0.961 0.961 0.961 1</ambient>
      <diffuse>0.961 0.961 0.961 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <emissive>0 0 0 0</emissive>
    </material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>
  <gazebo reference="link3">
    <material>
      <ambient>0.565 0.565 0.565 1</ambient>
      <diffuse>0.565 0.565 0.565 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <emissive>0 0 0 0</emissive>
    </material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>
  <gazebo reference="link4">
    <material>
      <ambient>0.961 0.961 0.961 1</ambient>
      <diffuse>0.961 0.961 0.961 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <emissive>0 0 0 0</emissive>
    </material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>
  <gazebo reference="link5">
    <material>
      <ambient>0.565 0.565 0.565 1</ambient>
      <diffuse>0.565 0.565 0.565 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <emissive>0 0 0 0</emissive>
    </material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>
  <gazebo reference="link6">
    <material>
      <ambient>0.961 0.961 0.961 1</ambient>
      <diffuse>0.961 0.961 0.961 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <emissive>0 0 0 0</emissive>
    </material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>
  <gazebo reference="flange">
    <material>
      <ambient>0.376 0.376 0.376 1</ambient>
      <diffuse>0.376 0.376 0.376 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <emissive>0 0 0 0</emissive>
    </material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>
  <!-- Kinematic Chain -->
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/base_joint.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0.2"/>
      <geometry>
        <cylinder length="0.050" radius="0.070"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin rpy="0 0 0" xyz="0 0 0.1"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-6.28" upper="6.28" velocity="2"/>
  </joint>
  <link name="link1">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/joint_shaft_j1.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/link_l1.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0.8"/>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/joint_housing_j2.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0.4"/>
      <geometry>
        <cylinder length="0.7200000000000001" radius="0.030"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin rpy="0 0 0" xyz="0 0 0.8"/>
    <axis xyz="1 0 0"/>
    <limit effort="100" lower="-2.0" upper="2.0" velocity="2"/>
  </joint>
  <link name="link2">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/joint_output_j2.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="0.26 0 0"/>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/joint_housing_j3.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 1.5707963267948966 0" xyz="0.13 0 0"/>
      <geometry>
        <cylinder length="0.39" radius="0.030"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint3" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <origin rpy="0 0 0" xyz="0.26 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-6.28" upper="6.28" velocity="2"/>
  </joint>
  <link name="link3">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/joint_shaft_j3.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/link_l2.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 1.64"/>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/joint_housing_j4.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0.82"/>
      <geometry>
        <cylinder length="1.5579999999999998" radius="0.030"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint4" type="revolute">
    <parent link="link3"/>
    <child link="link4"/>
    <origin rpy="0 0 0" xyz="0 0 1.64"/>
    <axis xyz="1 0 0"/>
    <limit effort="80" lower="-2.0" upper="2.0" velocity="2"/>
  </joint>
  <link name="link4">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/joint_output_j4.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="-0.26 0 0"/>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/joint_housing_j5.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 1.5707963267948966 0" xyz="-0.13 0 0"/>
      <geometry>
        <cylinder length="0.39" radius="0.030"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint5" type="revolute">
    <parent link="link4"/>
    <child link="link5"/>
    <origin rpy="0 0 0" xyz="-0.26 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="50" lower="-6.28" upper="6.28" velocity="3"/>
  </joint>
  <link name="link5">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/joint_shaft_j5.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/link_l3.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 1.24"/>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/joint_housing_j6.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0.62"/>
      <geometry>
        <cylinder length="1.178" radius="0.025"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint6" type="revolute">
    <parent link="link5"/>
    <child link="link6"/>
    <origin rpy="0 0 0" xyz="0 0 1.24"/>
    <axis xyz="1 0 0"/>
    <limit effort="50" lower="-2.0" upper="2.0" velocity="3"/>
  </joint>
  <link name="link6">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/joint_output_j6.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="0.24 0 0"/>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/joint_housing_j7.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 1.5707963267948966 0" xyz="0.12 0 0"/>
      <geometry>
        <cylinder length="0.36" radius="0.030"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint7" type="revolute">
    <parent link="link6"/>
    <child link="flange"/>
    <origin rpy="0 0 0" xyz="0.24 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="30" lower="-6.28" upper="6.28" velocity="4"/>
  </joint>
  <link name="flange">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/joint_shaft_j7.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/link_l4.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0.42"/>
      <geometry>
        <mesh filename="package://bcr_arm_description/meshes/tool_flange.dae" scale="0.004 0.004 0.004"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0.21"/>
      <geometry>
        <cylinder length="0.378" radius="0.025"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="gripper_tip"/>
  <joint name="flange_to_gripper_tip" type="fixed">
    <parent link="flange"/>
    <child link="gripper_tip"/>
    <origin rpy="0 0 0" xyz="0.25 0 0"/>
  </joint>
  <link name="custom_gripper_tip"/>
  <joint name="flange_to_custom_gripper_tip" type="fixed">
    <parent link="flange"/>
    <child link="custom_gripper_tip"/>
    <origin rpy="0 0 0" xyz="0.25 0.00 -0.05"/>
  </joint>
</robot>
'''
