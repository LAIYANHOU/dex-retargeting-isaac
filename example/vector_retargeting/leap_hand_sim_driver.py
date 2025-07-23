#!/usr/bin/env python3
import sys
import time
from typing import Optional
import os

import cv2
import numpy as np
import pygame
from pygame.locals import KEYDOWN

import multiprocessing
from pathlib import Path
from queue import Empty

import tyro

from dex_retargeting.retargeting_config import RetargetingConfig
from single_hand_detector import SingleHandDetector

import ctypes, os, sys

LULA_PATH = os.path.expanduser(
    "~/isaacsim/kit/python/lib/python3.10/site-packages/isaacsim/exts/isaacsim.robot_motion.lula/pip_prebundle/_lula_libs")

# ① First, load the built-in urdfdom as RTLD_GLOBAL so that downstream dependencies can find the correct symbols
for lib in ["liburdfdom_model.so.3.0", "liblula_motion_planning.so", "liblula_rmpflow.so", "liblula_trajectory.so"]:
    ctypes.CDLL(os.path.join(LULA_PATH, lib), mode=ctypes.RTLD_GLOBAL)  # :contentReference[oaicite:2]{index=2}

# ② Then, load the core Lula library
ctypes.CDLL(os.path.join(LULA_PATH, "liblula_kinematics.so"), mode=ctypes.RTLD_GLOBAL)

# ③ Put the path at the front of sys.path to prevent Python from finding the ROS version of modules with the same name
sys.path.insert(0, LULA_PATH)

# ④ Now it's safe to import Lula and ROS
import lula                        
import urdf_parser_py.urdf as _u

# ros_bridge.py or at the top of your main script
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R

class ROS2BridgeNode(Node):
    """
    處理 Isaac Sim 與 ROS2 之間通訊的橋樑節點。
    - 訂閱 /target_pose
    - 發布 /joint_states
    - 發布 /end_effector_pose
    """
    def __init__(self):
        super().__init__('isaac_sim_bridge')

        # --- Subscriber ---
        self.subscription = self.create_subscription(
            PoseStamped, '/target_pose', self.listener_callback, 10)
        self.target_position: Optional[np.ndarray] = None
        self.target_orientation: Optional[np.ndarray] = None

        # --- Publishers ---
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.cartesian_state_publisher = self.create_publisher(PoseStamped, '/franka_robot_state_broadcaster/current_pose', 10)

        self.get_logger().info("✅ Isaac Sim Bridge Node is running.")

    def listener_callback(self, msg: PoseStamped):
        self.target_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.target_orientation = np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])

    def publish_joint_states(self, dof_names: list, dof_positions: np.ndarray):
        """發布機器人的關節狀態"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = dof_names
        msg.position = dof_positions.tolist()
        self.joint_state_publisher.publish(msg)

    def publish_cartesian_state(self, position: np.ndarray, orientation: np.ndarray):
        try:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"

            # 直接讀 1D position
            msg.pose.position.x = float(position[0])
            msg.pose.position.y = float(position[1])
            msg.pose.position.z = float(position[2])

            # 將旋轉矩陣轉 quaternion
            r = R.from_matrix(orientation)
            qw, qx, qy, qz = r.as_quat(scalar_first=True)
            msg.pose.orientation.w = float(qw)
            msg.pose.orientation.x = float(qx)
            msg.pose.orientation.y = float(qy)
            msg.pose.orientation.z = float(qz)

            self.cartesian_state_publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Failed in publish_cartesian_state: {e}")

def publish_initial_target_pose_once(node, publisher):
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'world'
    
    x, y, z = 0.5, 0.0, 0.2
    roll, pitch, yaw = -180.0, 0.0, 0.0
    quat = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
    
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.x = quat[0]
    msg.pose.orientation.y = quat[1]
    msg.pose.orientation.z = quat[2]
    msg.pose.orientation.w = quat[3]
    
    publisher.publish(msg)
    node.get_logger().info(f"✅ One-shot target_pose: pos=({x:.2f}, {y:.2f}, {z:.2f}) rpy=({roll}, {pitch}, {yaw})")

def find_articulation_root():
    import omni.usd as usd, pxr
    stage = usd.get_context().get_stage()
    for prim in stage.TraverseAll():
        if prim.HasAPI(pxr.UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    raise RuntimeError("❌ No ArticulationRoot found")

def find_articulation_root():
    import omni.usd as usd, pxr
    stage = usd.get_context().get_stage()
    for prim in stage.TraverseAll():
        if prim.HasAPI(pxr.UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    raise RuntimeError("❌ No ArticulationRoot found")

# 函式：更新 prim world transform
def update_target_pose_prim(prim_path: str, position: tuple, orientation_quat_wxyz: tuple):
    """
    Update a prim's world pose using USD low-level API (Isaac Sim 4.5 compatible).
    Args:
        prim_path: full USD path of the prim
        position: (x, y, z)
        orientation_quat_wxyz: (w, x, y, z)
    """
    from pxr import UsdGeom, Gf, Sdf
    import omni.usd

    stage = omni.usd.get_context().get_stage()

    # Create prim if not exist
    if not stage.GetPrimAtPath(prim_path).IsValid():
        UsdGeom.Xform.Define(stage, Sdf.Path(prim_path))

    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    op = xform.AddTransformOp()

    mat = Gf.Matrix4d()
    mat.SetTranslateOnly(Gf.Vec3d(*position))

    quat = Gf.Quatd(
        orientation_quat_wxyz[0],  # w
        orientation_quat_wxyz[1],  # x
        orientation_quat_wxyz[2],  # y
        orientation_quat_wxyz[3],  # z
    )
    rot = Gf.Rotation(quat)
    mat.SetRotateOnly(rot)

    op.Set(mat)

# ======================================================================
# --- NEW: Dedicated function for the Hand-Tracking Process ---
# This function contains all the heavy logic and runs in its own process.
# ======================================================================
def run_hand_tracking_and_retargeting(
    image_queue: multiprocessing.Queue,
    command_queue: multiprocessing.Queue,
    annotated_image_queue: multiprocessing.Queue, # <-- NEW
    config_path: str,
    robot_dir: str,
):
    from dex_retargeting.retargeting_config import RetargetingConfig
    from single_hand_detector import SingleHandDetector
    print("✅ Hand-Tracking Process Initialized.")

    RetargetingConfig.set_default_urdf_dir(str(robot_dir))
    retargeting = RetargetingConfig.load_from_file(config_path).build()
    hand_type = "Right" if "right" in config_path.lower() else "Left"
    detector = SingleHandDetector(hand_type=hand_type, selfie=False)

    while True:
        try:
            rgb = image_queue.get()
            if rgb is None: continue

            _, joint_pos, keypoint_2d, _ = detector.detect(rgb)
            if keypoint_2d is not None:
                annotated_rgb = detector.draw_skeleton_on_image(rgb.copy(),keypoint_2d,style="white")
            else:
                annotated_rgb = rgb
            try:
                annotated_image_queue.put_nowait(annotated_rgb)
            except multiprocessing.queues.Full:
                pass # Drop frame if Isaac Sim UI is slow

            if joint_pos is not None:
                retargeting_type = retargeting.optimizer.retargeting_type
                indices = retargeting.optimizer.target_link_human_indices
                if retargeting_type == "POSITION":
                    ref_value = joint_pos[indices, :]
                else:
                    origin_indices, task_indices = indices[0, :], indices[1, :]
                    ref_value = joint_pos[task_indices, :] - joint_pos[origin_indices, :]

                qpos = retargeting.retarget(ref_value)
                qpos_cmd = np.zeros(16)
                qpos_cmd[4]=qpos[1];qpos_cmd[0]=qpos[0];qpos_cmd[8]=qpos[2];qpos_cmd[12]=qpos[3]
                qpos_cmd[6]=qpos[9];qpos_cmd[2]=qpos[8];qpos_cmd[10]=qpos[10];qpos_cmd[14]=qpos[11]
                qpos_cmd[7]=qpos[13];qpos_cmd[3]=qpos[12];qpos_cmd[15]=qpos[14];qpos_cmd[11]=qpos[15]
                qpos_cmd[1]=qpos[4];qpos_cmd[5]=qpos[5];qpos_cmd[9]=qpos[6];qpos_cmd[13]=qpos[7]

                command_queue.put(qpos_cmd)
        except Exception as e:
            print(f"[Hand-Tracking Process ERROR]: {e}")
            time.sleep(1)

# ======================================================================
# --- MODIFIED: The Main Isaac Sim Function ---
# This is now lightweight. It handles simulation, IK, and control.
# ======================================================================
def run_sim_teleop(
    command_queue: multiprocessing.Queue,
    annotated_image_queue: multiprocessing.Queue,
    config_path: str,
    robot_dir: str,
    camera_path: str,
    gui: bool,
):
    # --- 1. App and Module Imports ---
    from omni.isaac.kit import SimulationApp
    app = SimulationApp({"headless": not gui})

    # ------------------------------------------------------------------
    # 1. [IMPORTANT] start SimulationApp and then we improt other modules
    # ------------------------------------------------------------------\
    from omni.isaac.core.utils.extensions import enable_extension
    enable_extension("omni.isaac.debug_draw")
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.articulations import Articulation as SingleArticulation
    from omni.isaac.core.utils.types import ArticulationAction
    import urdf_parser_py.urdf as _u # laod urdfdom parser library
    import lula
    from omni.isaac.motion_generation import LulaKinematicsSolver, ArticulationKinematicsSolver
    import omni.ui as ui
    import omni, pxr
    from pxr import UsdGeom, Sdf, Gf

   # ------------------------------------------------------------------
    # 2. LeapHandSimDriver Class
    # ------------------------------------------------------------------\
    class LeapHandSimDriver:
        def __init__(self, robot, hand_idx, step=0.05):
            self.robot = robot
            self.hand_idx = np.asarray(hand_idx, dtype=np.int32)
            self.joint_angles = np.zeros(len(hand_idx), dtype=float)
            self.step = step
        def update(self, joint_angles: np.ndarray):
            # print("=== Finger Joint Angles ===")
            # print(f"Index  - Side: {joint_angles[4]: .1f}, Forward: {joint_angles[0]: .1f}, PIP: {joint_angles[8]: .1f}, DIP: {joint_angles[12]: .1f}")
            # print(f"Middle - Side: {joint_angles[6]: .1f}, Forward: {joint_angles[2]: .1f}, PIP: {joint_angles[10]: .1f}, DIP: {joint_angles[14]: .1f}")
            # print(f"Ring   - Side: {joint_angles[7]: .1f}, Forward: {joint_angles[3]: .1f}, PIP: {joint_angles[15]: .1f}, DIP: {joint_angles[11]: .1f}")
            # print(f"Thumb  - Side: {joint_angles[1]: .1f}, Forward: {joint_angles[5]: .1f}, PIP: {joint_angles[9]: .1f}, DIP: {joint_angles[13]: .1f}")
            action = ArticulationAction(joint_positions=joint_angles.astype(np.float32), joint_indices=self.hand_idx)
            self.robot.apply_action(action)
        def apply(self):
            self.update(self.joint_angles)
        def handle_key_event(self, key_name: str):
            key_map = {'x':0,'s':4,'w':8,'2':12,'z':1,'a':5,'q':9,'1':13,'c':2,'d':6,'e':10,'3':14,'v':3,'f':7,'r':11,'4':15}
            if key_name=='-': direction=-1
            else: direction=1
            if key_name in key_map:
                idx=key_map[key_name]
                self.joint_angles[idx]+=direction*self.step
                self.joint_angles[idx]=float(np.clip(self.joint_angles[idx],0.0,1.5))

    # ------------------------------------------------------------------
    # 3. Scene and Robot Initialization
    # ------------------------------------------------------------------\
    world = World()
    usd_path = str(Path(__file__).resolve().parents[2] / "src/franka_description/robots/fr3/fr3_with_leaphand/fr3_with_leaphand.usd")
    add_reference_to_stage(usd_path, "/World/Franka")
    world.reset()
    
    prim_path = find_articulation_root()
    robot = SingleArticulation(prim_path)
    robot.initialize()
    
    # ------------------------------------------------------------------
    # 4. core IK setting
    # ------------------------------------------------------------------\
    robot_description_path = str(Path(__file__).resolve().parents[2] / "src/franka_description/robots/fr3/fr3_robot_description.yaml")
    urdf_path = str(Path(__file__).resolve().parents[2] / "src/franka_description/robots/fr3/fr3_with_leaphand.urdf")
    if not (os.path.exists(robot_description_path) and os.path.exists(urdf_path)):
        raise FileNotFoundError("FATAL: Could not find robot description or URDF file at the specified absolute path.")

    kinematics_solver = LulaKinematicsSolver(robot_description_path=robot_description_path, urdf_path=urdf_path)
    articulation_kinematics_solver = ArticulationKinematicsSolver(
        robot_articulation=robot,
        kinematics_solver=kinematics_solver,
        end_effector_frame_name="fr3_link8"
    )
    print("✅ Motion Generation IK solvers initialized.")

    # ==================================================================
    # --- SECTION FOR VISUALIZATION SETUP ---
    # ==================================================================
    from pxr import UsdGeom, Sdf
    from omni.isaac.core.utils.extensions import enable_extension
    enable_extension("isaacsim.util.debug_draw")
    
    stage = omni.usd.get_context().get_stage()
    target_path = "/World/TargetPoseMarker"

    # 建立 prim
    if not stage.GetPrimAtPath(target_path).IsValid():
        UsdGeom.Xform.Define(stage, Sdf.Path(target_path))

    import omni.graph.core as og
    og.Controller.edit(
        "/AxisVisualizerGraph",
        {
            og.Controller.Keys.CREATE_NODES: [
                ("AxisViz", "isaacsim.util.debug_draw.IsaacXPrimAxisVisualizer"),
                ("EEAxis", "isaacsim.util.debug_draw.IsaacXPrimAxisVisualizer"),
                ("TargetAxis", "isaacsim.util.debug_draw.IsaacXPrimAxisVisualizer"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("AxisViz.inputs:xPrim", robot.prim_path),
                ("AxisViz.inputs:length", 0.2),
                ("AxisViz.inputs:thickness", 2.0),

                ("EEAxis.inputs:xPrim", "/World/Franka/fr3_link7/fr3_link8"),
                ("EEAxis.inputs:length", 0.2),
                ("EEAxis.inputs:thickness", 1.5),

                ("TargetAxis.inputs:xPrim", "/World/TargetPoseMarker"),
                ("TargetAxis.inputs:length", 0.1),
                ("TargetAxis.inputs:thickness", 2.0),
            ],
        },
    )
    
    # ------------------------------------------------------------------
    # 5. Initialize UI Provider for Camera Feed
    # ------------------------------------------------------------------\
    cam_w, cam_h = 640, 480  # Or your camera's resolution
    ui_provider = ui.ByteImageProvider()
    window = ui.Window("Camera Feed", width=cam_w, height=cam_h)
    with window.frame:
        ui.ImageWithProvider(ui_provider, width=cam_w, height=cam_h)

    # hand's DOF indices
    dof_names = robot.dof_names
    hand_idx = [i for i, n in enumerate(dof_names) if not n.startswith("fr3_")]
    driver = LeapHandSimDriver(robot, hand_idx, step=0.05)

    # === [ROS2] Initial ROS2
    rclpy.init()
    ros_bridge_node = ROS2BridgeNode()
    ros_logger = ros_bridge_node.get_logger()

    target_pose_pub = ros_bridge_node.create_publisher(PoseStamped, '/target_pose', 1)
    publish_initial_target_pose_once(ros_bridge_node, target_pose_pub)
    time.sleep(0.2)  # Allow message to propagate   

    # --- Add these lines for smoothing ---
    # Initializations
    last_joint_command = None

    # Smoothing params
    joint_eps = 2e-3     # Joint-level change threshold to apply command

    try:
        while app.is_running():
            world.step(render=True)
            
            # --- [ROS2] ---
            rclpy.spin_once(ros_bridge_node, timeout_sec=0.01)

            # 1. Safely publish the current joint states (Simpler Method)
            try:
                # This directly returns an array of joint positions
                joint_positions = robot.get_joint_positions()
                
                if joint_positions is not None:
                    ros_bridge_node.publish_joint_states(
                        dof_names=robot.dof_names, 
                        dof_positions=joint_positions
                    )
            except Exception as e:
                ros_logger.warn(f"Could not get/publish joint states: {e}")

            # 2. Safely publish the current end-effector pose
            try:
                ee_position, ee_orientation = articulation_kinematics_solver.compute_end_effector_pose()
                # --- Draw the TCP marker (Green) ---
                # debug_drawer.draw_point(ee_position, color=(0.0, 1.0, 0.0, 1.0), size=15)
                # -------------------------
                if ee_position is not None and ee_orientation is not None:
                    ros_bridge_node.publish_cartesian_state(
                        position=ee_position,
                        orientation=ee_orientation
                    )
            except Exception as e:
                ros_logger.warn(f"Could not get/publish end-effector pose: {e}")

            # --- IK 控制流程 ---
            if ros_bridge_node.target_position is not None:
                new_target = ros_bridge_node.target_position
                # --- Draw the Target marker ---
                try:
                    if ros_bridge_node.target_position is not None and ros_bridge_node.target_orientation is not None:
                        pos = tuple(ros_bridge_node.target_position)
                        ori = tuple(ros_bridge_node.target_orientation)
                        if len(pos) == 3 and len(ori) == 4:
                            update_target_pose_prim("/World/TargetPoseMarker", pos, ori)
                        else:
                            print(f"❌ Invalid target pose length: pos={pos}, ori={ori}")
                    else:
                        print("❌ target_position or target_orientation is None")
                except Exception as e:
                    print(f"❌ Exception when updating pose marker: {e}")
                # ----------------------------

                # ① 設定 base pose
                robot_base_translation, robot_base_orientation = robot.get_world_pose()
                kinematics_solver.set_robot_base_pose(robot_base_translation, robot_base_orientation)

                # ② 解 IK
                try:
                    action, success = articulation_kinematics_solver.compute_inverse_kinematics(
                        target_position=new_target,
                        target_orientation=ros_bridge_node.target_orientation
                    )
                except Exception as e:
                    ros_logger.error(f"💥 IK solver threw exception: {e}")
                    continue
                print(f"IK Action: {action.joint_positions} | Success: {success}")

                if success:
                    # 將結果轉為 numpy 陣列
                    try:
                        current_cmd = np.array(action.joint_positions)
                    except Exception as e:
                        ros_logger.error(f"❌ Could not extract joint_positions from action: {e}")
                        continue

                    # 判斷是否需要 apply
                    if last_joint_command is None or not np.allclose(current_cmd, last_joint_command, atol=joint_eps):
                        try:
                            ros_logger.info(f"✅ IK Action: {current_cmd}")
                            robot.apply_action(action)
                            last_joint_command = current_cmd
                        except Exception as e:
                            ros_logger.error(f"💥 apply_action() failed: {e}")
                            continue
                else:
                    ros_logger.warn("❌ IK solver failed to find a solution.")
            
            # --- MODIFIED: Lightweight hand control logic ---
            try:
                qpos_cmd = command_queue.get_nowait()
                driver.joint_angles = qpos_cmd
                driver.apply()
            except Empty:
                pass # No new command

            # --- Get Annotated Image and Update UI ---
            try:
                annotated_rgb = annotated_image_queue.get_nowait()
                # Convert to RGBA for the UI provider
                rgba = cv2.cvtColor(annotated_rgb, cv2.COLOR_RGB2RGBA)
                h, w, _ = rgba.shape
                # Update the UI widget
                ui_provider.set_data_array(rgba.flatten(), [w, h])
            except Empty:
                pass # No new image, do nothing
    finally:
        rclpy.shutdown()
        app.close()