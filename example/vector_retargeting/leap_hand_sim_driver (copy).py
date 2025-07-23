#!/usr/bin/env python3
import sys
import time
from typing import Optional

import cv2
import numpy as np
import pygame
from pygame.locals import KEYDOWN

import multiprocessing
from pathlib import Path
from queue import Empty

import tyro
from loguru import logger

from dex_retargeting.retargeting_config import RetargetingConfig
from single_hand_detector import SingleHandDetector

# import math, threading, rclpy
# from rclpy.node import Node
from sensor_msgs.msg import JointState

def find_articulation_root():
    import omni.usd as usd, pxr
    stage = usd.get_context().get_stage()
    for prim in stage.TraverseAll():
        if prim.HasAPI(pxr.UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    raise RuntimeError("❌ No ArticulationRoot found")

def run_sim_teleop(
    queue: multiprocessing.Queue,
    config_path: str,
    robot_dir: str,
    camera_path: str,
    gui: bool,
):
    """
    Use Teleoperation (keyboard or MediaPipe) to drive the LeapHand in Isaac Sim.  
    Args:
        config_path: Path to the retargeting configuration file  
        camera_path: Optional path to the camera  
        keyboard: If True, use keyboard input; otherwise, use MediaPipe  
        gui: If True, launch with GUI; otherwise, run in headless mode  
    """
    # 1. SimulationApp（you need to startup SimulationApp before import other isaac packages)
    from omni.isaac.kit import SimulationApp
    headless = not gui
    app = SimulationApp({
        "headless": headless,
        "enabled": ["omni.isaac.ros2_bridge"],
        "extensions": {"disable": ["omni.kit.test"]},
    })

    # 2. import packages after SimulationApp
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.types import ArticulationAction

    # Camera in isaac sim
    import omni.ui as ui
    last_frame = None
    cam_w, cam_h = 640, 480
    provider = ui.ByteImageProvider()
    window = ui.Window("Real Camera Feed", width=cam_w, height=cam_h)
    with window.frame:
        camera_widget = ui.ImageWithProvider(provider, width=cam_w, height=cam_h)


    class LeapHandSimDriver:
        """Control Isaac Sim LeapHand Joint"""
        def __init__(self, robot, hand_idx, step=0.05):
            self.robot = robot
            self.hand_idx = np.asarray(hand_idx, dtype=np.int32)   # store as ndarray
            self.joint_angles = np.zeros(len(hand_idx), dtype=float)
            self.step = step

        def update(self, joint_angles: np.ndarray):
            print("=== Finger Joint Angles ===")
            print(f"Index  - Side: {joint_angles[4]: .1f}, Forward: {joint_angles[0]: .1f}, PIP: {joint_angles[8]: .1f}, DIP: {joint_angles[12]: .1f}")
            print(f"Middle - Side: {joint_angles[6]: .1f}, Forward: {joint_angles[2]: .1f}, PIP: {joint_angles[10]: .1f}, DIP: {joint_angles[14]: .1f}")
            print(f"Ring   - Side: {joint_angles[7]: .1f}, Forward: {joint_angles[3]: .1f}, PIP: {joint_angles[15]: .1f}, DIP: {joint_angles[11]: .1f}")
            print(f"Thumb  - Side: {joint_angles[1]: .1f}, Forward: {joint_angles[5]: .1f}, PIP: {joint_angles[9]: .1f}, DIP: {joint_angles[13]: .1f}")
            # Create *partial* action – only the hand joints are listed
            action = ArticulationAction(
                joint_positions=joint_angles.astype(np.float32),
                joint_indices=self.hand_idx               # <-- key line
            )
            self.robot.apply_action(action)

        def apply(self):
            self.update(self.joint_angles)

        def handle_key_event(self, key_name: str):
            key_map = {
                #index
                'x': (0),   # index forward
                's': (4),   # index side
                'w': (8),   # index PIP
                '2': (12),  # index DIP

                # thumb
                'z': (1),   # thumb side
                'a': (5),   # thumb forward
                'q': (9),   # thumb PIP
                '1': (13),  # thumb DIP

                # middle
                'c': (2),   # middle forward
                'd': (6),   # middle side
                'e': (10),  # middle PIP
                '3': (14),  # middle DIP

                # ring
                'v': (3),   # ring forward   
                'f': (7),   # ring side
                'r': (11),  # ring PIP
                '4': (15),  # ring DIP
            }
            if key_name == '-':
                direction = -1
            else:
                direction = 1

            if key_name in key_map:
                idx = key_map[key_name]
                self.joint_angles[idx] += direction * self.step
                self.joint_angles[idx] = float(np.clip(self.joint_angles[idx], 0.0, 1.5))


    # ------------------------------------------------------------------
    # Startup ROS 2 Bridge  (4.5.0)
    # ------------------------------------------------------------------
    import omni.kit.app as kit
    ext_mgr = kit.get_app().get_extension_manager()
    ext_mgr.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Load FR3 USD → reset，let Stage really have ArticulationRoot
    # ------------------------------------------------------------------
    usd_path = str(Path(__file__).resolve().parents[2] / "src/franka_description/robots/fr3/fr3_with_leaphand/fr3_with_leaphand.usd")
    prim_path = "/World/Franka"          # ← from Stage panel ArticulationRoot
    world = World()
    add_reference_to_stage(usd_path, prim_path)
    world.reset()
    # ------------------------------------------------------------------
    prim_path = find_articulation_root()
    print("✅ USING ARTICULATION ROOT:", prim_path)

    # ------------------------------------------------------------------
    # Action Graph (node type all use omni.isaac.ros2_bridge）
    # ------------------------------------------------------------------\
    import omni.graph.core as og
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Tick",   "omni.graph.action.OnPlaybackTick"),
                ("PubJS",  "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubJS",  "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("AC",     "isaacsim.core.nodes.IsaacArticulationController"),
                ("Time",   "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ],
            og.Controller.Keys.CONNECT: [
                ("Tick.outputs:tick",  "PubJS.inputs:execIn"),
                ("Tick.outputs:tick",  "SubJS.inputs:execIn"),
                ("Tick.outputs:tick",  "AC.inputs:execIn"),

                ("Time.outputs:simulationTime", "PubJS.inputs:timeStamp"),

                ("SubJS.outputs:jointNames",      "AC.inputs:jointNames"),
                ("SubJS.outputs:positionCommand", "AC.inputs:positionCommand"),
                ("SubJS.outputs:velocityCommand", "AC.inputs:velocityCommand"),
                ("SubJS.outputs:effortCommand",   "AC.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("AC.inputs:robotPath",        prim_path),   # point to ArticulationRoot
                ("PubJS.inputs:targetPrim",    prim_path),

                ("PubJS.inputs:topicName",     "/fr3/joint_states"),
                ("SubJS.inputs:topicName",     "/fr3/joint_command"),
            ],
        },
    )
    print("✅ Graph builded，Bridge node: isaacsim.ros2.bridge.* Ready")

    # 4. Build Articulation
    robot = SingleArticulation(prim_path)
    robot.initialize()

    # Set initial joint positions
    init_qpos = robot.get_joint_positions()
    init_qpos[5] = 0.5   # index 5 = fr3_joint6 (0-based index)
    robot.set_joint_positions(init_qpos)

    # 5. Retargeting setting
    RetargetingConfig.set_default_urdf_dir(str(robot_dir))
    retargeting = RetargetingConfig.load_from_file(config_path).build()
    hand_type = "Right" if "right" in config_path.lower() else "Left"
    detector = SingleHandDetector(hand_type=hand_type, selfie=False)
    
    # 6. 找出手部 DOF indices
    dof_names = robot.dof_names
    hand_idx = [i for i, n in enumerate(dof_names) if not n.startswith("fr3_")]
    driver = LeapHandSimDriver(robot, hand_idx, step=0.05)
        
    # 7. pygame
    pygame.init()
    screen = pygame.display.set_mode((300, 300))
    pygame.display.set_caption("LeapHand Sim Teleop")
    
    #
    # 7) （Optional）
    #
    # for i, jp in enumerate(robot.joint_paths):
    #     drive = PhysxSchema.PhysxJointDriveAPI.Apply(get_prim_at_path(jp), "angular")
    #     if i < 5:
    #         drive.GetStiffnessAttr().Set(60000); drive.GetDampingAttr().Set(600)
    #         mode = "position"
    #     else:
    #         drive.GetStiffnessAttr().Set(0); drive.GetDampingAttr().Set(50)
    #         mode = "velocity"
    #     print(f"[DEBUG] Drive set for {jp}: mode={mode}, stiffness={drive.GetStiffnessAttr().Get()}, damping={drive.GetDampingAttr().Get()}")

    try:
        while app.is_running():
            st = world.current_time  # 或用讀取 sim time node
            jp = robot.get_joint_positions()
            print(f"[DEBUG] sim time={st:.3f}s  current qpos={[round(x,3) for x in jp]}")
            world.step(render=True)
            
            # keyboard input for teleoperation
            for event in pygame.event.get():
                if event.type == KEYDOWN:
                    key = pygame.key.name(event.key)
                    driver.handle_key_event(key)
                    driver.apply()
                    
            try:
                # rgb = queue.get(timeout=50)
                rgb = queue.get_nowait()
                last_frame = rgb
            except Empty:
                #logger.error(f"Fail to fetch image from camera in 50 secs. Please check your web camera device.")
                rgb = last_frame

            if rgb is not None: 
                _, joint_pos, keypoint_2d, _ = detector.detect(rgb)
                if keypoint_2d is not None:
                    # draw_skeleton_on_image 
                    annotated_rgb = detector.draw_skeleton_on_image(
                        rgb.copy(),       
                        keypoint_2d,            # NormalizedLandmarkList
                        style="white"           # "default"
                    )
                else:
                    annotated_rgb = rgb

                # Isaac UI（RGBA uint8）
                rgba = cv2.cvtColor(annotated_rgb, cv2.COLOR_RGB2RGBA)
                h, w, _ = rgba.shape
                provider.set_data_array(rgba, [w, h])

                if joint_pos is None:
                    logger.warning(f"{hand_type} hand is not detected.")
                else:
                    retargeting_type = retargeting.optimizer.retargeting_type
                    indices = retargeting.optimizer.target_link_human_indices
                    if retargeting_type == "POSITION":
                        indices = indices
                        ref_value = joint_pos[indices, :]
                    else:
                        origin_indices = indices[0, :]
                        task_indices = indices[1, :]
                        ref_value = joint_pos[task_indices, :] - joint_pos[origin_indices, :]
                    qpos = retargeting.retarget(ref_value)
                    print(retargeting_type + " qpos: " + ", ".join(f"{pos:.4f}" for pos in qpos))

                    qpos_cmd = np.zeros(16)

                    qpos_cmd[4] = qpos[1] # index side
                    qpos_cmd[0] = qpos[0] # index forward
                    qpos_cmd[8] = qpos[2]
                    qpos_cmd[12] = qpos[3]

                    qpos_cmd[6] = qpos[9] # middle side
                    qpos_cmd[2] = qpos[8]
                    qpos_cmd[10] = qpos[10]
                    qpos_cmd[14] = qpos[11]

                    qpos_cmd[7] = qpos[13] # ring side
                    qpos_cmd[3] = qpos[12]
                    qpos_cmd[15]= qpos[14]
                    qpos_cmd[11] = qpos[15]

                    qpos_cmd[1] = qpos[4] # thumb side
                    qpos_cmd[5] = qpos[5] # thumb forward
                    qpos_cmd[9] = qpos[6]
                    qpos_cmd[13] = qpos[7]    

                    # qpos_cmd = qpos
                    # print(f"{qpos_cmd[1]:.4f}")
                    # print("qpos_cmd: " + ", ".join(f"{pos:.4f}" for pos in qpos_cmd))
                    end_t = time.time()
                    # print(f"time: {end_t - start_t:.4f} s")
                    driver.joint_angles = qpos_cmd
                    #driver.update(qpos_cmd)
                    driver.apply()
            world.step(render=True)
    finally:
        # if cap:
        #     cap.release()
        pygame.quit()
        # rclpy.shutdown()
        app.close()