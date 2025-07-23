#!/usr/bin/env python3
import sys
import time
from typing import Optional

import cv2
import numpy as np
import pygame
from pygame.locals import KEYDOWN

import multiprocessing
import time
from pathlib import Path
from queue import Empty
from typing import Optional

import cv2
import numpy as np
import tyro
from loguru import logger


from dex_retargeting.retargeting_config import RetargetingConfig
from single_hand_detector import SingleHandDetector


def run_sim_teleop(
    queue: multiprocessing.Queue,
    config_path: str,
    robot_dir: str,
    camera_path: str,
    gui: bool,
):
    """
    在 Isaac Sim 中使用 Teleop (鍵盤或 MediaPipe) 驅動 LeapHand。  
    Args:
        config_path: retargeting config 的檔案路徑
        camera_path: 可選的攝影機路徑
        keyboard: 如果為 True，使用鍵盤輸入；否則使用 MediaPipe
        gui: 如果為 True，開啟 GUI；否則 headless 模式
    """
    # 1. 啟動 SimulationApp（必須最先）
    from omni.isaac.kit import SimulationApp
    headless = not gui
    app = SimulationApp({
        "headless": headless,
        "extensions": {"disable": ["omni.kit.test"]},
    })

    # 2. SimulationApp 建立後，再 import 需要的模組
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.types import ArticulationAction

    class LeapHandSimDriver:
        """控制 Isaac Sim 中的 LeapHand 關節"""
        def __init__(self, robot, hand_idx: list, step: float = 0.05):
            self.robot = robot
            self.hand_idx = hand_idx
            self.joint_angles = np.zeros(len(hand_idx), dtype=float)
            self.step = step
            self.current = self.robot.get_joint_positions()

        def update(self, joint_angles: np.ndarray):
            print("=== Finger Joint Angles ===")
            print(f"Index  - Side: {joint_angles[4]: .1f}, Forward: {joint_angles[0]: .1f}, PIP: {joint_angles[8]: .1f}, DIP: {joint_angles[12]: .1f}")
            print(f"Middle - Side: {joint_angles[6]: .1f}, Forward: {joint_angles[2]: .1f}, PIP: {joint_angles[10]: .1f}, DIP: {joint_angles[14]: .1f}")
            print(f"Ring   - Side: {joint_angles[7]: .1f}, Forward: {joint_angles[3]: .1f}, PIP: {joint_angles[15]: .1f}, DIP: {joint_angles[11]: .1f}")
            print(f"Thumb  - Side: {joint_angles[1]: .1f}, Forward: {joint_angles[5]: .1f}, PIP: {joint_angles[9]: .1f}, DIP: {joint_angles[13]: .1f}")
            self.current[self.hand_idx] = joint_angles
            self.robot.apply_action(ArticulationAction(joint_positions=self.current))

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


    # 3. 載入 USD
    usd_path = "/home/lai/ros2_ws/src/franka_description/robots/fr3/fr3_with_leaphand/fr3_with_leaphand.usd"
    prim_path = "/World/Franka"
    world = World()
    add_reference_to_stage(usd_path, prim_path)
    world.reset()

    # 4. 建立 Articulation
    robot = SingleArticulation(prim_path)
    robot.initialize()

    # 5. Retargeting 設定
    RetargetingConfig.set_default_urdf_dir(str(robot_dir))
    retargeting = RetargetingConfig.load_from_file(config_path).build()
    hand_type = "Right" if "right" in config_path.lower() else "Left"
    detector = SingleHandDetector(hand_type=hand_type, selfie=False)
    
    # 6. 找出手部 DOF indices
    dof_names = robot.dof_names
    hand_idx = [i for i, n in enumerate(dof_names) if not n.startswith("fr3_")]
    driver = LeapHandSimDriver(robot, hand_idx, step=0.05)

    # 7. 如果使用 MediaPipe，就打開攝影機
    # cap = None
    # if not keyboard:
    #     cap = cv2.VideoCapture(camera_path or 0)
    #     if not cap.isOpened():
    #         raise RuntimeError(f"Cannot open camera: {camera_path}")
        
    # 8. 初始化 pygame
    pygame.init()
    screen = pygame.display.set_mode((300, 300))
    pygame.display.set_caption("LeapHand Sim Teleop")
    
    try:
        while app.is_running():
            # 處理鍵盤輸入
            for event in pygame.event.get():
                if event.type == KEYDOWN:
                    key = pygame.key.name(event.key)
                    driver.handle_key_event(key)
            # 如果使用 MediaPipe，讀取影像並 retarget
            # if not keyboard and cap:
            #     ret, frame = cap.read()
            #     if ret:
            #         rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            #         _, joint_pos, _, _ = detector.detect(rgb)
            #         if joint_pos is not None:
            #             indices = retargeting.optimizer.target_link_human_indices
            #             if retargeting.optimizer.retargeting_type == "POSITION":
            #                 ref = joint_pos[indices, :]
            #             else:
            #                 org, task = indices
            #                 ref = joint_pos[task, :] - joint_pos[org, :]
            #                 print("Optimizer: ", retargeting.optimizer.retargeting_type)
            #             qpos = retargeting.retarget(ref)
            #             # 重新排序到 q
            #             q = np.zeros_like(qpos)
            #             q[0], q[1], q[2], q[3] = qpos[1], qpos[0], qpos[2], qpos[3]
            #             q[4], q[5], q[6], q[7] = qpos[9], qpos[8], qpos[10], qpos[11]
            #             q[8], q[9], q[10], q[11] = qpos[13], qpos[12], qpos[14], qpos[15]
            #             q[12], q[13], q[14], q[15] = qpos[4], qpos[5], qpos[6], qpos[7]
            #             driver.update(q)
            try:
                rgb = queue.get(timeout=50)
            except Empty:
                logger.error(f"Fail to fetch image from camera in 50 secs. Please check your web camera device.")
                return

            _, joint_pos, _, _ = detector.detect(rgb)
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
                # print("qpos: " + ", ".join(f"{pos:.4f}" for pos in qpos))

                qpos_cmd = np.zeros(16)
                # current_pos = leaphand.read_pos()
                # diff = (qpos[0]- current_pos[0] + 3.14) 
                # if  abs(diff)> 0.001:
                #     diff = np.sign(diff) * 0.001
                    
                # qpos_cmd[0] =  current_pos[0] +  diff 

                # qpos_cmd[0] = qpos[0]
                # qpos_cmd[1] = qpos[1]
                # qpos_cmd[2] = qpos[2]
                # qpos_cmd[3] = qpos[3]

                # qpos_cmd[4] = qpos[8] # thumb - middle
                # qpos_cmd[5] = qpos[9]
                # qpos_cmd[6] = qpos[10]
                # qpos_cmd[7] = qpos[11]

                # qpos_cmd[8] = qpos[12] # none
                # qpos_cmd[9] = qpos[13]
                # qpos_cmd[10] = qpos[14]
                # qpos_cmd[11] = qpos[15]

                # qpos_cmd[12] = qpos[4] # thumb - middle 
                # qpos_cmd[13] = qpos[5]
                # qpos_cmd[14] = qpos[6]
                # qpos_cmd[15] = qpos[7]

                # ['1', '0', '2', '3', '12', '13', '14', '15', '5', '4', '6', '7', '9', '8', '10', '11']

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

                # qpos_cmd[8] = qpos[8]        

                # qpos_cmd = qpos
                # print(f"{qpos_cmd[1]:.4f}")
                # print("qpos_cmd: " + ", ".join(f"{pos:.4f}" for pos in qpos_cmd))
                end_t = time.time()
                # print(f"time: {end_t - start_t:.4f} s")
                driver.joint_angles = qpos_cmd
                #driver.update(qpos_cmd)
            # 應用目前角度到模擬
            driver.apply()
            world.step(render=True)
    finally:
        # if cap:
        #     cap.release()
        pygame.quit()
        app.close()