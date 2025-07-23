#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from pathlib import Path

from ikpy.chain import Chain


class FR3IKSolverNode(Node):
    def __init__(self):
        super().__init__('fr3_ik_solver')

        # -----------------------------
        # 1. 載入 URDF 並建立 kinematic chain
        # -----------------------------
        usd_path = str(Path(__file__).resolve().parents[2] / "src/franka_description/robots/fr3/fr3_with_leaphand/fr3_with_leaphand.usd")
        URDF_PATH = os.getenv(
            'FR3_URDF',
            usd_path
        )
        # 指定 root link 名稱為 'base'，否則預設會找 'base_link'
        self.fr3_chain = Chain.from_urdf_file(
            URDF_PATH,
            base_elements=['base'],
        )

        # 【DEBUG】印出所有由 ikpy 建立的 link name，確認你要對應的正確字串
        actual_links = [link.name for link in self.fr3_chain.links]
        self.get_logger().info(f"IK chain links: {actual_links}")

        # -----------------------------
        # 2. Joint ↔ Link 對應表 (請依照實際輸出調整)
        # -----------------------------
        self.joint_to_link = {
            'fr3_joint1': 'fr3_link_1',
            'fr3_joint2': 'fr3_link_2',
            'fr3_joint3': 'fr3_link_3',
            'fr3_joint4': 'fr3_link_4',
            'fr3_joint5': 'fr3_link_5',
            'fr3_joint6': 'fr3_link_6',
            'fr3_joint7': 'fr3_link_7',
        }

        # -----------------------------
        # 3. Publisher & Subscriber
        # -----------------------------
        self.pub = self.create_publisher(JointState, '/fr3/joint_command', 10)
        self.create_subscription(
            JointState,
            '/fr3/joint_states',
            self.joint_state_callback,
            10
        )
        self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.target_pose_callback,
            10
        )

        # -----------------------------
        # 4. 初始關節狀態緩存
        # -----------------------------
        self.current_joint_state = [0.0] * len(self.fr3_chain.links)

        self.get_logger().info('FR3 IK Solver Node started.')

    def joint_state_callback(self, msg: JointState):
        """接收 /fr3/joint_states，更新 internal 初始關節角"""
        new_positions = [0.0] * len(self.fr3_chain.links)
        link_names = [link.name for link in self.fr3_chain.links]

        for idx, jn in enumerate(msg.name):
            if jn in self.joint_to_link:
                ln = self.joint_to_link[jn]
                try:
                    link_idx = link_names.index(ln)
                    new_positions[link_idx] = msg.position[idx]
                except ValueError:
                    # 只要對應錯了就記錄警告，避免無限例外
                    self.get_logger().warning(f"Link '{ln}' not in chain.")
        self.current_joint_state = new_positions

    def target_pose_callback(self, msg: PoseStamped):
        """接收目標位姿，計算 IK，並發布 /fr3/joint_command"""
        p = msg.pose.position
        o = msg.pose.orientation

        # 組成 ikpy 需要的輸入：位置 + 四元數
        target_position = [p.x, p.y, p.z]
        target_orientation = [o.x, o.y, o.z, o.w]

        # 利用上一次的關節狀態加速收斂
        solution = self.fr3_chain.inverse_kinematics(
            target_position,
            target_orientation,
            initial_position=self.current_joint_state
        )

        # 建立 ROS2 JointState 訊息
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = list(self.joint_to_link.keys())
        # chain.links[0] 通常是 base，不算在 7 軸解裡
        cmd.position = [
            float(solution[i])
            for i in range(1, 8)
        ]

        self.pub.publish(cmd)
        self.get_logger().debug(f"Published IK solution: {cmd.position}")

def main():
    rclpy.init()
    node = FR3IKSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
