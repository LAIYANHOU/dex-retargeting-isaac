#!/usr/bin/env python3
# ik_solver_node.py
# ROS2 node combining IK and debug for orientation mapping

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from ikpy.chain import Chain
from scipy.spatial.transform import Rotation as R


class FR3IKSolverNode(Node):
    def __init__(self):
        super().__init__('fr3_ik_solver')

        # 1. Load URDF and build full kinematic chain
        urdf_path = os.getenv(
            'FR3_URDF',
            '/home/lai/isaacsim/dex-retargeting/src/franka_description/robots/fr3/fr3_with_leaphand.urdf'
        )
        self.full_chain = Chain.from_urdf_file(
            urdf_path,
            base_elements=['base'],
        )
        self.get_logger().info(f"Loaded URDF chain with {len(self.full_chain.links)} links.")

        # 2. Define joint names and build IK chain mask
        self.joint_names = [f'fr3_joint{i}' for i in range(1, 8)]
        mask = [(link.name in self.joint_names) for link in self.full_chain.links]
        self.ik_chain = Chain(
            links=self.full_chain.links,
            active_links_mask=mask
        )
        self.get_logger().info(f"IK on joints: {self.joint_names}")

        # 3. Publishers and Subscribers
        self.joint_pub = self.create_publisher(JointState, '/fr3/joint_command', 10)
        self.create_subscription(JointState, '/fr3/joint_states', self.cb_joint, 10)
        self.create_subscription(PoseStamped, '/target_pose', self.cb_target, 10)

        # 4. State storage
        self.current_q = [0.0] * len(self.full_chain.links)
        self.latest_target = None  # tuple (pos, ori)

        self.get_logger().info('FR3 IK Solver Node started with orientation debug.')

    def cb_joint(self, msg: JointState):
        # Update current joint state vector for IK initial guess
        q = [0.0] * len(self.full_chain.links)
        for idx, link in enumerate(self.full_chain.links):
            if link.name in self.joint_names:
                try:
                    j = msg.name.index(link.name)
                    q[idx] = msg.position[j]
                except ValueError:
                    pass
        self.current_q = q

        # Compute and log current FK orientation if target exists
        if self.latest_target is not None:
            T_cur = self.full_chain.forward_kinematics(self.current_q)
            quat_cur = R.from_matrix(T_cur[:3, :3]).as_quat()
            tgt_ori = self.latest_target[1]
            # Compute orientation error
            err_rot = R.from_quat(tgt_ori).inv() * R.from_quat(quat_cur)
            angle_err = err_rot.magnitude() * 180.0 / 3.1415926
            self.get_logger().debug(
                f"[FK DEBUG] current_quat={quat_cur.tolist()}, angle_error={angle_err:.2f}°"
            )

    def cb_target(self, msg: PoseStamped):
        # Extract target position and orientation
        tgt_pos = [msg.pose.position.x,
                   msg.pose.position.y,
                   msg.pose.position.z]
        tgt_ori_xyzw = [msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w]
        self.latest_target = (tgt_pos, tgt_ori_xyzw)

        self.get_logger().info(
            f"[IK DEBUG] Received target_pos={tgt_pos}, target_ori_xyzw={tgt_ori_xyzw}"
        )

        # Reorder quaternion to solver format [w, x, y, z]
        solver_ori = [tgt_ori_xyzw[3], tgt_ori_xyzw[0], tgt_ori_xyzw[1], tgt_ori_xyzw[2]]
        self.get_logger().debug(f"[IK DEBUG] Solver target_ori={solver_ori} (wxyz)")

        # Compute IK using correct quaternion order
        # sol = self.ik_chain.inverse_kinematics(
        #     target_position=tgt_pos,
        #     target_orientation=solver_ori,
        #     initial_position=self.current_q
        # )

        from scipy.spatial.transform import Rotation as R

        # 原始 ROS 四元數 [x,y,z,w]
        quat_xyzw = tgt_ori_xyzw  
        # 轉成 3×3 旋轉矩陣
        rot_mat = R.from_quat(quat_xyzw).as_matrix()

        sol = self.ik_chain.inverse_kinematics(
            target_position= tgt_pos,
            target_orientation= rot_mat,
            initial_position= self.current_q
        )



        # FK on solution to debug orientation
        T_sol = self.full_chain.forward_kinematics(sol.tolist())
        quat_sol = R.from_matrix(T_sol[:3, :3]).as_quat().tolist()  # [x,y,z,w]
        self.get_logger().info(f"[IK DEBUG] Solution_quat=[x,y,z,w]={quat_sol}")

        # Publish IK command
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        names_out = []
        pos_out = []
        for idx, link in enumerate(self.full_chain.links):
            if link.name in self.joint_names:
                names_out.append(link.name)
                pos_out.append(float(sol[idx]))
        cmd.name = names_out
        cmd.position = pos_out
        self.joint_pub.publish(cmd)
        self.get_logger().debug(f"Published joint_command: {list(zip(names_out, pos_out))}")
        # cmd = JointState()
        # cmd.header.stamp = self.get_clock().now().to_msg()
        # names_out = []
        # pos_out = []
        # for idx, link in enumerate(self.full_chain.links):
        #     if link.name in self.joint_names:
        #         names_out.append(link.name)
        #         pos_out.append(float(sol[idx]))
        # cmd.name = names_out
        # cmd.position = pos_out
        # self.joint_pub.publish(cmd)
        # self.get_logger().debug(f"Published joint_command: {list(zip(names_out, pos_out))}")


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
