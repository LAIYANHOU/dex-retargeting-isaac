#!/usr/bin/env python3
# keyboard_pose_writer.py
# ROS2 node to WRITE PoseStamped to a file based on keyboard inputs.

import sys
import termios
import tty
import threading
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

# === [檔案通訊] 定義目標檔案路徑 ===
TARGET_FILE_PATH = "/tmp/ik_target.txt"

# ... (所有按鍵定義和初始值保持不變) ...
KEY_UP = '\x1b[A'; KEY_DOWN = '\x1b[B'; KEY_RIGHT = '\x1b[C'; KEY_LEFT = '\x1b[D'
KEY_Z_UP = 'r'; KEY_Z_DOWN = 'f'
KEY_PITCH_UP = 'i'; KEY_PITCH_DOWN = 'k'; KEY_YAW_LEFT = 'j'; KEY_YAW_RIGHT = 'l'
KEY_ROLL_LEFT = 'u'; KEY_ROLL_RIGHT = 'o'; KEY_QUIT = 'q'
STEP_POS = 0.01; STEP_ANG = 5.0

class KeyboardPoseWriter(Node):
    def __init__(self):
        super().__init__('keyboard_pose_writer')
        self.x, self.y, self.z = 0.5, 0.0, 0.5
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.running = True
        self.get_logger().info('Keyboard Pose Writer started. Press keys to write to file. q to quit.')
        thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        thread.start()

    def keyboard_loop(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self.running and rclpy.ok():
                key = sys.stdin.read(1)
                if key == '\x1b': key += sys.stdin.read(2)
                
                # ... (所有鍵盤處理邏輯保持不變) ...
                if key == KEY_QUIT: self.running = False; break
                elif key == KEY_UP: self.y += STEP_POS
                elif key == KEY_DOWN: self.y -= STEP_POS
                elif key == KEY_RIGHT: self.x += STEP_POS
                elif key == KEY_LEFT: self.x -= STEP_POS
                elif key == KEY_Z_UP: self.z += STEP_POS
                elif key == KEY_Z_DOWN: self.z -= STEP_POS
                elif key == KEY_PITCH_UP: self.pitch += STEP_ANG
                elif key == KEY_PITCH_DOWN: self.pitch -= STEP_ANG
                elif key == KEY_YAW_LEFT: self.yaw += STEP_ANG
                elif key == KEY_YAW_RIGHT: self.yaw -= STEP_ANG
                elif key == KEY_ROLL_LEFT: self.roll += STEP_ANG
                elif key == KEY_ROLL_RIGHT: self.roll -= STEP_ANG
                else: continue

                self.roll = (self.roll + 180) % 360 - 180
                self.pitch = (self.pitch + 180) % 360 - 180
                self.yaw = (self.yaw + 180) % 360 - 180

                quat = R.from_euler('xyz', [self.roll, self.pitch, self.yaw], degrees=True).as_quat()

                # === [檔案通訊] 核心修改：將姿態寫入檔案 ===
                # 格式: x y z qx qy qz qw
                pose_str = f"{self.x} {self.y} {self.z} {quat[0]} {quat[1]} {quat[2]} {quat[3]}"
                try:
                    with open(TARGET_FILE_PATH, 'w') as f:
                        f.write(pose_str)
                except IOError as e:
                    self.get_logger().error(f"Could not write to target file: {e}")

                self.get_logger().info(f"Wrote to file: pos=({self.x:.2f},{self.y:.2f},{self.z:.2f})")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def destroy_node(self):
        self.running = False
        super().destroy_node()

def main():
    rclpy.init()
    node = KeyboardPoseWriter()
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
