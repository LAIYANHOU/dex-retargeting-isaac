#!/usr/bin/env python3
# keyboard_pose_publisher.py
# ROS2 node to PUBLISH Pose messages to the /target_pos topic based on keyboard inputs.

import sys
import termios
import tty
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped  # Import the Pose message type
from scipy.spatial.transform import Rotation as R

# Key definitions remain the same
KEY_UP = '\x1b[A'; KEY_DOWN = '\x1b[B'; KEY_RIGHT = '\x1b[C'; KEY_LEFT = '\x1b[D'
KEY_Z_UP = 'r'; KEY_Z_DOWN = 'f'
KEY_PITCH_UP = 'i'; KEY_PITCH_DOWN = 'k'; KEY_YAW_LEFT = 'j'; KEY_YAW_RIGHT = 'l'
KEY_ROLL_LEFT = 'u'; KEY_ROLL_RIGHT = 'o'; KEY_QUIT = 'q'
STEP_POS = 0.01; STEP_ANG = 5.0

class KeyboardPosePublisher(Node):
    def __init__(self):
        super().__init__('keyboard_pose_publisher')
        
        # === [ROS2 通訊] 核心修改：建立 Publisher ===
        self.publisher_ = self.create_publisher(PoseStamped, '/target_pose', 10)
        
        # Initial pose values remain the same
        self.x, self.y, self.z = 0.5, 0.0, 0.5
        self.roll, self.pitch, self.yaw = -180.0, 0.0, 0.0
        self.running = True
        
        self.get_logger().info('Keyboard Pose Publisher started. Press keys to publish pose. q to quit.')
        
        # Publish the initial pose once on startup
        self.publish_pose()
        
        # Start the keyboard listener thread
        thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        thread.start()

    def publish_pose(self):
        """Constructs and publishes the current pose as a Pose message."""
        # Create a Pose message
        msg = PoseStamped()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'

        # Populate the position
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z
        
        # Convert Euler angles to quaternion
        # scipy's as_quat() returns in (x, y, z, w) format, which matches geometry_msgs/Pose
        quat = R.from_euler('xyz', [self.roll, self.pitch, self.yaw], degrees=True).as_quat()
        
        # Populate the orientation
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        
        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: pos=({self.x:.2f}, {self.y:.2f}, {self.z:.2f}) rpy=({self.roll:.0f}, {self.pitch:.0f}, {self.yaw:.0f})")


    def keyboard_loop(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self.running and rclpy.ok():
                key = sys.stdin.read(1)
                # Handle arrow keys which are multi-byte
                if key == '\x1b': 
                    key += sys.stdin.read(2)
                
                # Keyboard handling logic remains the same
                if key == KEY_QUIT: 
                    self.running = False
                    break
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
                else: 
                    continue # Skip if it's not a recognized key

                # Normalize angles to keep them within -180 to 180 range
                self.roll = (self.roll + 180) % 360 - 180
                self.pitch = (self.pitch + 180) % 360 - 180
                self.yaw = (self.yaw + 180) % 360 - 180

                # === [ROS2 通訊] 核心修改：發布姿態訊息 ===
                self.publish_pose()

        finally:
            # Restore terminal settings
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def destroy_node(self):
        self.running = False
        super().destroy_node()

def main():
    rclpy.init()
    node = KeyboardPosePublisher()
    try:
        # We just need to keep the node alive while the keyboard thread runs.
        # spin() is a good way to do this and handle shutdown signals.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down Keyboard Pose Publisher.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()