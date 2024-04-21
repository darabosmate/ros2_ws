import math

import geometry_msgs.msg
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import time
import numpy as np

class PSM(Node):

    def __init__(self):
        super().__init__('psm_grasp')
        self.servo_cp_pub = self.create_publisher(
            PoseStamped,
            '/PSM1/servo_cp',
            10)

        self.servo_jaw_pub = self.create_publisher(
            JointState,
            '/PSM1/jaw/servo_jp',
            10)

        self.measured_cp = None
        self.subscription = self.create_subscription(
            PoseStamped,
            '/PSM1/measured_cp',
            self.cb_measured_cp,
            10)

        self.measured_jaw = None
        self.subscription = self.create_subscription(
            JointState,
            '/PSM1/jaw/measured_js',
            self.cb_measured_jaw,
            10)

        self.marker = None
        self.subscription = self.create_subscription(
            Marker,
            '/dummy_target_marker',
            self.cb_marker,
            10)

    # Callback for pose
    def cb_measured_cp(self, msg):
        self.measured_cp = msg
        #print(self.measured_cp)

    # Callback for jaw
    def cb_measured_jaw(self, msg):
        self.measured_jaw = msg
        #print(self.measured_jaw)

    # Callback for marker
    def cb_marker(self, msg):
        self.marker = msg
        # print(self.marker)

    def move_tcp_to(self, target, v, dt):
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock())  # Hz
        while self.measured_cp is None and rclpy.ok():
            self.get_logger().info('Waiting for pose...')
            rclpy.spin_once(self)


        # Copy msg
        msg = self.measured_cp

        # Create linear trajectory
        measured_cp_np = np.array([self.measured_cp.pose.position.x,
                                   self.measured_cp.pose.position.y,
                                   self.measured_cp.pose.position.z])
        target_np = np.array(target)

        distance = np.linalg.norm(target_np - measured_cp_np)
        T = distance / v
        N = int(math.floor(T / dt))

        tr_x = np.linspace(self.measured_cp.pose.position.x, target[0], N)
        tr_y = np.linspace(self.measured_cp.pose.position.y, target[1], N)
        tr_z = np.linspace(self.measured_cp.pose.position.z, target[2], N)


        # Publish trajectory
        loop_rate = self.create_rate(1.0 / dt, self.get_clock())  # Hz
        print("Starting trajectory...")
        for i in range(N):
            if not rclpy.ok():
                break
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = tr_x[i]
            msg.pose.position.y = tr_y[i]
            msg.pose.position.z = tr_z[i]
            self.servo_cp_pub.publish(msg)
            rclpy.spin_once(self)
        print("Trajectory finsihed.")


    def move_jaw_to(self, target, omega, dt):
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock())  # Hz
        while self.measured_jaw is None and rclpy.ok():
            self.get_logger().info('Waiting for jaw...')
            rclpy.spin_once(self)


        # Copy msg
        msg = self.measured_jaw

        # Create linear trajectory
        distance = abs(target - self.measured_jaw.position[0])
        T = distance / omega
        N = int(math.floor(T / dt))
        tr_jaw = np.linspace(self.measured_jaw.position[0], target, N)

        # Publish trajectory
        loop_rate = self.create_rate(1.0 / dt, self.get_clock())  # Hz
        print("Starting jaw trajectory...")
        for i in range(N):
            if not rclpy.ok():
                break
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = [tr_jaw[i]]
            self.servo_jaw_pub.publish(msg)
            rclpy.spin_once(self)
        print("Jaw trajectory finsihed.")


    def grasp_marker(self, v, omega, dt):
        # Wait for marker position to be received
        loop_rate = self.create_rate(100, self.get_clock())  # Hz
        while self.marker is None and rclpy.ok():
            self.get_logger().info('Waiting for marker...')
            rclpy.spin_once(self)
        print(self.marker)

        # Open jaws
        self.move_jaw_to(0.8, omega, dt)

        # Nav to marker
        target = [self.marker.pose.position.x,
                  self.marker.pose.position.y,
                  self.marker.pose.position.z + 0.008]
        self.move_tcp_to(target, v, dt)

        # Close jaws
        self.move_jaw_to(0.0, omega, dt)


def main(args=None):
    rclpy.init(args=args)
    psm = PSM()

    # Reset the arm
    psm.move_tcp_to([0.0, 0.0, -0.12], 0.01, 0.01)
    psm.move_jaw_to(0.0, 0.1, 0.01)

    # Trajectory following
    #psm.move_tcp_to([0.0, 0.05, -0.12], 0.01, 0.01)
    #psm.move_jaw_to(0.8, 0.1, 0.01)
    psm.grasp_marker(0.01, 0.05, 0.01)



    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    psm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
