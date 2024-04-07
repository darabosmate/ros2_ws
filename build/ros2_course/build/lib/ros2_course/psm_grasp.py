import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
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

    # Callback for pose
    def cb_measured_cp(self, msg):
        self.measured_cp = msg
        #print(self.measured_cp)

    # Callback for jaw
    def cb_measured_jaw(self, msg):
        self.measured_jaw = msg
        print(self.measured_jaw)

    def move_tcp_to(self, target, v, dt):
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock())  # Hz
        while self.measured_cp is None and rclpy.ok():
            self.get_logger().info('Waiting for pose...')
            rclpy.spin_once(self)


        # Copy msg
        msg = self.measured_cp

        # Publish msg
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = target[0]
        msg.pose.position.y = target[1]
        msg.pose.position.z = target[2]
        self.servo_cp_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    psm = PSM()
    psm.move_tcp_to([0.0, 0.05, -0.12], 0.01, 0.01)



    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    psm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
