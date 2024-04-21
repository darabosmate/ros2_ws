from asyncore import loop
import math

from networkx import omega

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import time
import numpy as np

class UR(Node):

    def __init__(self):
        super().__init__('ur_controller')
        self.joint_states_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10)

        self.joint_state = None
        self.subscription_js = self.create_subscription(
            JointState,
            '/set_joint_states',
            self.cb_joint_states,
            10)

        self.desc = None
        self.chain = None
        self.subscription_desc = self.create_subscription(
            String,
            '/robot_description_latch',
            self.cb_desc,
            10)

    def cb_joint_states(self, msg):
        self.joint_state = msg
        print(self.joint_state)

    def cb_desc(self, msg):
        self.desc = msg.data
        print(self.desc)



    def to_configuration(self, q):
        loop_rate = self.create_rate(100, self.get_clock())
        while self.joint_state is None and rclpy.ok():
            self.get_logger().info('Waiting for joint_states...')
            rclpy.spin_once(self)

        joint_msg = self.joint_state
        joint_msg.position = q
        self.joint_states_pub.publish(joint_msg)

    def to_position(self, r_des):
        loop_rate = self.create_rate(100, self.get_clock())
        while self.joint_state is None and rclpy.ok():
            self.get_logger().info('Waiting for joint_states...')
            rclpy.spin_once(self)

        delta_r = np.array([50000.0, 50000.0, 50000.0])
        k1 = 0.005
        omega = np.array([0.0, 0.0, 0.1])
        while np.linalg.norm(delta_r) > 0.05 and rclpy.ok():
            r_curr = self.calc_position().pos
            delta_r = np.array(r_des) - np.array(r_curr)
            k1_delta_r = k1 * delta_r

        
def main(args=None):
    rclpy.init(args=args)
    ur = UR()

    ur.to_configuration([0, 0, 0, 0, 0, 0])
    rclpy.spin(ur)

    ur.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

