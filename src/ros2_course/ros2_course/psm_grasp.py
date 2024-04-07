import math
from matplotlib import pyplot as plt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import time
import numpy as np
from visualization_msgs.msg import Marker

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

        # self.marker = None
        # self.subscription = self.create_subscription(
        #     Marker, 'dummy_target_marker'
        #     self.marker,
        #     10)

    # Callback for pose
    def cb_measured_cp(self, msg):
        self.measured_cp = msg
        #print(self.measured_cp)

    # Callback for jaw
    def cb_measured_jaw(self, msg):
        self.measured_jaw = msg
        print(self.measured_jaw)

    def cb_marker(self, msg):
        self.marker = msg

    # def grasp_marker(self, v, omega,  v, dt):
    #     loop_rate = self.create_rate(100, self.get_clock())

    #     while(self.marker is None and rclpy.ok()):


    def move_tcp_to(self, target, v, dt):
        # Tervezett trajektória létrehozása
        num_steps = int(np.linalg.norm(target - self.current_position) / (v * dt))  # Lépésszám
        traj_x = np.linspace(self.current_position[0], target[0], num_steps)
        traj_y = np.linspace(self.current_position[1], target[1], num_steps)
        traj_z = np.linspace(self.current_position[2], target[2], num_steps)
        
        # Plotolás
        plt.plot(traj_x, label='X')
        plt.plot(traj_y, label='Y')
        plt.plot(traj_z, label='Z')
        plt.xlabel('Idő')
        plt.ylabel('Pozíció')
        plt.title('TCP mozgása lineáris trajektórián')
        plt.legend()
        plt.grid(True)
        plt.show()
        
        # Mozgatás és üzenet küldése
        loop_rate = self.create_rate(100)  # Hz
        for i in range(num_steps):
            # Mozgatás
            self.current_position = np.array([traj_x[i], traj_y[i], traj_z[i]])
            # Üzenet létrehozása és publikálása
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = self.current_position[0]
            msg.pose.position.y = self.current_position[1]
            msg.pose.position.z = self.current_position[2]
            self.servo_cp_pub.publish(msg)
            # Várakozás a következő lépésre
            loop_rate.sleep()
        
        self.get_logger().info('TCP moved to target position.')

    # def move_tcp_to(self, target, v, dt):
    #        # Wait for position to be received
    #     loop_rate = self.create_rate(100, self.get_clock())  # Hz
    #     while self.measured_jaw is None and rclpy.ok():
    #         self.get_logger().info('Waiting for pose...')
    #         rclpy.spin_once(self)


    #     # Copy msg
    #     msg = self.measured_jaw

    #     distance = abs(target - self.measured_jaw.position[0])
    #     T = distance / omega
    #     N = int(math.floor(T/dt))
    #     tr_jaw = np.linspace(self.measured_jaw.position[0], target, N)

    #     # publish trajectory
    #     for i in tr_jaw:


        # Publish msg
        #msg.header.stamp = self.get_clock().now().to_msg()
        #msg.pose.position.x = target[0]
        #msg.pose.position.y = target[1]
        #msg.pose.position.z = target[2]
        #self.servo_cp_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    psm = PSM()

    #Reset the arm
    psm.move_tcp_to([0.0, 0.0, -0.12], 0.01, 0.01)
    #psm.move_jaw_to(0.0, 0.1, 0.01)

    psm.move_tcp_to([0.0, 0.05, -0.12], 0.01, 0.01)
    #psm.move_jaw_to()



    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    psm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
