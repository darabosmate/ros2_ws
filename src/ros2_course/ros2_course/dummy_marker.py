import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import math


class DummyMarker(Node):
    def __init__(self, position):
        super().__init__('dummy_marker_publisher')
        self.position = position
        self.publisher_ = self.create_publisher(Marker, 'dummy_target_marker', 10)

        self.measured_cp = None
        self.subscription = self.create_subscription(
            PoseStamped,
            '/PSM1/measured_cp',
            self.cb_measured_cp,
            10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        i = 0

    # Callback for pose
    def cb_measured_cp(self, msg):
        self.measured_cp = msg
        #print(self.measured_cp)

    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = 'PSM1_psm_base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "dvrk_viz"
        marker.id = self.i
        marker.type = Marker.SPHERE
        marker.action = Marker.MODIFY
        marker.pose.position.x = self.position[0]
        marker.pose.position.y = self.position[1]
        marker.pose.position.z = self.position[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.008
        marker.scale.y = 0.008
        marker.scale.z = 0.008
        marker.color.a = 1.0 # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.publisher_.publish(marker)
        #self.i += 1

    def is_grabbed(self):
        if self.measured_cp is None:
            return False

        distance = math.sqrt((self.position[0] - self.measured_cp.pose.position.x)**2 +
                                (self.position[1] - self.measured_cp.pose.position.y)**2 +
                             (self.position[2] - self.measured_cp.pose.position.z)**2)

        threshold = 0.01  # Define your own threshold here
        return distance < threshold

def main(args=None):
    rclpy.init(args=args)
    marker_publisher = DummyMarker([-0.05, 0.08, -0.14])
    #rclpy.spin(marker_publisher)

    while rclpy.ok():
        if marker_publisher.measured_cp is not None:
            if marker_publisher.is_grabbed():
                marker_publisher.position = [marker_publisher.measured_cp.pose.position.x,
                                            marker_publisher.measured_cp.pose.position.y,
                                            marker_publisher.measured_cp.pose.position.z]
        rclpy.spin_once(marker_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
