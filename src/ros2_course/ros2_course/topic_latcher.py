import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import importlib

class TopicLatcher(Node):

    def __init__(self):
        super().__init__('topic_latcher')

        self.msg = None
        # Retrieve the topic type from the parameter server
        self.declare_parameter('topic_type', 'std_msgs/msg/String')
        topic_type = self.get_parameter('topic_type').get_parameter_value().string_value

        self.declare_parameter('hz', 1.0)
        hz = self.get_parameter('hz').get_parameter_value().double_value

        print("init start")

        try:
            # Dynamically import the module for the topic type
            topic_module = importlib.import_module('.'.join(topic_type.split('/')[:-1]))
            topic_class = getattr(topic_module, topic_type.split('/')[-1])

        except (ImportError, AttributeError):
            #TODO uncomment self.get_logger().error(f"Failed to import topic type: {topic_type_name}")
            print("err")
            return

        # Subscribers
        self.subscription = self.create_subscription(
            eval(topic_type.split('/')[-1]),
            '/input',
            self.cb_input,
            10)

        # Publishers
        self.pub_latch = self.create_publisher(
            eval(topic_type.split('/')[-1]),
            '/latch',
            10)

        # Set latch frequency and timer
        timer_period = 1.0 / hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        print("init done")




    # Callback for input topic
    def cb_input(self, msg):
        self.msg = msg
        print(msg)


    # Callback to latch message
    def timer_callback(self):
        print('timer')
        if not self.msg is None:
            self.pub_latch.publish(self.msg)



def main(args=None):
    rclpy.init(args=args)
    tl = TopicLatcher()
    rclpy.spin(tl)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
