import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__('turtle_controller_node')

        # Parameters
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('dead_zone', 0.1)

        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.dead_zone = self.get_parameter('dead_zone').value

        # Subscriber and Publisher
        self.subscription = self.create_subscription(
            Vector3,
            'gaze_vector',
            self.gaze_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        self.get_logger().info("TurtleControllerNode started.")

    def gaze_callback(self, msg):
        linear = 0.0
        angular = 0.0

        # Only respond if outside dead zone
        if abs(msg.y) > self.dead_zone:
            linear = max(-self.max_linear, min(self.max_linear, msg.y))
        if abs(msg.x) > self.dead_zone:
            angular = max(-self.max_angular, min(self.max_angular, msg.x))

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
