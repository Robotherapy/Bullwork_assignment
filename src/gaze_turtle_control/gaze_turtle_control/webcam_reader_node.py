import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamReaderNode(Node):
    def __init__(self):
        super().__init__('webcam_reader_node')
        
        # Declare and get frame rate parameter
        self.declare_parameter('frame_rate', 15.0)
        frame_rate = self.get_parameter('frame_rate').value
        self.timer_period = 1.0 / frame_rate

        # Set up publisher
        self.publisher_ = self.create_publisher(Image, 'webcam/image_raw', 10)

        # Set up timer
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Initialize webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open webcam.')

        # OpenCV <-> ROS image converter
        self.bridge = CvBridge()

        self.get_logger().info('WebcamReaderNode started.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to read frame from webcam.')
            return

        # Convert OpenCV image to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(ros_image)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
