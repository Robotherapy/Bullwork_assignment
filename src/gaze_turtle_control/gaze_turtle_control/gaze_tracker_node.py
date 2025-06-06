import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import cv2
import numpy as np

class GazeTrackerNode(Node):
    def __init__(self):
        super().__init__('gaze_tracker_node')

        # Parameters
        self.declare_parameter('gaze_sensitivity', 1.0)
        self.sensitivity = self.get_parameter('gaze_sensitivity').value

        # Subscribers and publishers
        self.subscription = self.create_subscription(
            Image,
            'webcam/image_raw',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(Vector3, 'gaze_vector', 10)

        self.bridge = CvBridge()
        self.get_logger().info('GazeTrackerNode started.')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        height, width, _ = frame.shape

        # Simulated gaze using face detection (replace with real eye tracking for better results)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        gaze_x = 0.0
        gaze_y = 0.0

        if len(faces) > 0:
            (x, y, w, h) = faces[0]
            face_cx = x + w / 2
            face_cy = y + h / 2

            # Normalize gaze based on face center (no inversion of Y for now)
            gaze_x = (face_cx - width / 2) / (width / 2)
            gaze_y = (face_cy - height / 2) / (height / 2)

            # Apply sensitivity
            gaze_x *= self.sensitivity
            gaze_y *= self.sensitivity

            # Clamp values between -1 and 1
            gaze_x = max(min(gaze_x, 1.0), -1.0)
            gaze_y = max(min(gaze_y, 1.0), -1.0)

            # Dead zone threshold
            dead_zone = 0.1
            if abs(gaze_x) < dead_zone:
                gaze_x = 0.0
            if abs(gaze_y) < dead_zone:
                gaze_y = 0.0

        # Prepare and publish gaze vector
        gaze_vector = Vector3()
        gaze_vector.x = gaze_x
        gaze_vector.y = gaze_y
        gaze_vector.z = 0.0

        self.publisher_.publish(gaze_vector)

        # Logging for debug
        self.get_logger().info(
            f"Face: {('Detected' if len(faces) > 0 else 'Not detected')}, "
            f"Gaze Vector => x: {gaze_x:.2f}, y: {gaze_y:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = GazeTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

