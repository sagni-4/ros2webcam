#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from builtin_interfaces.msg import Time as RosTime

class WebcamDriver(Node):
    def __init__(self):
        super().__init__('webcam_driver')

        # Best Effort QoS
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Publisher
        self.publisher_ = self.create_publisher(Image, '/webcam/image_raw', qos_profile)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error('Could not open webcam')
            return

        # Timer to read and publish image
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to capture frame')
            return

        # Get current ROS time in seconds
        now = self.get_clock().now()
        time_sec = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9

        # Overlay time on image
        time_text = f"Time: {time_sec:.2f}"
        cv2.putText(frame, time_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (255, 255, 0), 2, cv2.LINE_AA)

        # Convert and publish
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = now.to_msg()
        self.publisher_.publish(img_msg)

        self.get_logger().info('publishing video frame')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
