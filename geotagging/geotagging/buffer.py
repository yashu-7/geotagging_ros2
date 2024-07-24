import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time

class Buffer(Node):

    def __init__(self):
        super().__init__('buffer')
        self.subscription1 = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription2 = self.create_subscription(
            Int32,
            'trigger_pub',
            self.trigger_callback,
            10)
        
        self.bridge = CvBridge()
        self.trigger_count = 0
        self.trigger_active = False
        self.last_trigger_time = time.time()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return
        
        frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2BGRA)
        frame_resized = cv2.resize(frame, (800, 580))

        now = time.time()
        if self.trigger_active and (now - self.last_trigger_time) < 1:
            cv2.putText(frame_resized, "SAVED", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:
            self.trigger_active = False
        
        trigger_count_text = f"Trigger Count: {self.trigger_count}"
        cv2.putText(frame_resized, trigger_count_text, (240, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 100, 255), 2)

        try:
            with open('/dev/fb0', 'wb') as fb:
                fb.write(frame_resized.tobytes())
        except Exception as e:
            self.get_logger().error(f'Failed to write to framebuffer device: {e}')

    def trigger_callback(self, msg):
        if msg.data == 1:
            self.trigger_count += 1
            self.trigger_active = True
            self.last_trigger_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = Buffer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()