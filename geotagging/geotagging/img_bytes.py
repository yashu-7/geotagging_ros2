import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class Encoding(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            1
        )
        self.bridge = CvBridge()
        self.count = 0

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        bytes_array = self.image_to_bytes(cv_image)
        print(f'Image size in bytes: {len(bytes_array)}')

    def image_to_bytes(self, image):
        success, encoded_image = cv2.imencode('.jpg', image)
        if not success:
            self.get_logger().error('Failed to encode image')
            return []

        return encoded_image.tobytes()

def main(args=None):
    rclpy.init(args=args)
    node = Encoding()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()