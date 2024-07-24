import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.camera_loc = 0  # Update this with the appropriate camera index if needed
        self.publisher_name = '/camera/image_raw'
        self.fps = 20

        self.cam = cv2.VideoCapture(self.camera_loc)
        if not self.cam.isOpened():
            self.get_logger().error('Failed to open camera!')
            return

        self.pub = self.create_publisher(Image, self.publisher_name, 1)
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)
        self.bridge = CvBridge()
        self.count = 0

    def timer_callback(self):
        ret, src = self.cam.read()
        if ret:
            # Resize the image to 720x480
            resized_img = cv2.resize(src, (720, 480))
            cv2.flip(resized_img, 1, resized_img)  # Flip if needed

            img_msg = self.bridge.cv2_to_imgmsg(resized_img, encoding='bgr8')
            self.pub.publish(img_msg)
            self.count = (self.count + 1) % 10
        else:
            self.get_logger().error('Failed to capture camera image')

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
