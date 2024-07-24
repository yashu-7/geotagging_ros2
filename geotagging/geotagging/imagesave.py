import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time
import datetime
from pathlib import Path

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription1 = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback1,
            10)
        self.navsat_sub = self.create_subscription(
            NavSatFix,
            '/ap/navsat/navsat0',
            self.navsat_callback,
            10)
        self.subscription2 = self.create_subscription(
            Int32,
            'trigger_pub',
            self.image_callback2,
            10)
        
        self.bridge = CvBridge()
        self.save = False
        self.images_base_path = '/home/rpi/images/'
        self.directory_number = self.get_next_directory_number()
        self.images_path = self.create_image_directory(self.directory_number)
        self.images_meta = os.path.join(self.images_path, 'meta.txt')
        self.count = self.image_count()
        self.altitude = 0.0
        self.latitude = 0.0
        self.longitude = 0.0

    def get_next_directory_number(self):
        max_number = 0
        base_path = Path(self.images_base_path)
        for entry in base_path.iterdir():
            if entry.is_dir() and entry.name.startswith("pictures-"):
                try:
                    dir_number = int(entry.name.split("-")[1])
                    if dir_number > max_number:
                        max_number = dir_number
                except ValueError:
                    continue
        return max_number + 1

    def create_image_directory(self, dir_number):
        new_directory = os.path.join(self.images_base_path, f'pictures-{dir_number}')
        os.makedirs(new_directory, exist_ok=True)
        return new_directory

    def image_count(self):
        image_count = 0
        for entry in os.scandir(self.images_path):
            if entry.is_file() and entry.name.endswith('.jpg'):
                image_count += 1
        return image_count

    def image_callback1(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return
        
        if self.save:
            image_name = f'_image_{self.directory_number}_{self.count}.jpg'
            image_path = os.path.join(self.images_path, image_name)
            cv2.imwrite(image_path, cv_image)

            with open(self.images_meta, 'a') as meta_file:
                meta_file.write(f'{image_name},{self.get_timestamp()}\n')
                meta_file.write(f'Altitude: {self.altitude} meters\n')
                meta_file.write(f'Latitude: {self.latitude} degrees\n')
                meta_file.write(f'Longitude: {self.longitude} degrees\n')
            
            self.count += 1
            self.save = False

    def image_callback2(self, msg):
        self.save = msg.data == 1

    def navsat_callback(self, msg):
        self.altitude = msg.altitude
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def get_timestamp(self):
        now = datetime.datetime.now()
        return now.strftime('%Y-%m-%d_%H-%M-%S')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
