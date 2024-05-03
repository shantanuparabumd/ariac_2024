import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import random

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.i1,self.i2,self.i3,self.i4 = 0,0,0,0
        self.bridge = CvBridge()
        self.image_sub_1 = self.create_subscription(
            Image, '/ariac/sensors/left_bins_camera/rgb_image', self.image1_callback, 10)
        self.image_sub_2 = self.create_subscription(
            Image, '/ariac/sensors/right_bins_camera/rgb_image', self.image2_callback, 10)
        # self.image_sub_3 = self.create_subscription(
        #     Image, '/ariac/sensors/kts2_camera/rgb_image', self.image3_callback, 10)
        # self.image_sub_4 = self.create_subscription(
        #     Image, '/ariac/sensors/kts1_camera/rgb_image', self.image4_callback, 10)

    def image1_callback(self, msg):
        if self.i1==0:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(f'dataset/image_{random.randint(1,1000)}.jpg', cv_image)
            self.get_logger().info('Image saved!')
            self.i1=1
        if self.i1==1 and self.i2==1 and self.i3==1 and self.i4==1:
            self.destroy_node()
    
    def image2_callback(self, msg):
        if self.i2==0:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(f'dataset/image_{random.randint(1,1000)}.jpg', cv_image)
            self.get_logger().info('Image saved!')
            self.i2=1
        if self.i1==1 and self.i2==1 and self.i3==1 and self.i4==1:
            self.destroy_node()
        
        
    # def image3_callback(self, msg):
    #     if self.i3==0:
    #         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #         cv2.imwrite(f'tray/image_{random.randint(1,1000)}.jpg', cv_image)
    #         self.get_logger().info('Image saved!')
    #         self.i3=1
    #     if self.i1==1 and self.i2==1 and self.i3==1 and self.i4==1:
    #         self.destroy_node()
        
    # def image4_callback(self, msg):
    #     if self.i4==0:
    #         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #         cv2.imwrite(f'tray/image_{random.randint(1,1000)}.jpg', cv_image)
    #         self.get_logger().info('Image saved!')
    #         self.i4=1
    #     if self.i1==1 and self.i2==1 and self.i3==1 and self.i4==1:
    #         self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    try: 
        rclpy.spin(image_saver)
    except KeyboardInterrupt:  
        image_saver.destroy_node()
        rclpy.shutdown()
    finally:
        image_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
