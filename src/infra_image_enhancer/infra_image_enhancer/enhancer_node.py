import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber

class InfraImageEnhancer(Node):
    def __init__(self):
        super().__init__('infra_image_enhancer')
        self.bridge = CvBridge()
        self.sub1 = Subscriber(self, Image, '/camera/infra1/image_rect_raw')
        self.sub2 = Subscriber(self, Image, '/camera/infra2/image_raw')
        self.sync = ApproximateTimeSynchronizer([self.sub1, self.sub2], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.callback)
        self.pub1 = self.create_publisher(Image, '/proc/infra1/image_enhanced', 10)
        self.pub2 = self.create_publisher(Image, '/proc/infra2/image_enhanced', 10)

    def enhance_image(self, cv_img):
        # CLAHE
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        img_clahe = clahe.apply(cv_img)
        # ノイズ除去（ガウシアン）
        img_denoised = cv2.GaussianBlur(img_clahe, (3,3), 0.5)
        # 正規化
        img_norm = cv2.normalize(img_denoised, None, 0, 255, cv2.NORM_MINMAX)
        img_norm = img_norm.astype(np.uint8)
        return img_norm

    def callback(self, msg1, msg2):
        try:
            cv_img1 = self.bridge.imgmsg_to_cv2(msg1, desired_encoding='mono8')
            cv_img2 = self.bridge.imgmsg_to_cv2(msg2, desired_encoding='mono8')
            enh1 = self.enhance_image(cv_img1)
            enh2 = self.enhance_image(cv_img2)
            out1 = self.bridge.cv2_to_imgmsg(enh1, encoding='mono8')
            out2 = self.bridge.cv2_to_imgmsg(enh2, encoding='mono8')
            out1.header = msg1.header
            out2.header = msg2.header
            self.pub1.publish(out1)
            self.pub2.publish(out2)
        except Exception as e:
            self.get_logger().error(f'画像処理エラー: {e}')

def main():
    rclpy.init()
    node = InfraImageEnhancer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
