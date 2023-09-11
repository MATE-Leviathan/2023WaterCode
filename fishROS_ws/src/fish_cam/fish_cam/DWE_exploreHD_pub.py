import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


VIDEO_DEVICE = 4 # /dev/videoX


class ExploreHDPub(Node):

    
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(Image, 'Image', 10)
        self.get_logger().info(self.get_node_names_and_namespaces())
        VIDEO_DEVICE = self.get_parameter('video_device_id').get_parameter_value()
        print(f"Video device paramter is {VIDEO_DEVICE}")
        self.cap = cv2.VideoCapture(VIDEO_DEVICE)
        self.bridge = CvBridge()

        if not self.cap.isOpened():
            print("Cannot open camera")
            exit()


    def publish_image(self):

        # essetnially while True but is ros shutdown safe
        while rclpy.ok():
            # Capture frame-by-frame
            ret, frame = self.cap.read()
            # if frame is read correctly ret is True
            if not ret:
                self.get_logger().error("Unable to read frame from camera")
                break
            
            # publishes the image converted to a ros message to the topic 'Image'
            self.publisher.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = ExploreHDPub()
    minimal_publisher.publish_image()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
   main()