#!/usr/bin/env python3
"""
Test camera functionality - displays camera feed in RViz2
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraTest(Node):
    def __init__(self):
        super().__init__('camera_test')
        
        self.bridge = CvBridge()
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publish test image
        self.image_pub = self.create_publisher(Image, '/camera/test_image', 10)
        
        self.frame_count = 0
        self.get_logger().info('Camera Test Node Started')
        self.get_logger().info('View in RViz2: Add /camera/test_image topic')
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            self.frame_count += 1
            
            # Add text overlay
            cv2.putText(cv_image, f'Camera Test - Frame {self.frame_count}',
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            cv2.putText(cv_image, f'Resolution: {cv_image.shape[1]}x{cv_image.shape[0]}',
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Convert back to ROS Image and publish
            test_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.image_pub.publish(test_msg)
            
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Receiving frames: {self.frame_count}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
