#!/usr/bin/env python3
"""
YOLO-based person detection with distance-proportional buzzer
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8, Float32
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import time

class PeopleDetectionNode(Node):
    def __init__(self):
        super().__init__('people_detection_node')
        
        # Initialize YOLO model
        self.get_logger().info('Loading YOLO model...')
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLO model loaded successfully')
        
        # ROS 2 setup
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sound_pub = self.create_publisher(UInt8, '/sound', 10)
        self.detection_pub = self.create_publisher(Image, '/detection/image', 10)
        self.distance_pub = self.create_publisher(Float32, '/detection/distance', 10)
        
        # Detection parameters (declare as ROS parameters for easy tuning)
        self.declare_parameter('safety_distance', 1.5)
        self.declare_parameter('warning_distance', 2.5)
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('focal_length', 450.5)
        
        self.safety_distance = self.get_parameter('safety_distance').value
        self.warning_distance = self.get_parameter('warning_distance').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.focal_length = self.get_parameter('focal_length').value
        
        # Buzzer timing control
        self.last_beep_time = 0
        
        # Performance tracking
        self.frame_count = 0
        self.start_time = time.time()
        
        self.get_logger().info('=== People Detection Node Started ===')
        self.get_logger().info(f'Safety distance: {self.safety_distance}m')
        self.get_logger().info(f'Warning distance: {self.warning_distance}m')
        self.get_logger().info(f'Focal length: {self.focal_length}')
    
    def estimate_distance(self, bbox_height):
        """Estimate distance based on bounding box height"""
        person_height_real = 1.7  # meters
        
        if bbox_height > 0:
            distance = (person_height_real * self.focal_length) / bbox_height
            return min(distance, 10.0)  # Cap at 10m
        return 10.0
    
    def play_proximity_alert(self, distance):
        """
        Distance-proportional beeping
        Beep frequency increases as person gets closer
        """
        current_time = time.time()
        
        # Determine beep interval and note based on distance
        if distance < 0.5:
            # Very close - continuous high-pitched tone
            interval = 0.1
            note = 7  # B (highest)
            duration = 0.1
        elif distance < 1.0:
            # Close - very fast beeps
            interval = 0.2
            note = 7  # B
            duration = 0.1
        elif distance < 1.5:
            # Safety zone boundary - fast beeps
            interval = 0.4
            note = 6  # A
            duration = 0.1
        elif distance < 2.0:
            # Warning zone - moderate beeps
            interval = 0.8
            note = 5  # G
            duration = 0.15
        elif distance < 2.5:
            # Caution zone - slow beeps
            interval = 1.5
            note = 4  # F
            duration = 0.15
        else:
            # Safe - no beep
            return
        
        # Only beep if enough time has passed
        if current_time - self.last_beep_time >= interval:
            self.play_sound(note, duration)
            self.last_beep_time = current_time
            
            # Log warning level
            if distance < 1.0:
                self.get_logger().warn(f'🚨 DANGER: Person at {distance:.2f}m')
            elif distance < 1.5:
                self.get_logger().warn(f'⚠️  STOP ZONE: Person at {distance:.2f}m')
            elif distance < 2.5:
                self.get_logger().info(f'⚡ WARNING: Person at {distance:.2f}m')
    
    def play_sound(self, note, duration=0.2):
        """Play a note on the buzzer"""
        sound_msg = UInt8()
        sound_msg.data = note
        self.sound_pub.publish(sound_msg)
        
        # Schedule turning off the sound
        if note != 0:
            self.create_timer(duration, lambda: self.stop_sound(), oneshot=True)
    
    def stop_sound(self):
        """Stop the buzzer"""
        sound_msg = UInt8()
        sound_msg.data = 0
        self.sound_pub.publish(sound_msg)
    
    def emergency_stop(self):
        """Send stop command to robot"""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO detection (class 0 = person)
            results = self.model(cv_image, classes=[0], verbose=False)
            
            # Process detections
            person_detected = False
            closest_distance = float('inf')
            annotated_image = cv_image.copy()
            
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    confidence = float(box.conf[0])
                    
                    if confidence > self.confidence_threshold:
                        person_detected = True
                        
                        # Get bounding box coordinates
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        bbox_height = y2 - y1
                        
                        # Calculate distance
                        distance = self.estimate_distance(bbox_height)
                        closest_distance = min(closest_distance, distance)
                        
                        # Choose color based on distance
                        if distance < self.safety_distance:
                            color = (0, 0, 255)  # Red - DANGER
                            status = "STOP"
                        elif distance < self.warning_distance:
                            color = (0, 165, 255)  # Orange - CAUTION
                            status = "SLOW"
                        else:
                            color = (0, 255, 0)  # Green - SAFE
                            status = "OK"
                        
                        # Draw bounding box
                        cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, 3)
                        
                        # Add labels
                        label = f'Person {confidence:.2f}'
                        distance_label = f'{distance:.2f}m - {status}'
                        
                        cv2.putText(annotated_image, label, (x1, y1-35),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        cv2.putText(annotated_image, distance_label, (x1, y1-10),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # Calculate FPS
            self.frame_count += 1
            elapsed = time.time() - self.start_time
            if elapsed > 1.0:
                fps = self.frame_count / elapsed
                self.frame_count = 0
                self.start_time = time.time()
                
                cv2.putText(annotated_image, f'FPS: {fps:.1f}', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # Handle closest person detection
            if person_detected:
                # Add status overlay
                status_text = f'Closest: {closest_distance:.2f}m'
                status_color = (0, 0, 255) if closest_distance < self.safety_distance else (0, 255, 0)
                cv2.putText(annotated_image, status_text, (10, 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
                
                # Play distance-proportional beep
                self.play_proximity_alert(closest_distance)
                
                # Publish distance
                distance_msg = Float32()
                distance_msg.data = closest_distance
                self.distance_pub.publish(distance_msg)
                
                # Emergency stop if too close
                if closest_distance < self.safety_distance:
                    self.emergency_stop()
            else:
                # No person detected - stop any ongoing sound
                self.stop_sound()
            
            # Publish annotated image
            detection_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            self.detection_pub.publish(detection_msg)
            
        except Exception as e:
            self.get_logger().error(f'Detection error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PeopleDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
