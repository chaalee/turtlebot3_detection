#!/usr/bin/env python3
"""
Launch complete detection system: camera + YOLO + buzzer
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 1280,
                'image_height': 720,
                'pixel_format': 'yuyv',
                'camera_frame_id': 'camera_link',
                'framerate': 30.0,
            }]
        ),
        
        # YOLO detection node with buzzer
        Node(
            package='turtlebot3_detection',
            executable='detection_node',
            name='people_detection',
            output='screen',
            parameters=[{
                'safety_distance': 1.5,
                'warning_distance': 2.5,
                'confidence_threshold': 0.6,
                'focal_length': 450.5,  # Update after calibration
            }]
        )
    ])
EOF
