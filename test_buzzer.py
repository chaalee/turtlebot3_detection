#!/usr/bin/env python3
"""
Test buzzer functionality on OpenCR board
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
import time

class BuzzerTest(Node):
    def __init__(self):
        super().__init__('buzzer_test')
        self.sound_pub = self.create_publisher(UInt8, '/sound', 10)
        
        self.get_logger().info('Buzzer Test Node Started')
        self.get_logger().info('Playing test sounds...')
        
        # Test each note
        self.test_all_notes()
    
    def play_note(self, note, duration=0.5):
        """Play a single note"""
        sound_msg = UInt8()
        sound_msg.data = note
        self.sound_pub.publish(sound_msg)
        self.get_logger().info(f'Playing note: {note}')
        time.sleep(duration)
        
        # Stop sound
        sound_msg.data = 0
        self.sound_pub.publish(sound_msg)
        time.sleep(0.2)
    
    def test_all_notes(self):
        """Test all available notes"""
        notes = {
            0: 'OFF',
            1: 'C',
            2: 'D',
            3: 'E',
            4: 'F',
            5: 'G',
            6: 'A',
            7: 'B'
        }
        
        for note, name in notes.items():
            if note == 0:
                continue
            self.get_logger().info(f'Testing note {note} ({name})')
            self.play_note(note, 0.3)
        
        self.get_logger().info('Test complete!')
        
        # Play a melody
        self.get_logger().info('Playing melody...')
        melody = [1, 3, 5, 3, 1]  # C-E-G-E-C
        for note in melody:
            self.play_note(note, 0.3)

def main(args=None):
    rclpy.init(args=args)
    node = BuzzerTest()
    time.sleep(5)  # Give time for tests
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
