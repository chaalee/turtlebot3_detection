from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mooham',
    maintainer_email='mooham@student.chula.ac.th',
    description='YOLO person detection with distance-proportional buzzer alert',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = turtlebot3_detection.detection_node:main',
            'test_buzzer = turtlebot3_detection.test_buzzer:main',
            'test_camera = turtlebot3_detection.test_camera:main',
        ],
    },
)
EOF
