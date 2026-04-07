from setuptools import setup
import os
from glob import glob

package_name = 'yolov8_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yks',
    maintainer_email='yks09011@naver.com',
    description='YOLOv8 object detection ROS2 package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov8_dis = yolov8_ros.yolov8_dis:main'
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  # launch 파일도 설치 (필요하면)
    ],
)

