from setuptools import setup
import os
from glob import glob

package_name = 'cavepi_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name + '/launch', ['launch/all_in_one_launch.py']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xianyao',
    maintainer_email='xianyao.li@ufl.edu',
    description='ROS2 package for camera and cave line detection, developed by ROBOPI Lab.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = cavepi_detection.camera_publisher:main',
            'qr_detector = cavepi_detection.qr_detector:main',
            'caveline_detector = cavepi_detection.caveline_detector:main',
            'data_sender = cavepi_detection.data_sender:main',
            'test_camera = cavepi_detection.test_camera:main',
            'detected_lines = cavepi_detection.detected_lines:main',
        ],
    },
)

