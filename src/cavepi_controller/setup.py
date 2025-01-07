from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cavepi_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alankritgupta',
    maintainer_email='alankritgupta@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_receiver = cavepi_controller.data_receiver:main',
            'pixhawk_logic = cavepi_controller.pixhawk_logic:main',
            'autopilot = cavepi_controller.autopilot:main',
            'test_pixhawk = cavepi_controller.test_pixhawk:main',
            'rope_pose_waypoints_pub = cavepi_controller.rope_pose_waypoints_pub:main',
            'test_depth_hold = cavepi_controller.test_depth_hold:main',
        ],
    },
)
