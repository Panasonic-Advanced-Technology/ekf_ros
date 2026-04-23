from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ekf_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch ファイル
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # config ファイル
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='EKF ROS2 package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_node = ekf_ros.ekf_node:main',
            'ekf_viewer = ekf_ros.viewer:main',
        ],
    },
)