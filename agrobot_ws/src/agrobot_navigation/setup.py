from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agrobot_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('lib', package_name), ['agrobot_navigation/roboclaw_3.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nelson Durrant',
    maintainer_email='snelsondurrant@gmail.com',
    description='Navigation system for the BYU Agrobotics Team',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roboclaw_wrapper = agrobot_navigation.roboclaw_wrapper:main',
            'drive_controller = agrobot_navigation.drive_controller:main',
            'test_drive_straight = agrobot_navigation.test_drive_straight:main',
            'test_center = agrobot_navigation.test_center:main',
            'test_turn = agrobot_navigation.test_turn:main',
        ],
    },
)
