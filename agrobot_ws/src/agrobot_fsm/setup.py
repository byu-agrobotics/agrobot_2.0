from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agrobot_fsm'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nelson Durrant',
    maintainer_email='snelsondurrant@gmail.com',
    description='Task state machines for the BYU Agrobotics Team',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collect_fsm = agrobot_fsm.collect_fsm:main',
            'handle_fsm = agrobot_fsm.handle_fsm:main',
            'navigate_fsm = agrobot_fsm.navigate_fsm:main',
            'sort_fsm = agrobot_fsm.sort_fsm:main',
        ],
    },
)