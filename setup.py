import os
from setuptools import setup
from glob import glob

package_name = 'b_rover_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bavantha',
    maintainer_email='bavanthau@eng.pdn.ac.lk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'robot_controller = b_rover_controller.robot_controller:main',
          'robot_estimator = b_rover_controller.robot_estimator:main',
          'multi_robot_controller = b_rover_controller.multi_robot_controller:main',
          'multi_robot_estimator = b_rover_controller.multi_robot_estimator:main'
        ],
    },
)
