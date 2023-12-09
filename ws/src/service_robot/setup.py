import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'service_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cmtabr',
    maintainer_email='caio.abreu@sou.inteli.edu.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_socket = service_robot.web_socket:main',
            'logger = service_robot.logger:main',
            'robot_controller = service_robot.robot_controller:main',
        ],
    },
)
