
import glob
import os

from setuptools import find_packages, setup

package_name = 'ros_to_serial'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (share_dir + '/launch', glob.glob(os.path.join('launch','*.launch.py')),)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='irl',
    maintainer_email='face5921@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_read = ros_to_serial.serial_read:main',
            'serial_write = ros_to_serial.serial_write:main',
            'esp_motor = ros_to_serial.esp_motor:main',
            'esp_imu = ros_to_serial.esp_imu:main',
            'serial_FT = ros_to_serial.serial_FT:main',
        ],
    },
)
