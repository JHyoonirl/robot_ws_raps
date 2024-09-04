# 파일: launch/your_launch_file.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 문자열 변수 선언
    usb_port_motor  = DeclareLaunchArgument(
        'usb_port_motor', default_value='/dev/ttyAMA0',
        description='USB port for the ESP32 board')
    
    usb_port_thigh  = DeclareLaunchArgument(
        'usb_port_thigh', default_value='/dev/ttyAMA1',
        description='USB port for the ESP32 board')
    
    usb_port_shank  = DeclareLaunchArgument(
        'usb_port_shank', default_value='/dev/ttyAMA2',
        description='USB port for the ESP32 board')
    # 노드 설정
    motor_node = Node(
        package='ros_to_serial',
        executable='esp_motor',
        name='motor_node',
        parameters=[{
            'usb_port': LaunchConfiguration('usb_port_motor')
        }]
    )

    imu_thigh_node = Node(
        package='ros_to_serial',
        executable='esp_imu',
        name='imu_thigh_node',
        parameters=[{
            'usb_port': LaunchConfiguration('usb_port_thigh')
        }]
    )

    imu_shank_node = Node(
        package='ros_to_serial',
        executable='esp_imu',
        name='imu_shank_node',
        parameters=[{
            'usb_port': LaunchConfiguration('usb_port_shank')
        }]
    )

    return LaunchDescription([
        usb_port_motor, 
        usb_port_thigh,
        usb_port_shank,
        motor_node,
        imu_thigh_node,
        imu_shank_node
    ])
