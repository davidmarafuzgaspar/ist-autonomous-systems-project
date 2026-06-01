from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

NAMESPACE = "alphabot2"
IMAGE_SIZE = "[320,240]"

MOTION_DRIVER_LOG_LVL = "WARN"
IR_OBSTACLE_SENSORS_LOG_LVL = "WARN"
VIRTUAL_ODOMETER_LOG_LVL = "WARN"
QR_DETECTOR_LOG_LVL = "WARN"
V4L2_CAMERA_LOG_LVL = "FATAL"


def generate_launch_description():

    # Launch Arguments
    force_obstacle_stop_arg = DeclareLaunchArgument(
        'force_obstacle_stop',
        description='If true, the robot force-stops when an obstacle is detected by IR sensors.')

     # Launch Configurations
    force_obstacle_stop = LaunchConfiguration('force_obstacle_stop')

    motion_driver_node = Node(
        package="alphabot2",
        namespace=NAMESPACE,
        executable="motion_driver",
        output="screen",
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', MOTION_DRIVER_LOG_LVL],
        parameters=[{
                    'force_obstacle_stop': force_obstacle_stop,
                }],
    )

    ir_obstacle_sensors_node = Node(
        package="alphabot2",
        namespace=NAMESPACE,
        executable="IR_obstacle_sensors",
        output="screen",
        emulate_tty=True,
        arguments=['--ros-args',
                   '--log-level', IR_OBSTACLE_SENSORS_LOG_LVL],
    )

    ir_line_sensors_node = Node(
        package="alphabot2",          
        namespace=NAMESPACE,
        executable="IR_line_sensors",   
        output="screen",
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'WARN'], 
     )

    # virtual_odometer_node = Node(
    #     package="alphabot2",
    #     namespace=NAMESPACE,
    #     executable="virtual_odometer",
    #     output="screen",
    #     emulate_tty=True,
    #     arguments=['--ros-args',
    #                '--log-level', VIRTUAL_ODOMETER_LOG_LVL],
    # )

    # qr_detector_node = Node(
    #     package="alphabot2",
    #     namespace=NAMESPACE,
    #     executable="QR_detector",
    #     output="screen",
    #     emulate_tty=True,
    #     arguments=['--ros-args',
    #                '--log-level', QR_DETECTOR_LOG_LVL],
    # )

    # v4l2_camera_node = Node(
    #     package="v4l2_camera",
    #     namespace=NAMESPACE,
    #     executable="v4l2_camera_node",
    #     output="screen",
    #     emulate_tty=True,
    #     arguments=['--ros-args',
    #                '--log-level', V4L2_CAMERA_LOG_LVL,],
    #     parameters=[{
    #     	 'image_size': [320, 240],
    #     	 'camera_info_url': 'file:///home/deec/camera_info.yaml',
    #	 }],
    # )

    return LaunchDescription([
        force_obstacle_stop_arg,
        motion_driver_node,
        ir_obstacle_sensors_node,
        ir_line_sensors_node
        # virtual_odometer_node,
        # qr_detector_node,
        # v4l2_camera_node,
    ])
