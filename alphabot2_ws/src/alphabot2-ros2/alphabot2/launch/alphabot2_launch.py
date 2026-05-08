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
    use_obstable_avoidance_emergency_stop_arg = DeclareLaunchArgument(
        'use_obstacle_avoidance_emergency_stop',
        default_value='true',
        description='Whether to use obstacle avoidance emergency stop or not. If true, the robot will stop when an obstacle is detected by the IR sensors.')

     # Launch Configurations
    use_obstable_avoidance_emergency_stop = LaunchConfiguration('use_obstacle_avoidance_emergency_stop')

    motion_driver_node = Node(
        package="alphabot2",
        namespace=NAMESPACE,
        executable="motion_driver",
        output="screen",
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', MOTION_DRIVER_LOG_LVL],
        parameters=[{
                    'use_obstacle_avoidance_emergency_stop': use_obstable_avoidance_emergency_stop,
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

    virtual_odometer_node = Node(
        package="alphabot2",
        namespace=NAMESPACE,
        executable="virtual_odometer",
        output="screen",
        emulate_tty=True,
        arguments=['--ros-args',
                   '--log-level', VIRTUAL_ODOMETER_LOG_LVL],
    )

    qr_detector_node = Node(
        package="alphabot2",
        namespace=NAMESPACE,
        executable="QR_detector",
        output="screen",
        emulate_tty=True,
        arguments=['--ros-args',
                   '--log-level', QR_DETECTOR_LOG_LVL],
    )

    v4l2_camera_node = Node(
        package="v4l2_camera",
        namespace=NAMESPACE,
        executable="v4l2_camera_node",
        output="screen",
        emulate_tty=True,
        arguments=['--ros-args',
                   '--log-level', V4L2_CAMERA_LOG_LVL,],
        parameters=[{
       		 'image_size': [320, 240],
       		 'camera_info_url': 'file:///home/deec/camera_info.yaml',
   	 }],
    )

    
    line_sensors_node = Node(
 	 package="alphabot2",          # change if different package
   	 namespace=NAMESPACE,
   	 executable="line_sensors",    # your node executable name
   	 output="screen",
         emulate_tty=True,
	 arguments=['--ros-args', '--log-level', 'WARN'], 
     )

    return LaunchDescription([
        use_obstable_avoidance_emergency_stop_arg,
        ir_obstacle_sensors_node,
        motion_driver_node,
        virtual_odometer_node,
        qr_detector_node,
        v4l2_camera_node,
        line_sensors_node
    ])
