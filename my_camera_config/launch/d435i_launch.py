import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Path to official RealSense launch
    rs_launch_dir = get_package_share_directory('realsense2_camera')
    
    # 2. Path to your custom params file
    config_path = os.path.join(
        get_package_share_directory('my_camera_config'),
        'launch',
        'd435i_params.yaml'
    )

    # 3. Define the RealSense Node (Included Launch)
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rs_launch_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'params_file': config_path,
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2'
        }.items()
    )

    # 4. Define the Relay Node with explicit QoS
    imu_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='imu_relay',
        # We use 'parameters' to set the internal node state
        parameters=[{
            'reliability': 'reliable',
            'durability': 'volatile',
            'input_topic': '/camera/camera/imu',
            'output_topic': '/camera/camera/imu_reliable'
        }],
        # We keep 'arguments' as a secondary enforcement
        arguments=[
            '/camera/camera/imu', 
            '/camera/camera/imu_reliable', 
            '--qos-reliability', 'reliable'
        ],
        output='screen'
    )
    return LaunchDescription([
        realsense_node,
        imu_relay_node
    ])
