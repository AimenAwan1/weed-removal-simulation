import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Set the path to your SDF world file
    world_file = os.path.join(
        os.getenv('HOME'), 'auto-weed-removal/ros2_ws/src/simulation/src/worlds/building_robot.sdf'
    )

    # Optionally, set IGN_GAZEBO_RESOURCE_PATH to your worlds folder
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = os.path.join(
        os.getenv('HOME'), 'auto-weed-removal/ros2_ws/src/simulation/src/worlds'
    )

    # Launch Gazebo with your world file
    gazebo = ExecuteProcess(
        cmd=[
            'ign',
            'gazebo',
            PathJoinSubstitution([
                FindPackageShare('simulation'),
                'src',
                'worlds',
                'building_robot.sdf'
            ]),
            '--render-engine', 'ogre',
            '-v', '4'
        ],
        output='screen'
    )

    # Example of conditional node launch based on launch arguments
    rviz = None
    if LaunchConfiguration('use_rviz'):
        rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )

    # Bridge camera topic from Gazebo to ROS
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/image_raw', 'sensor_msgs/Image', 'ignition.msgs.Image'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
        rviz  # Make sure to include rviz as part of the launch description
    ])
