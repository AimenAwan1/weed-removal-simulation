import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import FindPackageShare

def generate_launch_description():
    # Set the path to your SDF world file
    world_file = os.path.join(
        FindPackageShare('simulation').find('simulation'), 
        'src/worlds/building_robot.sdf'
    )

    # Optionally, set IGN_GAZEBO_RESOURCE_PATH to your worlds folder
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = os.path.join(
        FindPackageShare('simulation').find('simulation'), 
        'src/worlds'
    )

    # Launch Gazebo with your world file
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '--sdf', world_file],
        output='screen'
    )

    # Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    # Bridge camera topic from Gazebo to ROS
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/camera/image_raw', 'sensor_msgs/Image', 'ignition.msgs.Image'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rviz,
        bridge
    ])
