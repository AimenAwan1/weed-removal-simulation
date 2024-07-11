import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    #robot_description = Command([
    #    FindExecutable(name='sdf'), ' ',
    #    PathJoinSubstitution([
    #        FindPackageShare('simulation'), 'src', 'worlds', 'building_robot.sdf'
    #    ])
    #])

    """ robot_description_path = PathJoinSubstitution([
    FindPackageShare('simulation'), 'src', 'worlds', 'building_robot.sdf'
    ])

    robot_description = {'robot_description': Command([robot_description_path])}
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output ='screen'
    )

    # for RVIZ
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    robot_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot'],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'diff_drive_base_controller'],
        output='screen'
    )

    run_plotjuggler = ExecuteProcess(
        cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler'],
        output='screen',
        shell=True
    )
 """
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
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.path.join(os.getenv('HOME'), 'auto-weed-removal/ros2_ws/src/simulation/src/worlds')),
        gazebo,
        #robot_state_publisher_node,
        #robot_spawn_node,
        #joint_state_publisher_node,
        #load_joint_state_broadcaster,
        #load_diff_drive_base_controller,
        #run_plotjuggler,
        bridge,
        rviz  # Make sure to include rviz as part of the launch description
    ])
