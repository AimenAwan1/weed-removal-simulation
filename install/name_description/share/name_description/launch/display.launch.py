 #!/usr/bin/python3
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from name_description.config_modifier import modify_rviz_config
import os
import xacro

def render_namespace(context: LaunchContext, namespace,launch_description:LaunchDescription,use_rviz):
    namespace_str = context.perform_substitution(namespace)
    use_rviz_bool = context.perform_substitution(use_rviz) == 'True'
    
    description_pkg = get_package_share_directory('name_description')
        
    description_path = os.path.join(description_pkg,'robot','dynamics','name.xacro')
    robot_description = xacro.process_file(description_path).toxml()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=namespace_str,
        parameters=[{
            'robot_description':robot_description,
            'frame_prefix': namespace_str+'/'
        }]
    )

    if use_rviz_bool:
        description_pkg = get_package_share_directory('name_description')
        if namespace_str:
            new_rviz_file = modify_rviz_config(description_pkg,'config.rviz',namespace_str)
        else:
            new_rviz_file = os.path.join(description_pkg,'config','config.rviz')
        rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d',new_rviz_file],
            output = 'screen',
            condition = IfCondition(use_rviz)
        )
        launch_description.add_action(rviz2)
    launch_description.add_action(robot_state_publisher)

def generate_launch_description():
    
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_launch_arg = DeclareLaunchArgument('use_rviz', default_value='True')
    
    namespace = LaunchConfiguration('namespace')
    namespace_launch_arg = DeclareLaunchArgument('namespace', default_value='')
    
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=namespace
    )

    launch_description = LaunchDescription()
    
    opaque_function = OpaqueFunction(
        function=render_namespace, 
        args=[
            namespace,
            launch_description,
            use_rviz
        ]
    )

    launch_description.add_action(use_rviz_launch_arg)
    launch_description.add_action(namespace_launch_arg)
    launch_description.add_action(opaque_function)
    launch_description.add_action(joint_state_publisher_gui)
    return launch_description