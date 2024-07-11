 #!/usr/bin/python3
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import xml.etree.ElementTree as ET

def tag_replace(file_path:str,name:str):
    tree = ET.parse(file_path)
    root = tree.getroot()
    world_element = root.find('.//world[@name="default"]')
    # Find the first instance of <world>
    if world_element is not None:
        # Modify the value of the "name" attribute
        world_element.set('name', name)
        tree.write(file_path)

def render_launch_config(context:LaunchContext,launch_description:LaunchDescription,name:LaunchConfiguration):
    world_name = context.perform_substitution(name)
    ros_ign_pkg = get_package_share_directory('ros_gz_sim')
    ign_pkg = get_package_share_directory('name_ign')
    ign_launch_file = os.path.join(ros_ign_pkg,'launch','gz_sim.launch.py')
    world_file = os.path.join(ign_pkg,'world','empty.sdf')
    if world_name:
        tag_replace(world_file,world_name)
    ign = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ign_launch_file),
        launch_arguments={
            'gz_args':world_file
        }.items())
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(ign_pkg, 'world')
        ]
    )
    bridge_file = os.path.join(ign_pkg,'config','global_bridge.yaml')
        
    global_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='global_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_file}',
        ],
        output='screen',
    )
    launch_description.add_action(ign)
    launch_description.add_action(ign_resource_path)
    launch_description.add_action(global_bridge)
def generate_launch_description():
    name_launch_arg = DeclareLaunchArgument('world_name',default_value="empty")
    name = LaunchConfiguration('world_name')
    launch_description = LaunchDescription()
    launch_description.add_action(name_launch_arg)
    
    opaque_function = OpaqueFunction(
        function=render_launch_config,
        args=[launch_description,name]
        )
    launch_description.add_action(opaque_function)
    
    return launch_description