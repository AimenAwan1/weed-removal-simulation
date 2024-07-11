from launch import LaunchDescription
from name_ign.spawner import NamespaceSpawner

def generate_launch_description():
    launch_description = LaunchDescription()
    spawner = NamespaceSpawner(
        ign_pkg_name='name_ign',
        robot_type = 'n501',
        xacro_file='name.gazebo.xacro'
    )
    
    spawner.modify_launch_description(launch_description)
    
    return launch_description
