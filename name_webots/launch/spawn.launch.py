import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import EmitEvent,RegisterEventHandler
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
import xacro

def generate_launch_description():
    webot_pkg = get_package_share_directory('name_webots')
    xacro_name = 'name_multi.webots.xacro'
    urdf_name = xacro_name[0:-5]+'urdf'
    robot_description_path = os.path.join(webot_pkg,'robot',xacro_name)

    
    robot_description = xacro.process_file(robot_description_path,mappings={}).toprettyxml()
    urdf_file = os.path.join(webot_pkg,'robot',urdf_name)
    with open(urdf_file, 'w') as file:
        file.write(robot_description)
    
    webots = WebotsLauncher(
        world=os.path.join(webot_pkg, 'worlds', 'my_world_multi.wbt')
    )

    my_robot1_driver = WebotsController(
        robot_name='my_robot1',
        parameters=[
            {'robot_description': urdf_file},
        ]
    )
    my_robot2_driver = WebotsController(
        robot_name='my_robot2',
        parameters=[
            {'robot_description': urdf_file},
        ]
    )
    my_robot3_driver = WebotsController(
        robot_name='my_robot3',
        parameters=[
            {'robot_description': urdf_file},
        ]
    )
    
    obstacle_avoider = Node(
        package='name_webots',
        executable='obstacle_avoider.py',
    )

    shutdown = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=webots,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    launch_description = LaunchDescription()
    launch_description.add_action(webots)
    launch_description.add_action(my_robot1_driver)
    launch_description.add_action(my_robot2_driver)
    launch_description.add_action(my_robot3_driver)
    #launch_description.add_action(obstacle_avoider)
    launch_description.add_action(shutdown)

    return launch_description