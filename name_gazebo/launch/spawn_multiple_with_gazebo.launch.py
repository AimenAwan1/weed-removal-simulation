 #!/usr/bin/python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
def spawn(name,pose):
    full_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('name_gazebo'),
                'launch',
                'spawn.launch.py'
            ])
        ]),
        launch_arguments={
            'name':name,
            'x':str(pose[0]),
            'y':str(pose[1]),
            'theta':str(pose[2])
        }.items()
    )
    return full_spawn
def generate_launch_description():
    
    launch_description = LaunchDescription()
    
    robot_names = ['adrian','bethoven','charlotte','derek','evangeline']
    y = 0.0
    time_stamp = 0.0
    for name in robot_names:
        time_stamp = time_stamp + 5.0
        spawn_action = spawn(name,[0.0,y,0.0])
        launch_description.add_action(TimerAction(period=time_stamp,actions=[spawn_action]))
        y = y + 1.0
    
    return launch_description
