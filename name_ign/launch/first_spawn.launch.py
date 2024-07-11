from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os, yaml
from name_ign.sdf_modifier import add_velocity_controller
from name_ign.spawner import NamespaceSpawner


def generate_launch_description():
    is_fleet = True
    launch_description = LaunchDescription()
    pkg_name = 'name_ign'
    ign_pkg = get_package_share_directory(pkg_name)
    pkg = ign_pkg

    # modify the first file
    fleet_info_path = os.path.join(ign_pkg,'config','fleet_info.yaml')
    with open(fleet_info_path,'r') as file:
        fleet = yaml.load(file,Loader=yaml.SafeLoader)
    
    xacro_file = 'name.gazebo.xacro'
    xacro_path = os.path.join(pkg,'robot',xacro_file)
    
    with open(xacro_path,'r') as file:
        content = file.read()    
    spawner = NamespaceSpawner(
        ign_pkg_name='name_ign',
        robot_type = 'n501',
        xacro_file='name.gazebo.xacro',
        is_fleet = is_fleet
    )
    first = True
    for robot in fleet:
        if first:
            new_xacro_path = os.path.join(pkg,'robot',robot['name']+'_'+xacro_file)
            name = robot['name']
            first = False
        spawner.modify_controller_config(robot['name'],fleet)
    content = add_velocity_controller(content,name,pkg_name,is_fleet=is_fleet) 
    with open(new_xacro_path,'w') as file:
        file.write(content) 
    
    first = True
    for robot in fleet:
        if first:
            spawner.modify_launch_description(
                launch_description=launch_description,
                name=robot['name'],
                pose=robot['pose'],
                xacro_path=new_xacro_path
            )
            first = False
        else:
            spawner.modify_launch_description(
                launch_description=launch_description,
                name=robot['name'],
                pose=robot['pose']
            )
    
    return launch_description
