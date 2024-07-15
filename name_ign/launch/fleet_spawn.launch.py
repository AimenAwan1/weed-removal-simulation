from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, LogInfo
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os, yaml, xacro, copy
from name_ign.sdf_modifier import add_robot, add_controller_plugin
class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True
def modify_controller_config(config_path,new_config_path,fleet:list=None)->None:
        with open(config_path,'r') as file:
            controller_param = yaml.load(file,yaml.SafeLoader)
        new_param = copy.deepcopy(controller_param)
        
        for robot in fleet:
            for key,node in controller_param['controller_manager']['ros__parameters'].items():
                if not key == 'update_rate':
                    new_param['controller_manager']['ros__parameters'][robot['name']+'/'+key] = node
            
            nodes = {}
            for key,node in controller_param.items():
                if not key == 'controller_manager':
                    nodes[key] = node
                    for index,joint in enumerate(node['ros__parameters']['joints']):
                        nodes[key]['ros__parameters']['joints'][index]=robot['name']+'/'+nodes[key]['ros__parameters']['joints'][index]
            new_param[robot['name']]= nodes
        new_controller_param = copy.deepcopy(new_param)
        with open(new_config_path,'w') as file:
            yaml.dump(new_controller_param,file,Dumper=NoAliasDumper)  
           
def extract_info(input_str):
    # Remove leading '/' character if present
    if input_str.startswith('/'):
        input_str = input_str[1:]

    # Split the input string based on '/'
    parts = input_str.split('/')

    # Extract package from the first element
    package = parts[0]

    # Extract folder from the second to second-to-last elements
    folder = '/'.join(parts[1:-1])

    # Extract file from the last element
    file = parts[-1]

    return package, folder, file

def state_publisher_spawner(model_path,world,description_pkg,gazebo_pkg):
    robot_description = xacro.process_file(model_path,mappings={'description_pkg_arg':description_pkg,'gazebo_pkg_arg':gazebo_pkg}).toprettyxml()
    directory, filename = os.path.split(model_path)
    name, extension = os.path.splitext(filename)
    urdf_filepath = os.path.join(directory, f"{name}" + '.urdf')
    with open(urdf_filepath, 'w') as file:
        file.write(robot_description)
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description':robot_description
        }]
    )

    spawner = Node(
        package='ros_gz_sim', 
        executable='create',
        output='screen',
        arguments=[
            '-world',world,
            '-file', urdf_filepath,
        ]
    )
    return robot_state_publisher,spawner
def render_launch_config(context:LaunchContext,launch_description:LaunchDescription,configurations):
    description_pkg = context.perform_substitution(configurations['description_pkg'])
    model_info_path = context.perform_substitution(configurations['model'])
    fleet_info_path = context.perform_substitution(configurations['fleet_info'])
    node_config_path = context.perform_substitution(configurations['node_config'])
    world = context.perform_substitution(configurations['world'])
    model_info_pkg_name, model_info_folder, model_info_file = extract_info(model_info_path)
    fleet_info_pkg_name, fleet_info_folder, fleet_info_file = extract_info(fleet_info_path)
    node_config_pkg_name, fnode_config_folder, node_config_file = extract_info(node_config_path) 
    
    model_info_file = os.path.join(get_package_share_directory(model_info_pkg_name),model_info_folder,model_info_file)
    fleet_info_file = os.path.join(get_package_share_directory(fleet_info_pkg_name),fleet_info_folder,fleet_info_file)
    node_config_file = os.path.join(get_package_share_directory(node_config_pkg_name), fnode_config_folder, node_config_file)
    with open(fleet_info_file,'r') as file:
        fleet_info = yaml.load(file,Loader=yaml.SafeLoader)
    with open(model_info_file,'r') as file:
        model_info = yaml.load(file,Loader=yaml.SafeLoader)
    model_pkg_name, model_folder, model_file = extract_info(model_info['model_path'])
    controller_pkg_name, controller_folder, controller_file = extract_info(model_info['controller_config_path'])
    robot_type = model_info['model_name']

    controller_path = os.path.join(get_package_share_directory(controller_pkg_name),controller_folder,controller_file)
    new_controller_path = os.path.join(get_package_share_directory(controller_pkg_name),controller_folder,'new_'+controller_file)
    modify_controller_config(controller_path,new_controller_path,fleet_info)
            
    model_path = os.path.join(get_package_share_directory(model_pkg_name),model_folder,model_file)
    with open(model_path,'r') as file:
        content = file.read()
    for robot in fleet_info:
        content = add_robot(content,description_pkg,model_pkg_name,robot['name'])
    content = add_controller_plugin(content,controller_pkg_name)
    new_model_path = os.path.join(get_package_share_directory(model_pkg_name),model_folder,'new_'+model_file)
    with open(new_model_path,'w') as file:
        file.write(content)  
    robot_state_publisher,spawner = state_publisher_spawner(new_model_path,world,description_pkg,model_pkg_name)  
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(spawner)                   
    
    #### run the rest of the nodes with node_config_path
    
    #with open(node_config_file,'r') as file:
    #    node_info = yaml.load(file,Loader=yaml.SafeLoader)
    #node_action = []
    """
    for robot in fleet_info:
        
        for node in node_info:
            node_args = copy.deepcopy(node)
            
            if node['executable']=='spawner' and node['package']=='controller_manager':
                i = 0
                for field in node['arguments']:
                    if field == 'controller_manager':
                        node_args['arguments'][i] = '/controller_manager'
                    i = i + 1
            node_args['namespace'] = robot['name']
            remappings = []
            if 'remappings' in node.keys():
                for topic in node['remappings']:
                    remappings.append(tuple(topic))
                node_args['remappings'] = remappings
            node_action.append(Node(**node_args))
            
    for node in node_action:
        launch_description.add_action(node)
    """
def generate_launch_description():
    model_pkg_name = 'name_ign'
    fleet_pkg_name = 'name_ign'
    description_pkg_name = 'name_description'
    description_package = description_pkg_name
    default_model_path = os.path.join(model_pkg_name,'config','model_info_config.yaml')
    fleet_info_path = os.path.join(fleet_pkg_name,'config','fleet_info.yaml')
    node_config_path = os.path.join(fleet_pkg_name,'config','node_config.yaml')
    
    description_pkg_arg = DeclareLaunchArgument('description_pkg',default_value=description_package)
    model_path_arg = DeclareLaunchArgument('model_info_path',default_value=default_model_path)
    fleet_info_arg = DeclareLaunchArgument('fleet_info_path',default_value=fleet_info_path)
    node_config_arg = DeclareLaunchArgument('node_config_path',default_value=node_config_path)
    world_arg = DeclareLaunchArgument('world',default_value='empty')

    description_pkg = LaunchConfiguration('description_pkg')
    model_path = LaunchConfiguration('model_info_path')
    fleet_info = LaunchConfiguration('fleet_info_path')
    node_config = LaunchConfiguration('node_config_path')
    world = LaunchConfiguration('world')
    launch_description = LaunchDescription()
    
    configurations = {
        'description_pkg':description_pkg,
        'model':model_path,
        'fleet_info':fleet_info,
        'node_config':node_config,
        'world':world
    }
    opaque_function = OpaqueFunction(
                function=render_launch_config,
                    args=[launch_description,configurations]
                )
    launch_description.add_action(description_pkg_arg)
    launch_description.add_action(model_path_arg)
    launch_description.add_action(fleet_info_arg)
    launch_description.add_action(node_config_arg)
    launch_description.add_action(world_arg)
    launch_description.add_action(opaque_function)
    
    return launch_description
