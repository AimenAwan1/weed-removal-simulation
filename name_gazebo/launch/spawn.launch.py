#!/usr/bin/python3
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro, yaml, copy

class NamespaceSpawner():
    def __init__(self, gazebo_pkg_name: str,robot_type:str,xacro_file:str) -> None:
        self.gazebo_pkg = get_package_share_directory(gazebo_pkg_name)
        self.xacro_file = xacro_file
        self.robot_type = robot_type
        name_launch_arg = DeclareLaunchArgument('name',default_value="")
        use_rviz_launch_arg = DeclareLaunchArgument('use_rviz',default_value="False")
        self.namespace = LaunchConfiguration('name')
        self.use_rviz = LaunchConfiguration('use_rviz')
        self.launch_description = LaunchDescription()
        
        self.launch_description.add_action(name_launch_arg)
        self.launch_description.add_action(use_rviz_launch_arg)
    
        pose_arguments = ['x','y','z','R','P','Y']
        self.pose = []
        for pose_argument in pose_arguments:
            self.pose.append(LaunchConfiguration(pose_argument))
            pose_launch_arg = DeclareLaunchArgument(pose_argument,default_value='0.0')
            self.launch_description.add_action(pose_launch_arg)
        opaque_function = OpaqueFunction(
            function=self.render_launch_config,
                args=[]
            )
        self.launch_description.add_action(opaque_function)
    def render_launch_config(self,context: LaunchContext):
        namespace = context.perform_substitution(self.namespace)
        use_rviz_str = context.perform_substitution(self.use_rviz)
        use_rviz = (use_rviz_str=='True') or (use_rviz_str=='true')
        
        spawn_xyz = [context.perform_substitution(position) for position in self.pose]
        
        gazebo_path = os.path.join(self.gazebo_pkg,'robot',self.xacro_file)
        
        self.modify_controller_config(namespace)
        robot_description = xacro.process_file(gazebo_path,mappings={'namespace_arg':namespace}).toxml()
        if namespace:
            prefix = namespace+'/'
        else:
            prefix = ''
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=namespace,
            parameters=[{
                'robot_description':robot_description,
                'frame_prefix':prefix
            }]
        )

        spawner = Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-topic', namespace+'/robot_description',
                '-entity',namespace+'/'+self.robot_type,
                '-x', spawn_xyz[0],
                '-y', spawn_xyz[1],
                '-z', spawn_xyz[2],
                '-R', spawn_xyz[3],
                '-P', spawn_xyz[4],
                '-Y', spawn_xyz[5],
            ]
        )

        node_config = os.path.join(self.gazebo_pkg,'config','node_config.yaml')
        
        with open(node_config,'r') as file:
            node_info = yaml.load(file,Loader=yaml.SafeLoader)
        node_action = []
        
        for node in node_info:
            node_args = copy.deepcopy(node)
            node_args['namespace'] = namespace
            remappings = []
            if 'remappings' in node.keys():
                for topic in node['remappings']:
                    remappings.append(tuple(topic))
                node_args['remappings'] = remappings
            node_action.append(Node(**node_args))
        
        self.launch_description.add_action(robot_state_publisher)
        self.launch_description.add_action(spawner)
        for node in node_action:
            self.launch_description.add_action(node)
        if use_rviz:
            rviz_file_name = 'config.rviz'
            if namespace:
                new_rviz_file = self.modify_rviz_config(namespace,rviz_file_name)
            else:
                new_rviz_file = os.path.join(self.gazebo_pkg,'config',rviz_file_name)                
            rviz2 = Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", new_rviz_file]
            )
            
            self.launch_description.add_action(rviz2)
    def modify_controller_config(self,namespace:str)->None:
        config_path = os.path.join(self.gazebo_pkg,'config','controller_config.yaml')
        new_config_path = os.path.join(self.gazebo_pkg,'config',namespace+'_controller_config'+'.yaml')
        with open(config_path,'r') as file:
            controller_param = yaml.load(file,yaml.SafeLoader)
        new_controller_param = {namespace:controller_param}
        with open(new_config_path,'w') as file:
            yaml.dump(new_controller_param,file)   
    def modify_rviz_config(self,namespace:str,file_name:str)->None:
        rviz_file = os.path.join(self.gazebo_pkg,'config',file_name)
        new_rviz_file = os.path.join(self.gazebo_pkg,'config',namespace+'_'+file_name)
                    
        with open(rviz_file,'r') as file:
            rviz_param = yaml.load(file,yaml.SafeLoader)
        frame = namespace+'/base_footprint'
        rviz_param['Visualization Manager']['Global Options']['Fixed Frame'] = frame
        for display in rviz_param['Visualization Manager']['Displays']:
            if display['Class']=='rviz_default_plugins/RobotModel':
                display['Description Topic']['Value']=namespace+'/robot_description'
                display['TF Prefix'] = namespace
            elif display['Class']=='rviz_default_plugins/LaserScan':
                display['Topic']['Value'] = namespace+'/scan'
            elif display['Class']=='rviz_default_plugins/Image':
                display['Topic']['Value'] = namespace+'/front_camera/image_raw'
        with open(new_rviz_file,'w') as file:
            yaml.dump(rviz_param,file) 
        return new_rviz_file


def generate_launch_description():
    spawner = NamespaceSpawner(
        gazebo_pkg_name='name_gazebo',
        robot_type = 'n501',
        xacro_file='name.gazebo.xacro'
    )
    
    return spawner.launch_description
