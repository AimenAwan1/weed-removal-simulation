#!/usr/bin/python3
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro, yaml, copy
from name_ign.sdf_modifier import add_velocity_controller
class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True

class NamespaceSpawner():
    def __init__(self, ign_pkg_name: str,robot_type:str,xacro_file:str,is_fleet:bool=False) -> None:
        self.ign_pkg_name = ign_pkg_name
        self.ign_pkg = get_package_share_directory(ign_pkg_name)
        self.xacro_file = xacro_file
        self.robot_type = robot_type
        
        self.namespace = LaunchConfiguration('name')
        self.use_rviz = LaunchConfiguration('use_rviz')
        self.world = LaunchConfiguration('world')
        self.use_bridge = LaunchConfiguration('use_bridge')
        self.pose = []
        self.isFleet = is_fleet
    def modify_launch_description(self,launch_description:LaunchDescription,name:str=None,pose:dict=None,xacro_path:str=None):
        pose_arguments = ['x','y','z','R','P','Y']
        self.pose = []
        if name:
            pose_value = [pose.get(arg, 0.0) for arg in pose_arguments]
        else:
            name_launch_arg = DeclareLaunchArgument('name',default_value="")
            launch_description.add_action(name_launch_arg)
            for pose_argument in pose_arguments:
                self.pose.append(LaunchConfiguration(pose_argument))
                pose_launch_arg = DeclareLaunchArgument(pose_argument,default_value='0.0')
                launch_description.add_action(pose_launch_arg)
                
        use_rviz_launch_arg = DeclareLaunchArgument('use_rviz',default_value="False")
        world_launch_arg = DeclareLaunchArgument('world',default_value="empty")
        use_bridge_arg = DeclareLaunchArgument('use_bridge',default_value="True")
        launch_description.add_action(use_rviz_launch_arg)
        launch_description.add_action(world_launch_arg)
        launch_description.add_action(use_bridge_arg)
        if name is not None:
            opaque_function = OpaqueFunction(
                function=self.render_launch_config,
                    args=[launch_description,name,pose_value,xacro_path]
                )
        else:
            opaque_function = OpaqueFunction(
                function=self.render_launch_config,
                    args=[launch_description]
                )
        launch_description.add_action(opaque_function)
    def render_launch_config(self,context: LaunchContext,launch_description:LaunchDescription,name:str=None,pose:list=None,xacro_path:str=None):
        if name is not None:
            namespace = name
            spawn_xyz = [str(value) for value in pose]
        else:
            namespace = context.perform_substitution(self.namespace)
            spawn_xyz = [context.perform_substitution(position) for position in self.pose]
        
        use_rviz_str = context.perform_substitution(self.use_rviz)
        use_rviz = (use_rviz_str=='True') or (use_rviz_str=='true')
        world = context.perform_substitution(self.world)
        use_bridge_str = context.perform_substitution(self.use_bridge)
        use_bridge = (use_bridge_str=='True') or (use_bridge_str=='true')
        
        gazebo_path = os.path.join(self.ign_pkg,'robot',self.xacro_file)
            
        if self.isFleet:
            if xacro_path is not None:
                new_gazebo_path = xacro_path
            else:
                new_gazebo_path = gazebo_path
                #self.modify_controller_config(namespace)
        else:
            new_gazebo_path = os.path.join(self.ign_pkg,'robot',namespace+'_'+self.xacro_file)
            if not self.isFleet:
                with open(gazebo_path,'r') as file:
                    content = file.read()
                content = add_velocity_controller(content, namespace, self.ign_pkg_name)
                with open(new_gazebo_path,'w') as file:
                    file.write(content)
            self.modify_controller_config(namespace)
        robot_description = xacro.process_file(new_gazebo_path,mappings={'namespace_arg':namespace}).toprettyxml()
        urdf_file = os.path.join(self.ign_pkg,'robot',namespace+'_'+self.xacro_file[0:-5]+'urdf')
        with open(urdf_file, 'w') as file:
            file.write(robot_description)
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
        if namespace:
            robot_name = namespace+'/'+self.robot_type
        else:
            robot_name = self.robot_type
        spawner = Node(
            package='ros_gz_sim', 
            executable='create',
            output='screen',
            arguments=[
                '-world',world,
                '-file', urdf_file,
                '-x', spawn_xyz[0],
                '-y', spawn_xyz[1],
                '-z', spawn_xyz[2],
                '-R', spawn_xyz[3],
                '-P', spawn_xyz[4],
                '-Y', spawn_xyz[5],
                '-name', robot_name
            ]
        )
        launch_description.add_action(robot_state_publisher)
        launch_description.add_action(spawner)
        # if using multiple robots, use a shared bridge
        if use_bridge:    
            if namespace:
                bridge_file = self.modify_bridge_config(namespace)
            else:
                bridge_file = os.path.join(self.ign_pkg,'config','ros_gz_bridge.yaml')
            ros_gz_bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                namespace=namespace,
                arguments=[
                    '--ros-args',
                    '-p',
                    f'config_file:={bridge_file}',
                ],
                output='screen',
            )
            launch_description.add_action(ros_gz_bridge)
            
            """
            # currently use regular bridge
            # not sure how to use image bridge with namespace
            
            with open(bridge_file,'r') as file:
                topics = yaml.load(file,Loader=yaml.SafeLoader)
            for topic in topics:
                if topic['ros_type_name'] == "sensor_msgs/msg/CameraInfo":
                    ros_image_bridge= Node(
                        package='ros_gz_image',
                        executable='image_bridge',
                        namespace=namespace,
                        arguments=['camera/image_raw'],
                        parameters=[{'expand_gz_topic_names':True}],
                        output='screen',
                    )
            #launch_description.add_action(ros_image_bridge)

            """
            
        # other nodes
        node_config = os.path.join(self.ign_pkg,'config','node_config.yaml')
        
        with open(node_config,'r') as file:
            node_info = yaml.load(file,Loader=yaml.SafeLoader)
        
        node_action = []
        
        for node in node_info:
            node_args = copy.deepcopy(node)
            
            if node['executable']=='spawner' and node['package']=='controller_manager':
                i = 0
                for field in node['arguments']:
                    if field == 'controller_manager':
                        node_args['arguments'][i] = '/controller_manager'
                    i = i + 1
            node_args['namespace'] = namespace
            remappings = []
            if 'remappings' in node.keys():
                for topic in node['remappings']:
                    remappings.append(tuple(topic))
                node_args['remappings'] = remappings
            node_action.append(Node(**node_args))
            
        for node in node_action:
            launch_description.add_action(node)

        #load_joint_trajectory_controller = ExecuteProcess(
        #cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controllers'],
        #output='screen')
        #
        # launch_description.add_action(load_joint_trajectory_controller)

        if use_rviz:
            rviz_file_name = 'config.rviz'
            if namespace:
                new_rviz_file = self.modify_rviz_config(namespace,rviz_file_name)
            else:
                new_rviz_file = os.path.join(self.ign_pkg,'config',rviz_file_name)                
            rviz2 = Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", new_rviz_file]
            )
            
            launch_description.add_action(rviz2)
    def modify_bridge_config(self,namespace:str)->None:
        config_path = os.path.join(self.ign_pkg,'config','ros_gz_bridge.yaml')
        new_config_path = os.path.join(self.ign_pkg,'config',namespace+'_ros_gz_bridge'+'.yaml')
        with open(config_path,'r') as file:
            bridge_param = yaml.load(file,yaml.SafeLoader)
        for topic in bridge_param:
            topic['ros_topic_name'] = topic['ros_topic_name']
            topic['gz_topic_name'] = namespace +'/'+ topic['gz_topic_name']
        
        with open(new_config_path,'w') as file:
            yaml.dump(bridge_param,file)
        return new_config_path
    def modify_controller_config(self,namespace:str,fleet:list=None)->None:
        config_path = os.path.join(self.ign_pkg,'config','controller_config.yaml')
        with open(config_path,'r') as file:
            controller_param = yaml.load(file,yaml.SafeLoader)
        if fleet:
            new_param = copy.deepcopy(controller_param)
            new_config_path = os.path.join(self.ign_pkg,'config','new_controller_config.yaml')
            
            for robot in fleet:
                for key,node in controller_param['controller_manager']['ros__parameters'].items():
                    if not key == 'update_rate':
                        new_param['controller_manager']['ros__parameters'][robot['name']+'/'+key] = node
                for key,node in controller_param.items():
                    nodes = {}
                    if not key == 'controller_manager':
                        nodes[key] = node
                    new_param[robot['name']]= nodes
            new_controller_param = copy.deepcopy(new_param)
        else:
            new_config_path = os.path.join(self.ign_pkg,'config',namespace+'_controller_config'+'.yaml')
            new_controller_param = {namespace:controller_param}
        with open(new_config_path,'w') as file:
            yaml.dump(new_controller_param,file,Dumper=NoAliasDumper)  
            
    def modify_rviz_config(self,namespace:str,file_name:str)->None:
        rviz_file = os.path.join(self.ign_pkg,'config',file_name)
        new_rviz_file = os.path.join(self.ign_pkg,'config',namespace+'_'+file_name)
                    
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

