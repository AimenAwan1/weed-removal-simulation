# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os, copy, yaml

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro

class NamespaceSpawner():
    def __init__(self, ign_pkg_name: str,robot_type:str,xacro_file:str) -> None:
        self.ign_pkg = get_package_share_directory(ign_pkg_name)
        self.xacro_file = xacro_file
        self.robot_type = robot_type
        name_launch_arg = DeclareLaunchArgument('name',default_value="")
        world_launch_arg = DeclareLaunchArgument('world',default_value="empty")
        self.namespace = LaunchConfiguration('name')
        self.world = LaunchConfiguration('world')
        self.launch_description = LaunchDescription()
        self.launch_description.add_action(name_launch_arg)
        self.launch_description.add_action(world_launch_arg)
        opaque_function = OpaqueFunction(
                function=self.render_launch_config,
                    args=[]
                )
        self.launch_description.add_action(opaque_function)
    def render_launch_config(self,context: LaunchContext):
        namespace = context.perform_substitution(self.namespace)
        world = context.perform_substitution(self.world)
        xacro_file = os.path.join(self.ign_pkg,'robot',self.xacro_file)
        self.modify_controller_config(namespace)
        robot_description = xacro.process_file(xacro_file,mappings={'namespace_arg':namespace}).toprettyxml()
        
        urdf_file = os.path.join(self.ign_pkg,'robot',self.xacro_file[0:-5]+'urdf')
        with open(urdf_file, 'w') as file:
            file.write(robot_description)
        if namespace:
            prefix = namespace+'/'
        else:
            prefix = ''
        node_robot_state_publisher = Node(
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
            
        ignition_spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['world',world
                    ,'-file', urdf_file,
                    '-name', robot_name],
        )
        
        node_config = os.path.join(self.ign_pkg,'config','node_config.yaml')
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
        
        self.launch_description.add_action(node_robot_state_publisher)
        self.launch_description.add_action(ignition_spawn_entity)
        
        for node in node_action:
            self.launch_description.add_action(node)
    def modify_controller_config(self,namespace:str)->None:
        config_path = os.path.join(self.ign_pkg,'config','cart_controller_velocity.yaml')
        new_config_path = os.path.join(self.ign_pkg,'config',namespace+'_cart_controller_velocity.yaml')
        with open(config_path,'r') as file:
            controller_param = yaml.load(file,yaml.SafeLoader)
        new_controller_param = {namespace:controller_param}
        with open(new_config_path,'w') as file:
            yaml.dump(new_controller_param,file)   

def generate_launch_description():
    # Launch Arguments
    spawner = NamespaceSpawner(
        ign_pkg_name='name_ign',
        robot_type = 'cart',
        xacro_file='cart.gazebo.xacro'
    )
    return spawner.launch_description