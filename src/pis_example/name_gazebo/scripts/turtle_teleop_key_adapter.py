#!/usr/bin/python3

# other libraries
from typing import List
import os
import yaml

# package module

# RCLPY libraries, classes, functions
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

# ROS packages
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory
def get_valid_parameter(candidate,default_parameter,condition:bool=True):
    if condition:
        parameter = candidate
        successful = True
    else:
        parameter = default_parameter
        successful = False
    return parameter,successful
class TurtleTeleopKeyAdapter(Node):
    def __init__(self):
        super().__init__('turtle_teleop_key_adapter')
        self.cmd_vel_publisher = self.create_publisher(Twist,'output_cmd_vel',10)
        self.cmd_vel_subscription = self.create_subscription(Twist,'input_cmd_vel',self.cmd_vel_callback,10)
        self.cmd_vel = Twist()
        self.default_parameters = {
            'key_press_hold_time':0.5,
            'scale_linear':0.5,
            'scale_angular':0.5,
            'deceleration_linear':0.5,
            'deceleration_angular':0.5
        }
        descriptions = {}
        descriptions['scale_linear'] = 'Scaling factor of linear component'
        descriptions['scale_angular'] = 'Scaling factor of angular component'
        descriptions['deceleration_linear'] = 'Linear deceleration after releasing key press'
        descriptions['deceleration_angular'] = 'Angular deceleration after releasing key press'
        descriptions['key_press_hold_time'] = 'The duration which the adpater hold the value of the most recently pressed key before decelerating'
        self.conditions = {}
        for name in self.default_parameters.keys():
            if name == 'key_press_hold_time':
                self.conditions[name] = lambda candidate:candidate>0
            else:
                self.conditions[name] = lambda candidate:True
        
        for name,value in self.default_parameters.items():
            self.declare_parameter(
                name=name,
                value=value,
                descriptor=ParameterDescriptor(description=descriptions[name])
            )
        
        self.declare_parameter(
            name='publish_rate',
            value=10.0,
            descriptor= ParameterDescriptor(description='Publishing rate of the velocity from the adapter')
        )
        rate_candidate = self.get_parameter('publish_rate').value
        if rate_candidate == 0.0:
            self.get_logger().warning(f'The publishing rate cannot set to 0 Hz. 10 Hz was used instead.')
            self.period = 0.1
        else:
            self.period = 1/rate_candidate
        
        self.counter = 0
        description_pkg = get_package_share_directory('name_description')
        properties_file = os.path.join(description_pkg,'config','properties.yaml')
        with open(properties_file,'r') as file:
            properties = yaml.load(file,Loader=yaml.SafeLoader)
        self.wheel_separation = float(properties['wheel_separation'])
        self.wheel_radius = float(properties['wheel_radius'])
        
        self.parameters = {}
        for name,value in self.default_parameters.items():
            candidate = self.get_parameter(name).value
            self.parameters[name],successful = get_valid_parameter(candidate,value,condition=self.conditions[name](candidate))
        
        self.add_on_set_parameters_callback(self.set_key_press_hold_time_callback)

        self.timer = self.create_timer(self.period,self.timer_callback)
        self.get_logger().info(f'{self.get_name()} has started.')
        self.get_logger().info(f'The timer has started. The publishing rate cannot be changed.')
        self.velocity = [0.0,0.0]
        self.state = 0
        
    def set_key_press_hold_time_callback(self,parameters:List[Parameter]):
        for param in parameters:
            for name,value in self.parameters.items():
                if param.name == name:
                    candidate = param.get_parameter_value().double_value
                    self.parameters[name],successful = get_valid_parameter(candidate,value,condition=self.conditions[name](candidate)) 
                    if not successful:
                        self.get_logger().info(f'Invalid value. The most recent valid value is used instead.')
        return SetParametersResult(successful=True)
        
    def timer_callback(self):
        self.counter = self.counter + self.period
        if self.state == 0 :
            if self.counter >= self.parameters['key_press_hold_time']:
                self.state = 1
                self.init_vel = self.velocity
        elif self.state == 1:
            
            if abs(self.init_vel[0])>0.0001:
                self.velocity[0] = (1-self.parameters['deceleration_linear']/abs(self.init_vel[0])*self.period)*self.velocity[0]
            else:
                self.velocity[0] = 0.0
            if abs(self.init_vel[1])>0.0001:
                self.velocity[1] = (1-self.parameters['deceleration_angular']/abs(self.init_vel[1])*self.period)*self.velocity[1]
            else:
                self.velocity[1] = 0.0
            if (self.velocity[0]==0.0) and (self.velocity[1]==0.0):
                self.state = 2
            cmd = Twist()
            cmd.linear.x = self.velocity[0]
            cmd.angular.z = self.velocity[1]
            self.cmd_vel_publisher.publish(cmd)


    def cmd_vel_callback(self,msg:Twist):
        self.state = 0
        self.counter = 0
        cmd_msg = Twist()
        cmd_msg.linear.x = msg.linear.x*self.parameters['scale_linear']
        cmd_msg.angular.z = msg.angular.z*self.parameters['scale_angular']
        self.velocity = [cmd_msg.linear.x,cmd_msg.angular.z]
        self.cmd_vel_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleTeleopKeyAdapter()
    try:
        while rclpy.ok(): # while the node isn't shut down
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node has stopped cleanly.')
    except SystemExit:
        node.get_logger().info('Node is complete.')
    except BaseException as exc:
        type = exc.__class__.__name__
        node.get_logger().error(f'{type} exception in node has occured.')
        raise # raise without argument = raise the last exception
    finally:
        node.destroy_node()
        rclpy.shutdown() 
if __name__=='__main__':
    main()
