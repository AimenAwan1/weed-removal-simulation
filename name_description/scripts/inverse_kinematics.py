#!/usr/bin/python3

# other libraries
from typing import List
import os
import yaml

# package module

# RCLPY libraries, classes, functions
import rclpy
from rclpy.node import Node

# ROS packages
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')
        self.command_publisher = self.create_publisher(Float64MultiArray,'wheel_velocity',10)
        self.cmd_vel_subscription = self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)
        self.cmd_vel = Twist()
        
        description_pkg = get_package_share_directory('name_description')
        properties_file = os.path.join(description_pkg,'config','properties.yaml')
        with open(properties_file,'r') as file:
            properties = yaml.load(file,Loader=yaml.SafeLoader)
        self.wheel_separation = float(properties['wheel_separation'])
        self.wheel_radius = float(properties['wheel_radius'])
        self.get_logger().info(f'{self.get_name()} has started.')
        
    def cmd_vel_callback(self,msg:Twist):
        cmd = Float64MultiArray()
        cmd.data = self.inverse_kinematics(msg.linear.x,msg.angular.z)
        self.command_publisher.publish(cmd)
    def inverse_kinematics(self,v,w):
        left_wheel_velocity = v/self.wheel_radius-w*self.wheel_separation/(2*self.wheel_radius)
        right_wheel_velocity = v/self.wheel_radius+w*self.wheel_separation/(2*self.wheel_radius)        
        return [left_wheel_velocity,right_wheel_velocity]

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
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
