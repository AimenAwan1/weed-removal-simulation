#!/usr/bin/python3

# other libraries
from typing import List
import os
import yaml
import math

# package module

# RCLPY libraries, classes, functions
import rclpy
from rclpy.node import Node

# ROS packages
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry

class InverseKinematicsController(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_controller')
        self.command_publisher = self.create_publisher(Float64MultiArray,'wheel_velocity',10)
        self.cmd_vel_subscription = self.create_subscription(Point,'target_position',self.target_callback,10)
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.cmd_vel = Twist()
        
        description_pkg = get_package_share_directory('name_description')
        properties_file = os.path.join(description_pkg,'config','properties.yaml')
        with open(properties_file,'r') as file:
            properties = yaml.load(file,Loader=yaml.SafeLoader)
        self.wheel_separation = float(properties['wheel_separation'])
        self.wheel_radius = float(properties['wheel_radius'])
        self.get_logger().info(f'{self.get_name()} has started.')

        self.target_position = None
        self.current_position = Point()

        self.kp_linear = 0.5
        self.ki_linear = 0.005
        self.kd_linear = 0.001
        self.kp_angular = 1.0
        self.ki_angular = 0.01
        self.kd_angular = 0.01

        self.prev_error_linear = 0.0
        self.integral_linear = 0.0
        self.prev_error_angular = 0.0
        self.integral_angular = 0.0

        self.prev_time = self.get_clock().now()

        self.control_timer = self.create_timer(0.1, self.control_loop)

    def target_callback(self, msg: Point):
        self.target_position = msg
        self.get_logger().info(f'Received target position: ({msg.x}, {msg.y})')

    def odom_callback(self, msg: Odometry):
        self.current_position = msg.pose.pose.position

    def control_loop(self):
        if self.target_position is None:
            return
        
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time

        self.get_logger().info(f'current x: {self.current_position.x}, current y: {self.current_position.y}')

        #position error
        dx = self.target_position.x - self.current_position.x
        dy = self.target_position.y - self.current_position.y
        distance = math.sqrt(dx**2 + dy**2)

        #linear velocity using PID
        error_linear = distance
        self.integral_linear += error_linear * dt
        derivative_linear = (error_linear -self.prev_error_linear) / dt
        self.prev_error_linear = error_linear

        linear_velocity = (self.kp_linear * error_linear 
                           + self.ki_linear * self.integral_linear 
                           + self.kd_linear * derivative_linear)
        
        #angular velocity using PID
        angle_to_target = math.atan2(dy, dx)
        error_angular = angle_to_target
        self.integral_angular += error_angular * dt
        derivative_angular = (error_angular - self.prev_error_angular) / dt
        self.prev_error_angular = error_angular

        angular_velocity = (self.kp_angular * error_angular 
                           + self.ki_angular * self.integral_angular 
                           + self.kd_angular * derivative_angular)
        
        if distance < 0.1:
            linear_velocity = 0.0
            angular_velocity = 0.0
            self.get_logger().info('Target reached, stopped the robot')

        self.cmd_vel.linear.x = 0.1 * linear_velocity
        self.cmd_vel.angular.z = 0.1 * angular_velocity

        cmd = Float64MultiArray()
        cmd.data = self.inverse_kinematics(self.cmd_vel.linear.x, self.cmd_vel.angular.z)
        self.command_publisher.publish(cmd)     

    def inverse_kinematics(self,v,w):
        left_wheel_velocity = v/self.wheel_radius-w*self.wheel_separation/(2*self.wheel_radius)
        right_wheel_velocity = v/self.wheel_radius+w*self.wheel_separation/(2*self.wheel_radius) 

        self.get_logger().info(f'Linear Velocity: {v}, Angular Velocity: {w}')
        self.get_logger().info(f'Left Wheel Velocity: {left_wheel_velocity}, Right Wheel Velocity: {right_wheel_velocity}')
               
        return [left_wheel_velocity,right_wheel_velocity]

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsController()
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