import rclpy,math
from geometry_msgs.msg import Twist, Pose
from controller import Node


HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

def rot2quat(R):
    R11, R12, R13 = R[0][0], R[0][1], R[0][2]
    R21, R22, R23 = R[1][0], R[1][1], R[1][2]
    R31, R32, R33 = R[2][0], R[2][1], R[2][2]

    # Calculate quaternion elements
    qw = math.sqrt(1 + R11 + R22 + R33) / 2
    qx = (R32 - R23) / (4 * qw)
    qy = (R13 - R31) / (4 * qw)
    qz = (R21 - R12) / (4 * qw)

    # Return the unit quaternion
    return [qw, qx, qy, qz]

class MyRobotDriver:
    def init(self, webots_node, properties):
        
        self.__robot = webots_node.robot
        self.__supervisor = self.__robot.getSelf()
        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')
        

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__pub = self.__node.create_publisher(Pose,'pose',1)
        self.__node.create_timer(0.1,self.__timer_callback)
        
    def __timer_callback(self):
        pose_msg = Pose()
        pose = self.__supervisor.getPose()
        position = [pose[3],pose[7],pose[11]]
        rotation_matrix = [pose[0:3],pose[4:7],pose[8:11]]
        orientation = rot2quat(rotation_matrix)
        pose_msg.position.x = position[0]
        pose_msg.position.y = position[1]
        pose_msg.position.z = position[2]
        pose_msg.orientation.w = orientation[0]
        pose_msg.orientation.x = orientation[1]
        pose_msg.orientation.y = orientation[2]
        pose_msg.orientation.z = orientation[3]
        self.__pub.publish(pose_msg)
    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)