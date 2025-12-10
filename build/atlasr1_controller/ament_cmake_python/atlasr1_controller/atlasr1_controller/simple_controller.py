#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np

class SimpleController(Node):

    def __init__(self):
        super().__init__("simple_controller")
        self.declare_parameter("wheel_radius", 0.019)
        self.declare_parameter("wheel_separation", 0.0774)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel radius %f" %self.wheel_radius_)
        self.get_logger().info("Using wheel separation %f" %self.wheel_separation_)

        self.wheel_pub = self.create_publisher(Float64MultiArray,"simple_velocity_controller/commands",10)
        self.sub = self.create_subscription(TwistStamped,"/atlasr1_controller/cmd_vel",self.velCallback,10)
        # 3 wheel angles (rad)
        theta = np.array([
            np.radians(0),
            np.radians(120),
            np.radians(240)
        ])
        local_matrix = np.identity(3)*np.cos(theta[0])
        transform_matrix = np.array([
        [-np.sin(theta[0]),np.cos(theta[0]),self.wheel_separation_],
        [-np.sin(theta[0]+theta[1]),np.cos(theta[0]+theta[1]),self.wheel_separation_],
        [-np.sin(theta[0]+theta[2]),np.cos(theta[0]+theta[2]),self.wheel_separation_]
        ])
        
        self.A = np.matmul(local_matrix,transform_matrix)/self.wheel_radius_
        # Build A matrix for inverse kinematics
        #A = [[-np.sin(th), np.cos(th), 0.019] for th in theta]
        #self.A = (1.0 / self.wheel_separation_) * np.array(A)     # 3×3 matrix

    def velCallback(self, msg):
        robot_speed = np.array([msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z])
        wheel_speed = np.matmul(self.A,robot_speed)

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [float(wheel_speed[0]), float(wheel_speed[1]), float(wheel_speed[2])]
        self.wheel_pub.publish(wheel_speed_msg)


def main():
    rclpy.init()
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
