import logging
import time
import rclpy
from threading import Timer
import numpy as np
import math

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist

robot_list = list()

class Robot():
    def __init__(self, parent, id):
        self.parent = parent
        self.id = id
        self.pose = Pose()
        self.sub_pose = self.parent.create_subscription(Pose, self.id + '/pose', self.gtpose_callback, 10)
        self.publisher_order = self.parent.create_publisher(String, self.id + '/order', 10)

    def gtpose_callback(self, msg):
        self.pose = msg

    def order_callback(self, msg):
        self.publisher_order.publish(msg)

###################
##  Swarm Class  ##
###################
class SwarmDriver(Node):
    def __init__(self):
        super().__init__('swarm_driver')
        # Params
        self.declare_parameter('robots', 'robot01')
        # Services

        # Publisher
        self.publisher_status = self.create_publisher(String,'swarm/status', 10)
        self.publisher_pose = self.create_publisher(Pose,'swarm/pose', 10)
        # Subscription
        self.sub_order = self.create_subscription(String, 'swarm/cf_order', self.order_callback, 10)
        self.sub_goal_pose = self.create_subscription(Pose, 'swarm/goal_pose', self.goalpose_callback, 10)
        # 
        self.initialize()
        self.timer_task = self.create_timer(0.02, self.centroid_estimate)

    def initialize(self):
        self.get_logger().info('SwarmDriver::inicialize() ok.')
        # Read Params
        aux = self.get_parameter('robots').get_parameter_value().string_value
        robots = aux.split(', ')
        for i in range(len(robots)):
            robot = Robot(self, robots[i])
            robot_list.append(robot)

    def order_callback(self, msg):
        self.get_logger().info('SWARM::Order: "%s"' % msg.data)
        for robot in robot_list:
            robot.order_callback(msg)
        if msg.data == 'distance_formation_run':
            msg = String()
            msg.data = 'Ready'
            self.publisher_status.publish(msg)

    def goalpose_callback(self, msg):
        self.get_logger().info('SWARM::New Goal pose: In progress ...')

    def centroid_estimate(self):
        msg = Pose()
        x = y = z = 0
        for robot in robot_list:
            x += robot.pose.position.x
            y += robot.pose.position.y
            z += robot.pose.position.z

        msg.position.x = x/len(robot_list)
        msg.position.y = y/len(robot_list)
        msg.position.z = z/len(robot_list)

        self.publisher_pose.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    swarm_driver = SwarmDriver()
    rclpy.spin(swarm_driver)

    swarm_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

