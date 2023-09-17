import logging
import time
import os
import rclpy
import yaml
import tf_transformations
from threading import Timer
import numpy as np
import random
from math import atan2, cos, sin, sqrt, pi

from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, UInt16, UInt16MultiArray, Float64
from geometry_msgs.msg import Pose, Twist, PointStamped, Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time



class OpenLoop(Node):
    def __init__(self):
        super().__init__('formation_controller')
        # Params
        self.declare_parameter('range', 2.0)
        self.declare_parameter('mPeriod', 1.0)
        self.declare_parameter('robot', 'mobile')
        self.declare_parameter('signal', 'cmd_vel')
        self.declare_parameter('signal_type', 'twist')
        self.declare_parameter('signal_shape', 'random')
        self.declare_parameter('signal_shape_file', 'path')
        self.declare_parameter('output', 'local_pose')
        self.declare_parameter('output_type', 'pose')

        # Publisher
        self.publisher_status = self.create_publisher(String,'open_loop/status', 10)
        self.publisher_ident = self.create_publisher(String,'ident/order', 10)
        self.path_publisher = self.create_publisher(Path, 'ident/path', 10)
        
        # Subscription
        self.sub_order = self.create_subscription(String, 'open_loop/order', self.order_callback, 10)

        self.initialize()

    def initialize(self):
        self.get_logger().info('Open Loop::inicialize() ok.')
        self.robot = self.get_parameter('robot').get_parameter_value().string_value
        signal = self.get_parameter('signal').get_parameter_value().string_value
        self.signal_type = self.get_parameter('signal_type').get_parameter_value().string_value
        
        if self.signal_type == 'pose':
            self.pub_signal = self.create_publisher(PoseStamped, signal, 10)
            self.target = PoseStamped()
            self.target.header.frame_id = "map"
        elif self.signal_type == 'twist':
            self.pub_signal = self.create_publisher(Twist, signal, 10)
            self.target = Twist()
        else:
            self.pub_signal = self.create_publisher(Float64, signal, 10)
        
        signal = self.get_parameter('output').get_parameter_value().string_value
        self.output_type = self.get_parameter('output_type').get_parameter_value().string_value
        
        if self.output_type == 'pose':
            self.sub_signal = self.create_subscription(PoseStamped, signal, self.output_callback, 10)
            self.output = PoseStamped()
        elif self.output_type == 'twist':
            self.sub_signal = self.create_subscription(Twist, signal, self.output_callback, 10)
            self.output = Twist()
        else:
            self.subb_signal = self.create_subscription(Float64, signal, self.output_callback, 10)
            self.output = Float64()

        self.period = self.get_parameter('mPeriod').get_parameter_value().double_value
        self.range = self.get_parameter('range').get_parameter_value().double_value
        self.shape = self.get_parameter('signal_shape').get_parameter_value().string_value
        shape_file = self.get_parameter('signal_shape_file').get_parameter_value().string_value
        self.new = False
        time = self.get_clock().now().to_msg()
        self.t_init = time.sec+time.nanosec*1e-9
        if self.shape == 'polygon':
             self.j = 0
             with open(shape_file, 'r') as file:
                self.points = yaml.safe_load(file)

        self.timer_task = self.create_timer(0.1, self.iterate)
        self.path = Path()
        self.path.header.frame_id = "map"

        self.get_logger().info('Open Loop::inicialized.')

    def _ready(self):
        msg = String()
        msg.data = 'ident_start'
        self.publisher_ident.publish(msg)
        self.new = True
        
    def order_callback(self,msg):
        self.get_logger().info('Open Loop::Order: "%s"' % msg.data)
        msg = String()
        msg.data = 'ident_start'
        self.publisher_ident.publish(msg)
        self.new = True

    def output_callback(self, msg):
        self.output = msg

    def polygon_drawing(self):
        self.get_logger().info('TO-DO')

    def iterate(self):
        if self.new:
            self.new = False
            if self.shape == 'random':
                if self.signal_type == 'pose':
                    self.target.pose.position.x = random.uniform(-self.range, self.range)
                    self.target.pose.position.y = random.uniform(-self.range, self.range)
                    if self.robot == 'dron':
                        self.target.pose.position.z = 1.0
                    else:
                        self.target.pose.position.z = 0.0

                    q = tf_transformations.quaternion_from_euler(0.0, 0.0, random.uniform(-pi, pi))
                    self.get_logger().info('Pose X %.3f  Y %.3f' % (self.target.pose.position.x, self.target.pose.position.y))
                    self.target.pose.orientation.x = q[0]
                    self.target.pose.orientation.y = q[1]
                    self.target.pose.orientation.z = q[2]
                    self.target.pose.orientation.w = q[3]
                    self.path.poses.append(self.target)
                    self.path_publisher.publish(self.path)
                    
                elif self.signal_type == 'twist':
                    self.target.linear.x = random.uniform(0.0, self.range)
                    self.target.angular.z = random.uniform(-self.range/20, self.range/20)
                    self.get_logger().info('CMD v %.3f  w %.3f' % (self.target.linear.x, self.target.angular.z))
                else:
                    self.target = Float64()
                t = random.uniform(self.period, self.period+2)
                self.get_logger().info('New t %f.' % t)
            else:
                if self.shape == 'polygon':
                    self.target.pose.position.x = self.points['P'+str(self.j)]['x'] * self.range
                    self.target.pose.position.y = self.points['P'+str(self.j)]['y'] * self.range
                    self.target.pose.position.z = self.points['P'+str(self.j)]['z']
                    t = self.points['P'+str(self.j)]['t']
                    self.get_logger().info('P%s: X %.3f  Y %.3f' % (str(self.j),self.target.pose.position.x, self.target.pose.position.y))
                    self.j = ((self.j+1) % len(self.points))
                if self.shape == 'circle':
                    time = self.get_clock().now().to_msg()
                    t = time.sec+time.nanosec*1e-9
                    self.target.pose.position.x = sin((t-self.t_init)*0.2) * self.range
                    self.target.pose.position.y = cos((t-self.t_init)*0.2) * self.range
                    if self.robot == 'dron':
                        self.target.pose.position.z = 1.0
                    else:
                        self.target.pose.position.z = 0.0
                    t = 0.1
                yaw = atan2(self.target.pose.position.y-self.output.pose.position.y,self.target.pose.position.x-self.output.pose.position.x)
                q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
                self.target.pose.orientation.x = q[0]
                self.target.pose.orientation.y = q[1]
                self.target.pose.orientation.z = q[2]
                self.target.pose.orientation.w = q[3]

            
            self.pub_signal.publish(self.target)  
            self.t_ready = Timer(t, self._ready)
            self.t_ready.start()  
        
        
        

def main(args=None):
    rclpy.init(args=args)
    openloop_node = OpenLoop()
    rclpy.spin(openloop_node)

    openloop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()