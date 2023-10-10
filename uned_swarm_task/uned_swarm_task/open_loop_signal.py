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
        self.declare_parameter('signal_value', 'linear.x')
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
        self.signal_value = self.get_parameter('signal_value').get_parameter_value().string_value
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
        # Relé
        self.dseta = 0.05
        self.rele = False

        self.non_stop= True
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
        if self.non_stop:
            msg = String()
            msg.data = 'ident_start'
            self.publisher_ident.publish(msg)
            self.new = True
        
    def order_callback(self,msg):
        self.get_logger().info('Open Loop::Order: "%s"' % msg.data)
        if msg.data == 'start':
            msg = String()
            msg.data = 'ident_start'
            self.publisher_ident.publish(msg)
            self.new = True
        elif msg.data == 'stop':
            self.non_stop= False
        

    def output_callback(self, msg):
        self.output = msg

    def iterate(self):
        if self.new:
            self.new = False
            if self.shape == 'random':
                if self.signal_type == 'pose':
                    PoseStamp = PoseStamped()
                    PoseStamp.header.stamp = self.get_clock().now().to_msg()
                    if self.signal_value == 'position.x':
                        PoseStamp.pose.position.x = random.uniform(-self.range, self.range)
                        PoseStamp.pose.position.y = self.output.pose.position.y
                        PoseStamp.pose.position.z = self.output.pose.position.z
                    if self.signal_value == 'position.y':
                        PoseStamp.pose.position.x = self.output.pose.position.x
                        PoseStamp.pose.position.y = random.uniform(-self.range, self.range)
                        PoseStamp.pose.position.z = self.output.pose.position.z
                    if self.signal_value == 'position.z':
                        PoseStamp.pose.position.x = self.output.pose.position.x
                        PoseStamp.pose.position.y = self.output.pose.position.y
                        if self.robot == 'dron':
                            PoseStamp.pose.position.z = random.uniform(1.0-self.range, 1.0+self.range)
                        else:
                            PoseStamp.pose.position.z = 0.0
                    if self.signal_value == 'full':
                        PoseStamp.pose.position.x = random.uniform(-self.range, self.range)
                        PoseStamp.pose.position.y = random.uniform(-self.range, self.range)
                        if self.robot == 'dron':
                            PoseStamp.pose.position.y = random.uniform(1.0-self.range, 1.0+self.range)
                        else:
                            PoseStamp.pose.position.z = 0.0
                    q = tf_transformations.quaternion_from_euler(0.0, 0.0, random.uniform(-pi, pi))
                    self.get_logger().info('Pose X %.3f  Y %.3f' % (PoseStamp.pose.position.x, PoseStamp.pose.position.y))
                    PoseStamp.pose.orientation.x = q[0]
                    PoseStamp.pose.orientation.y = q[1]
                    PoseStamp.pose.orientation.z = q[2]
                    PoseStamp.pose.orientation.w = q[3]
                    self.path.header.stamp = self.get_clock().now().to_msg()
                    self.path.poses.append(PoseStamp)
                    self.path_publisher.publish(self.path)
                    self.target = PoseStamp
                elif self.signal_type == 'twist':
                    if self.signal_value == 'linear.x' or self.signal_value == 'full':
                        self.target.linear.x = random.uniform(0.0, self.range)
                    if self.signal_value == 'linear.y' or self.signal_value == 'full':
                        self.target.linear.y = random.uniform(-self.range, self.range)
                    if self.signal_value == 'linear.z' or self.signal_value == 'full':
                        self.target.linear.z = random.uniform(-self.range, self.range)
                    if self.signal_value == 'angular.z' or self.signal_value == 'full':
                        self.target.angular.z = random.uniform(-self.range/20, self.range/20)
                    self.get_logger().info('CMD v %.3f  w %.3f' % (self.target.linear.x, self.target.angular.z))
                else:
                    self.target = Float64()
                t = random.uniform(self.period, self.period+2)
                self.get_logger().info('New t %f.' % t)
                self.pub_signal.publish(self.target)  
                self.t_ready = Timer(t, self._ready)
                self.t_ready.start()
            elif self.shape == 'rele':
                if self.signal_type == 'pose':
                    PoseStamp = PoseStamped()
                    PoseStamp.header.frame_id = "map"
                    PoseStamp.header.stamp = self.get_clock().now().to_msg()
                    if self.signal_value == 'position.x':
                        error = -self.output.pose.position.x
                        if not self.rele:
                            if error > self.range/2:
                                PoseStamp.pose.position.x = self.range
                                self.rele = True
                                self.get_logger().info('Switch rele: True')
                            else:
                                PoseStamp.pose.position.x = -self.range
                        else:
                            if error < -self.range/2:
                                PoseStamp.pose.position.x = -self.range
                                self.rele = False
                                self.get_logger().info('Switch rele: False')
                            else: 
                                PoseStamp.pose.position.x = self.range
                        PoseStamp.pose.position.y = 0.0
                        PoseStamp.pose.position.z = 1.0
                    if self.signal_value == 'position.y':
                        error = -self.output.pose.position.y
                        if not self.rele:
                            if error > self.dseta:
                                PoseStamp.pose.position.y = self.range
                                self.rele = True
                                self.get_logger().info('Switch rele: True')
                            else:
                                PoseStamp.pose.position.y = -self.range
                        else:
                            if error < -self.dseta:
                                PoseStamp.pose.position.y = -self.range
                                self.rele = False
                                self.get_logger().info('Switch rele: False')
                            else: 
                                PoseStamp.pose.position.y = self.range
                        PoseStamp.pose.position.x = 0.0
                        PoseStamp.pose.position.z = 1.0
                    q = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
                    self.get_logger().info('Pose X %.3f  Y %.3f Z %.3f' % (PoseStamp.pose.position.x, PoseStamp.pose.position.y, PoseStamp.pose.position.z))
                    PoseStamp.pose.orientation.x = q[0]
                    PoseStamp.pose.orientation.y = q[1]
                    PoseStamp.pose.orientation.z = q[2]
                    PoseStamp.pose.orientation.w = q[3]
                    self.target = PoseStamp
                    self.pub_signal.publish(self.target)
                    self.t_ready = Timer(0.1, self._ready)
                    self.t_ready.start()
            else:
                PoseStamp = PoseStamped()
                if self.shape == 'polygon':
                    PoseStamp.pose.position.x = self.points['P'+str(self.j)]['x'] * self.range
                    PoseStamp.pose.position.y = self.points['P'+str(self.j)]['y'] * self.range
                    PoseStamp.pose.position.z = self.points['P'+str(self.j)]['z']
                    t = self.points['P'+str(self.j)]['t']
                    self.get_logger().info('P%s: X %.3f  Y %.3f' % (str(self.j),PoseStamp.pose.position.x, PoseStamp.pose.position.y))
                    self.j = ((self.j+1) % len(self.points))
                elif self.shape == 'circle':
                    time = self.get_clock().now().to_msg()
                    t = time.sec+time.nanosec*1e-9
                    PoseStamp.pose.position.x = sin((t-self.t_init)*0.2) * self.range
                    PoseStamp.pose.position.y = cos((t-self.t_init)*0.2) * self.range
                    if self.robot == 'dron':
                        PoseStamp.pose.position.z = 1.0
                    else:
                        PoseStamp.pose.position.z = 0.0
                    t = 0.1
                yaw = atan2(PoseStamp.pose.position.y-self.output.pose.position.y,PoseStamp.pose.position.x-self.output.pose.position.x)
                q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
                PoseStamp.pose.orientation.x = q[0]
                PoseStamp.pose.orientation.y = q[1]
                PoseStamp.pose.orientation.z = q[2]
                PoseStamp.pose.orientation.w = q[3]
                PoseStamp.header.frame_id = "map"
                PoseStamp.header.stamp = self.get_clock().now().to_msg()
                self.path.header.stamp = self.get_clock().now().to_msg()
                self.path.poses.append(PoseStamp)
                self.path_publisher.publish(self.path)
                self.pub_signal.publish(PoseStamp)  
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