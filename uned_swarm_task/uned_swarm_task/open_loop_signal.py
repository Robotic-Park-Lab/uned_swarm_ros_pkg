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
        self.declare_parameter('file', 'path')

        # Publisher
        self.publisher_status = self.create_publisher(String,'open_loop/status', 10)
        self.publisher_ident = self.create_publisher(String,'ident/order', 10)
        self.path_publisher = self.create_publisher(Path, 'ident/path', 10)
        
        # Subscription
        self.sub_order = self.create_subscription(String, 'open_loop/order', self.order_callback, 10)

        self.initialize()

    def initialize(self):
        self.get_logger().info('Open Loop::inicialize() ok.')
        config_file = self.get_parameter('file').get_parameter_value().string_value
        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)

        self.robot = self.config['config']['robot']
        output = self.config['config']['output']
        self.output_type = self.config['config']['output_type']
        self.output_field = self.config['config']['output_field']
        
        
        if self.output_type == 'PoseStamped':
            self.target = PoseStamped()
            self.target.header.frame_id = "map"
            self.pub_signal = self.create_publisher(PoseStamped, output, 10)
        elif self.output_type == 'Twist':
            self.target = Twist()
            self.pub_signal = self.create_publisher(Twist, output, 10)
        else:
            self.pub_signal = self.create_publisher(Float64, output, 10)
        
        signal = self.config['config']['input']
        self.input_type = self.config['config']['output_type']
        self.input_field = self.config['config']['output_field']
        
        if self.input_type == 'PoseStamped':
            self.input = PoseStamped()
            self.sub_signal = self.create_subscription(PoseStamped, signal, self.input_callback, 10)
        elif self.input_type == 'Twist':
            self.input = Twist()
            self.sub_signal = self.create_subscription(Twist, signal, self.input_callback, 10)
        else:
            self.input = Float64()
            self.sub_signal = self.create_subscription(Float64, signal, self.input_callback, 10)

        self.period = self.config['config']['period']
        self.range = self.config['config']['range']
        self.shape = self.config['config']['shape']
        self.repeat = self.config['config']['repeat']
        self.last_point = 0
        self.points = self.config['points']

        self.new = False
        # Relé
        self.dseta = 0.05
        self.rele = False

        self.non_stop= True
        time = self.get_clock().now().to_msg()
        self.t_init = time.sec+time.nanosec*1e-9
        
        self.timer_task = self.create_timer(self.period, self.iterate)
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
        

    def input_callback(self, msg):
        self.input = msg

    def iterate(self):
        if self.new:
            self.new = False
            if self.shape == 'random':
                if self.signal_type == 'pose':
                    PoseStamp = PoseStamped()
                    PoseStamp.header.stamp = self.get_clock().now().to_msg()
                    if self.signal_field == 'position.x':
                        PoseStamp.pose.position.x = random.uniform(-self.range, self.range)
                        PoseStamp.pose.position.y = self.output.pose.position.y
                        PoseStamp.pose.position.z = self.output.pose.position.z
                    if self.signal_field == 'position.y':
                        PoseStamp.pose.position.x = self.output.pose.position.x
                        PoseStamp.pose.position.y = random.uniform(-self.range, self.range)
                        PoseStamp.pose.position.z = self.output.pose.position.z
                    if self.signal_field == 'position.z':
                        PoseStamp.pose.position.x = self.output.pose.position.x
                        PoseStamp.pose.position.y = self.output.pose.position.y
                        if self.robot == 'dron':
                            PoseStamp.pose.position.z = random.uniform(1.0-self.range, 1.0+self.range)
                        else:
                            PoseStamp.pose.position.z = 0.0
                    if self.signal_field == 'full':
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
                    if self.signal_field == 'linear.x' or self.signal_field == 'full':
                        self.target.linear.x = random.uniform(0.0, self.range)
                    if self.signal_field == 'linear.y' or self.signal_field == 'full':
                        self.target.linear.y = random.uniform(-self.range, self.range)
                    if self.signal_field == 'linear.z' or self.signal_field == 'full':
                        self.target.linear.z = random.uniform(-self.range, self.range)
                    if self.signal_field == 'angular.z' or self.signal_field == 'full':
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
                    if self.signal_field == 'position.x':
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
                    if self.signal_field == 'position.y':
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
                    PoseStamp.pose.position.x = self.points['P'+str(self.last_point)]['x'] * self.range
                    PoseStamp.pose.position.y = self.points['P'+str(self.last_point)]['y'] * self.range
                    PoseStamp.pose.position.z = self.points['P'+str(self.last_point)]['z']
                    t = self.points['P'+str(self.last_point)]['t']
                    self.get_logger().info('P%s: X %.3f  Y %.3f' % (str(self.last_point),PoseStamp.pose.position.x, PoseStamp.pose.position.y))
                    self.last_point = ((self.last_point+1) % len(self.points))
                    if self.last_point == len(self.points):
                        self.repeat = self.repeat - 1
                        self.get_logger().warn('STOPPED:-1 : %d' % self.repeat)
                elif self.shape == 'circle':
                    time = self.get_clock().now().to_msg()
                    t = time.sec+time.nanosec*1e-9
                    PoseStamp.pose.position.x = self.points['center']['x'] + sin((t-self.t_init)*0.4) * self.range
                    PoseStamp.pose.position.y = self.points['center']['y'] + cos((t-self.t_init)*0.4) * self.range
                    PoseStamp.pose.position.z = self.points['center']['z']
                    t = self.period
                    self.last_point = self.last_point + 1
                    if self.points['center']['t'] < self.last_point*self.period:
                        self.last_point = 0
                        self.repeat = self.repeat - 1
                        self.get_logger().warn('STOPPED:-1 : %d' % self.repeat)

                yaw = atan2(PoseStamp.pose.position.y-self.input.pose.position.y,PoseStamp.pose.position.x-self.input.pose.position.x)
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
                if self.repeat>0:
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