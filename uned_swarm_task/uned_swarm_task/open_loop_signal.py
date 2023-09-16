import logging
import time
import os
import rclpy
import yaml
from yaml.loader import SafeLoader
from threading import Timer
import numpy as np
import random
from math import atan2, cos, sin, sqrt

from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, UInt16, UInt16MultiArray, Float64
from geometry_msgs.msg import Pose, Twist, PointStamped, Point
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time



class OpenLoop(Node):
    def __init__(self):
        super().__init__('formation_controller')
        # Params
        self.declare_parameter('range', 2.0)
        self.declare_parameter('mPeriod', 1.0)
        self.declare_parameter('signal', 'cmd_vel')
        self.declare_parameter('signal_type', 'twist')

        # Publisher
        self.publisher_status = self.create_publisher(String,'open_loop/status', 10)
        self.publisher_ident = self.create_publisher(String,'ident/order', 10)
        
        # Subscription
        self.sub_order = self.create_subscription(String, 'open_loop/order', self.order_callback, 10)

        self.initialize()

    def initialize(self):
        self.get_logger().info('Open Loop::inicialize() ok.')
        signal = self.get_parameter('signal').get_parameter_value().string_value
        self.signal_type = self.get_parameter('signal_type').get_parameter_value().string_value
        if self.signal_type == 'pose':
            self.pub_signal = self.create_publisher(Pose, signal, 10)
        elif self.signal_type == 'twist':
            self.pub_signal = self.create_publisher(Twist, signal, 10)
        else:
            self.pub_signal = self.create_publisher(Float64, signal, 10)
        
        self.period = self.get_parameter('mPeriod').get_parameter_value().double_value
        self.range = self.get_parameter('range').get_parameter_value().double_value

        self.new = False
        self.timer_task = self.create_timer(0.1, self.iterate)

        self.t_ready = Timer(self.period, self._ready)
        self.t_ready.start()

        self.get_logger().info('Open Loop::inicialized.')

    def _ready(self):
        msg = String()
        msg.data = 'ident_start'
        self.publisher_ident.publish(msg)
        self.new = True
        
    def order_callback(self,msg):
        self.get_logger().info('Open Loop::Order: "%s"' % msg.data)

    def iterate(self):
        if self.new:
            self.new = False
            if self.signal_type == 'pose':
                command = Pose()
                command.position.x = random.uniform(-self.range, self.range)
                command.position.y = random.uniform(-self.range, self.range)
                self.get_logger().info('Open Loop::Pose X %.3f  Y %.3f' % (command.position.x, command.position.y))
            elif self.signal_type == 'twist':
                command = Twist()
                command.linear.x = random.uniform(0.0, self.range)
                command.angular.z = random.uniform(-self.range/10, self.range/10)
                self.get_logger().info('Open Loop::CMD v %.3f  w %.3f' % (command.linear.x, command.angular.z))
            else:
                command = Float64()
            self.pub_signal.publish(command)
            t = random.uniform(self.period, self.period+2)
            self.get_logger().info('New t %f.' % t)
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