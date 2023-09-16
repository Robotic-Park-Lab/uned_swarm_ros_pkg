import logging
import time
import os
import rclpy
import yaml
from yaml.loader import SafeLoader
from threading import Timer
import numpy as np
from math import atan2, cos, sin, sqrt

import matplotlib.pyplot as plt
from sysidentpy.model_structure_selection import FROLS
from sysidentpy.basis_function._basis_function import Polynomial
from sysidentpy.metrics import root_relative_squared_error
from sysidentpy.utils.generate_data import get_siso_data
from sysidentpy.utils.display_results import results
from sysidentpy.utils.plotting import plot_residues_correlation, plot_results
from sysidentpy.residues.residues_correlation import compute_residues_autocorrelation, compute_cross_correlation


from rclpy.node import Node
from std_msgs.msg import String, Float32, Float64MultiArray, UInt16, UInt16MultiArray, Float64
from geometry_msgs.msg import Pose, Twist, PointStamped, Point
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time



class SystemIdentification(Node):
    def __init__(self):
        super().__init__('formation_controller')
        # Params
        self.declare_parameter('input', 'signal0')
        self.declare_parameter('input_type', 'pose')
        self.declare_parameter('output', 'signal1')
        self.declare_parameter('output_type', 'pose')
        self.declare_parameter('delay', 0.0)
        self.declare_parameter('order','1p0z')

        # Publisher
        self.publisher_status = self.create_publisher(String,'ident/status', 10)
        
        # Subscription
        self.sub_order = self.create_subscription(String, 'ident/order', self.order_callback, 10)

        self.u_signal = np.zeros(5)
        self.y_signal = np.zeros(5)

        self.initialize()

    def initialize(self):
        self.get_logger().info('System Identification::inicialize() ok.')
        self.order = self.get_parameter('order').get_parameter_value().string_value
    
        # Read Params
        input_signal = self.get_parameter('input').get_parameter_value().string_value
        self.input_type = self.get_parameter('input_type').get_parameter_value().string_value
        if self.input_type == 'pose':
            self.sub_input = self.create_subscription(Pose, input_signal, self.input_callback, 10)
            self.u = Pose()
        elif self.input_type == 'twist':
            self.sub_input = self.create_subscription(Twist, input_signal, self.input_callback, 10)
            self.u = Twist()
        else:
            self.sub_input = self.create_subscription(Float64, input_signal, self.input_callback, 10)
            self.u = Float64()
        
        output_signal = self.get_parameter('output').get_parameter_value().string_value
        self.output_type = self.get_parameter('output_type').get_parameter_value().string_value
        if self.output_type == 'pose':
            self.sub_input = self.create_subscription(Pose, output_signal, self.output_callback, 10)
            self.y = Pose()
        elif self.output_type == 'twist':
            self.sub_input = self.create_subscription(Twist, output_signal, self.output_callback, 10)
            self.y = Twist()
        elif self.output_type == 'point':
            self.sub_input = self.create_subscription(Point, output_signal, self.output_callback, 10)
            self.y = Point()
        elif self.output_type == 'float32':
            self.sub_input = self.create_subscription(Float32, output_signal, self.output_callback, 10)
            self.y = Point()
        else:
            self.sub_input = self.create_subscription(Float64, output_signal, self.output_callback, 10)
            self.y = Float64()

        self.ident_process_bool = False
        self.timer_task = self.create_timer(0.05, self.iterate)

        # Estimator
        if self.order == '1p0z':
            self.N = 2
        elif self.order == '2p0z':
            self.N = 4
        else:
            self.N = 2
        self.m = np.zeros(self.N)
        self.m = self.m.reshape(1, -1)
        self.theta = np.zeros(self.N)
        self.theta = self.theta.reshape(-1, 1)
        self.P = np.identity(self.N)*10
        self.ek1 = 100
        self.finish = False

        # basis_function = Polynomial(degree=2)

        # self.model = FROLS(order_selection=True, n_info_values=3, extended_least_squares=False, ylag=1, xlag=1, info_criteria='aic', estimator='least_squares', basis_function=basis_function)
        
        self.get_logger().info('System Identification::inicialized.')

    def order_callback(self,msg):
        self.get_logger().info('System Identification::Order: "%s"' % msg.data)
        if msg.data == 'ident_start' and not self.finish:
            self.ident_process_bool = True
        elif msg.data == 'ident_stop':
            self.ident_process_bool = False
            self.finish = True
        else:
            self.get_logger().debug('"%s": Unknown order' % (msg.data))

    def input_callback(self,msg):
        self.u_signal = np.delete(self.u_signal, 0)
        self.u = msg
        self.u_signal = np.append(self.u_signal, self.u.linear.x)

    def output_callback(self,msg):
        self.y_signal = np.delete(self.y_signal, 0)
        self.y = msg
        if self.output_type == 'pose':
            self.y_signal = np.append(self.y_signal, self.y.position.x)
        elif self.output_type == 'twist':
            self.y_signal = np.append(self.y_signal, self.y.linear.x)
        elif self.output_type == 'point':
            self.y_signal = np.append(self.y_signal, self.y.x)
        elif self.output_type == 'float32':
            self.y_signal = np.append(self.y_signal, self.y.data*100)
        else:
            self.y_signal = np.append(self.y_signal, self.y.data*100)

        
    def iterate(self):
        if self.ident_process_bool:
            msg = self.get_clock().now().to_msg()
            ## ESTIMATION
            # First Order
            if self.N == 2:
                self.m[0,1] = self.u_signal[4]
                self.m[0,0] = np.sign(self.u_signal[4])*self.y_signal[3]
            # Second Order
            if self.N == 4:
                self.m[0,2] = self.u_signal[4]
                self.m[0,3] = self.u_signal[3]
                self.m[0,0] = np.sign(self.u_signal[3])*self.y_signal[3]
                self.m[0,1] = np.sign(self.u_signal[2])*self.y_signal[2]
            mt = self.m.reshape(-1, 1)
            K=np.matmul(self.P,mt)/(1+np.matmul(self.m,np.matmul(self.P,mt)))
            ek = self.y_signal[4]-np.matmul(self.m,self.theta)
            self.theta = self.theta+K*ek
            self.P = np.matmul((np.identity(self.N)-np.matmul(K,self.m)),self.P)
            # First Order
            if self.N == 2:
                gain = self.theta[1,0]/(1-self.theta[0,0])
                tau = -0.05*np.log(self.theta[0,0])
                self.get_logger().info('Estimation: a=%.3f \tb=%.3f \tk=%.3f \ttau=%.3f  \te=%.3f' % (self.theta[0,0], self.theta[1,0], gain, tau, ek))
            # Second Order
            if self.N == 4:
                self.get_logger().info('Estimation: a1=%.3f \ta2=%.3f \tb0=%.3f \tb1=%.3f  \te=%.3f' % (self.theta[0,0], self.theta[1,0], self.theta[2,0], self.theta[3,0], ek))
            if abs(ek)<0.08 and abs(self.ek1)<0.08 and False:
                self.ident_process_bool = False
                self.finish = True
            else:
                self.ek1 = ek
            '''
            for i=k:1:N
                % First Order
                m=[out(i-1) in(i)];
                % % Sencond Order
                % m=[out(i-1) out(i-2) in(i) in(i-1)];
                K=P*m'/(1+m*P*m'); 
                theta=theta+K*(out(i)-m*theta);
                P=(eye(length(theta))-K*m)*P;
                id(i-k+1,:)=theta';
                y1.est(i) = m * theta + y1.data(k-1);
            end
            '''

def main(args=None):
    rclpy.init(args=args)
    ident_node = SystemIdentification()
    rclpy.spin(ident_node)

    ident_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()