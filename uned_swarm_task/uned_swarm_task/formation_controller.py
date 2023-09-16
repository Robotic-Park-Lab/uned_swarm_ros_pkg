import logging
import time
import os
import rclpy
import yaml
from yaml.loader import SafeLoader
from threading import Timer
import numpy as np
from math import atan2, cos, sin, sqrt
from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, UInt16, UInt16MultiArray, Float64
from geometry_msgs.msg import Pose, Twist, PointStamped, Point
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time


class Agent():
    def __init__(self, parent, id, x = None, y = None, z = None, d = None):
        self.id = id
        self.distance = False
        self.parent = parent
        self.k = 1.0
        if d == None:
            self.x = x
            self.y = y
            self.z = z
            self.parent.get_logger().info('Agent: %s' % self.str_())
        else:
            self.d = d
            self.distance = True
            self.parent.get_logger().info('Agent: %s' % self.str_distance_())
        self.pose = Pose()
        if self.id == 'origin':
            self.pose.position.x = 0.0
            self.pose.position.y = 0.0
            self.pose.position.z = 0.0
        else:
            self.sub_pose_ = self.parent.create_subscription(Pose, '/' + self.id + '/local_pose', self.gtpose_callback, 10)
        if self.parent.onboard:
            self.parent.get_logger().info('TO DO')
        self.sub_d_ = self.parent.create_subscription(Float64, '/' + self.id + '/d', self.d_callback, 10)
        self.publisher_data_ = self.parent.create_publisher(Float64, self.id + '/data', 10)
        self.publisher_marker_ = self.parent.create_publisher(Marker, self.id + '/marker', 10)

    def str_(self):
        return ('ID: ' + str(self.id) + ' X: ' + str(self.x) +
                ' Y: ' + str(self.y)+' Z: ' + str(self.z))
    
    def str_distance_(self):
        return ('ID: ' + str(self.id) + ' Distance: ' + str(self.d))

    def d_callback(self, msg):
        self.d = msg.data

    def gtpose_callback(self, msg):
        self.pose = msg
        if self.parent.onboard:
            self.parent.get_logger().debug('Update Neighbour pose')
        self.parent.get_logger().debug('Agent: X: %.2f Y: %.2f Z: %.2f' % (msg.position.x, msg.position.y, msg.position.z))
        
        line = Marker()
        p0 = Point()
        p0.x = self.parent.pose.position.x
        p0.y = self.parent.pose.position.y
        p0.z = self.parent.pose.position.z

        p1 = Point()
        p1.x = self.pose.position.x
        p1.y = self.pose.position.y
        p1.z = self.pose.position.z

        line.header.frame_id = 'map'
        line.header.stamp = self.parent.get_clock().now().to_msg()
        line.id = 1
        line.type = 5
        line.action = 0
        line.scale.x = 0.01
        line.scale.y = 0.01
        line.scale.z = 0.01
        
        if self.distance:
            distance = sqrt(pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2)+pow(p0.z-p1.z,2))
            if abs(distance - self.d) > 0.05:
                line.color.r = 1.0
            else:
                if abs(distance - self.d) > 0.025:
                    line.color.r = 1.0
                    line.color.g = 0.5
                else:
                    line.color.g = 1.0
        else:
            dx = p0.x-p1.x
            dy = p0.y-p1.y
            dz = p0.z-p1.z
            if abs(dx) > 0.05 or abs(dy) > 0.05 or abs(dz) > 0.05:
                line.color.r = 1.0
            else:
                if abs(dx) > 0.025 or abs(dy) > 0.025 or abs(dz) > 0.025:
                    line.color.r = 1.0
                    line.color.g = 0.5
                else:
                    line.color.g = 1.0
        line.color.a = 1.0
        line.points.append(p1)
        line.points.append(p0)

        self.publisher_marker_.publish(line)


class CentralizedFormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')
        # Params
        self.declare_parameter('config_file', 'file_path.yaml')
        self.declare_parameter('id', 'khepera01')
        self.declare_parameter('pkg', 'uned_swarm_config')

        # Publisher
        self.publisher_status = self.create_publisher(String,'/swarm/status', 10)
        self.publisher_order = self.create_publisher(String,'/swarm/order', 10)
        self.publisher_time = self.create_publisher(Time,'/swarm/time', 10)
        # Subscription
        self.sub_order = self.create_subscription(String, '/swarm/order', self.order_callback, 10)

        self.initialize()

    def initialize(self):
        self.get_logger().info('Formation Controller::inicialize() ok.')
        pkg = self.get_parameter('pkg').get_parameter_value().string_value
        pkg_dir = get_package_share_directory(pkg)
        self.id = self.get_parameter('id').get_parameter_value().string_value
        self.create_subscription(Pose, '/' + self.id + '/local_pose', self.pose_callback, 1)
        self.pub_goal_pose_ = self.create_publisher(Pose,'/' + self.id + '/goal_pose', 10)

        self.distance_formation_bool = False
        self.pose = Pose()
        # Read Params
        config_file = self.get_parameter('config_file').get_parameter_value().string_value

        with open(config_file, 'r') as file:
            documents = yaml.safe_load(file)
        config = documents[self.id]
        self.task = config['task']['enable']
        # Formation Init
        if self.task:
            self.get_logger().info('Task %s' % config['task']['type'])
            self.agent_list = list()
            aux = config['task']['relationship']
            self.relationship = aux.split(', ')
            self.onboard = config['task']['Onboard']
            if config['task']['type'] == 'distance':
                if self.onboard:
                    self.timer_task = self.create_timer(config['task']['T']/1000, self.task_formation_info)
                else:
                    self.timer_task = self.create_timer(config['task']['T']/1000, self.task_formation_distance)
                for rel in self.relationship:
                    aux = rel.split('_')
                    robot = Agent(self, aux[0], d = float(aux[1]))
                    self.agent_list.append(robot)

        self.get_logger().info('Formation Controller::inicialized.')

    def pose_callback(self, msg):
        self.pose = msg

    def order_callback(self,msg):
        self.get_logger().info('Multi-Robot-System::Order: "%s"' % msg.data)
        if msg.data == 'distance_formation_run':
            self.distance_formation_bool = True
            # self.t_stop = Timer(20, self.stop_dataset)
            # self.t_stop.start()
        elif msg.data == 'formation_stop':
            self.distance_formation_bool = False
        elif not msg.data.find("agent") == -1:
            aux = msg.data.split('_')
            if aux[1] == 'remove':
                for agent in self.agent_list:
                    for robot in agent.neighbour_list:
                        if robot.id == aux[2]:
                            self.get_logger().info('Multi-Robot-System::Remove Neighbour: "%s"' % aux[2])
                            robot.clear_neighbour()
                            agent.neighbour_list.remove(robot)
                            break
                for agent in self.agent_list:
                    if agent.id == aux[2]:
                        self.get_logger().info('Multi-Robot-System::Remove Agent: "%s"' % aux[2])
                        self.agent_list.remove(agent)
                        break
                self.down_signal_ = self.create_publisher(String,'/'+aux[2]+'/order', 10)
                msg = String()
                msg.data = 'land'
                self.down_signal_.publish(msg)
                # self.destroy_publisher(self.down_signal_)
        else:
            self.get_logger().debug('"%s": Unknown order' % (msg.data))

    def stop_dataset(self):
        self.distance_formation_bool = False
        msg = String()
        msg.data = 'formation_stop'
        self.publisher_order.publish(msg)
        self.get_logger().info('Multi-Robot-System::Order: "%s"' % msg.data)
        
    ###############
    #    Tasks    #
    ###############
    def task_formation_distance(self):
        if self.distance_formation_bool:
            # self.distance_formation_bool = False
            dx = dy = 0
            target_pose = Pose()
            for agent in self.agent_list:
                if agent.id == 'origin':
                    distance = sqrt((pow(self.pose.position.x,2)+pow(self.pose.position.y,2)+pow(self.pose.position.z,2)))
                    error_r = pow(agent.d,2) - (pow(self.pose.position.x,2)+pow(self.pose.position.y,2)+pow(self.pose.position.z,2))
                    dx += 2 * (error_r * self.pose.position.x)
                    dy += 2 * (error_r * self.pose.position.y)
                else:
                    error_x = self.pose.position.x - agent.pose.position.x
                    error_y = self.pose.position.y - agent.pose.position.y
                    error_z = self.pose.position.z - agent.pose.position.z
                    distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2)
                    dx += (pow(agent.d,2) - distance) * error_x
                    dy += (pow(agent.d,2) - distance) * error_y

                msg_data = Float64()
                msg_data.data = abs(agent.d - sqrt(distance))
                agent.publisher_data_.publish(msg_data)


            if dx > 4.0:
                dx = 4.0
            if dx < -4.0:
                dx = -4.0
            if dy > 4.0:
                dy = 4.0
            if dy < -4.0:
                dy = -4.0
   
            target_pose.position.x = self.pose.position.x + dx/4
            target_pose.position.y = self.pose.position.y + dy/4
            
            # self.parent.get_logger().debug('Formation: X: %.2f->%.2f Y: %.2f->%.2f Z: %.2f->%.2f' % (self.pose.position.x, target_pose.position.x, self.pose.position.y, target_pose.position.y, self.pose.position.z, target_pose.position.z)) 

            self.pub_goal_pose_.publish(target_pose)

            # self.parent.get_logger().debug('Distance: %.4f eX: %.2f eY: %.2f eZ: %.2f' % (sqrt(distance), error_x, error_y, error_z))
            # self.parent.get_logger().debug('Delta: %.4f X: %.2f Y: %.2f Z: %.2f' % (delta, dx, dy, dz))
            # self.parent.get_logger().debug('Target: X: %.2f Y: %.2f Z: %.2f' % (target_pose.position.x, target_pose.position.y, target_pose.position.z))

    def task_formation_info(self):
        print('TO-DO')

    def task_formation_pose(self):
        if self.ready:
            # TO-DO
            dx = dy = dz = 0

def main(args=None):
    rclpy.init(args=args)
    swarm_driver = CentralizedFormationController()
    rclpy.spin(swarm_driver)

    swarm_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()