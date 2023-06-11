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


class Neighbour():
    def __init__(self, parent, id, d = None):
        self.id = id
        self.parent = parent
        self.d = d
        self.pose = Pose()
        self.disable = False
        if self.id == 'origin':
            self.pose.position.x = 0.0
            self.pose.position.y = 0.0
            self.pose.position.z = 0.0
        else:
            self.sub_pose_ = self.parent.parent.create_subscription(Pose, '/' + self.id + '/local_pose', self.gtpose_callback, 10)
        self.publisher_data_ = self.parent.parent.create_publisher(Float64, self.parent.id + '/' + self.id + '/data', 10)
        self.publisher_marker_ = self.parent.parent.create_publisher(Marker, self.parent.id + '/' + self.id + '/marker', 10)

    def clear_neighbour(self):
        self.disable = True
        # self.parent.parent.destroy_publisher(self.publisher_data_)
        # self.parent.parent.destroy_publisher(self.publisher_marker_)
        # if not self.id == 'origin':
        #     self.parent.parent.destroy_subscription(self.sub_pose_)

    def gtpose_callback(self, msg):
        if not self.disable:
            self.pose = msg

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
            line.header.stamp = self.parent.parent.get_clock().now().to_msg()
            line.id = 1
            line.type = 5
            line.action = 0
            line.scale.x = 0.01
            line.scale.y = 0.01
            line.scale.z = 0.01
            
            distance = sqrt(pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2)+pow(p0.z-p1.z,2))
            if abs(distance - self.d) > 0.05:
                line.color.r = 1.0
            else:
                if abs(distance - self.d) > 0.025:
                    line.color.r = 1.0
                    line.color.g = 0.5
                else:
                    line.color.g = 1.0

            line.color.a = 1.0
            line.points.append(p1)
            line.points.append(p0)

            self.publisher_marker_.publish(line)


class Agent():
    def __init__(self, parent, id):
        self.parent = parent
        self.id = id
        self.neighbour_list = list()
        self.pose = Pose()
        self.parent.get_logger().info('New Agent: %s' % self.id)
        self.sub_pose = self.parent.create_subscription(Pose, self.id + '/local_pose', self.gtpose_callback, 10)
        self.publisher_goalpose = self.parent.create_publisher(Pose, self.id + '/goal_pose', 10)

    def get_neighbourhood(self,config_file):
        with open(config_file, 'r') as file:
            documents = yaml.safe_load(file)
        aux = documents[self.id]['task']['relationship']
        self.relationship = aux.split(', ')
        for rel in self.relationship:
            aux = rel.split('_')
            robot = Neighbour(self, aux[0], d = float(aux[1]))
            self.parent.get_logger().info('Agent: %s. Neighbour %s ::: d= %s' % (self.id, aux[0],aux[1]))
            self.neighbour_list.append(robot)

    def gtpose_callback(self, msg):
        self.pose = msg



class CentralizedFormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')
        # Params
        self.declare_parameter('config_file', 'path')

        # Publisher
        self.publisher_status = self.create_publisher(String,'swarm/status', 10)
        # Subscription
        self.sub_order = self.create_subscription(String, 'swarm/order', self.order_callback, 10)

        self.timer_task = self.create_timer(0.1, self.iterate)
        self.initialize()

    def initialize(self):
        self.get_logger().info('Formation Controller::inicialize() ok.')
        pkg_dir = get_package_share_directory('uned_swarm_config')
        self.agent_list = list()
        self.distance_formation_bool = False

        # Read Params
        config_file = self.get_parameter('config_file').get_parameter_value().string_value

        with open(config_file) as f:
            data = yaml.load(f, Loader=SafeLoader)
            for key, robot in data.items():
                new_robot = Agent(self, robot['name'])
                individual_config_path = os.path.join(pkg_dir, 'resources', robot['config_path'])
                new_robot.get_neighbourhood(individual_config_path)
                self.agent_list.append(new_robot)
        


        self.get_logger().info('Formation Controller::inicialized.')

    def order_callback(self,msg):
        self.get_logger().info('Multi-Robot-System::Order: "%s"' % msg.data)
        if msg.data == 'distance_formation_run':
            self.distance_formation_bool = True
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
            self.get_logger().error('"%s": Unknown order' % (msg.data))

    def iterate(self):
        if self.distance_formation_bool:
            for agent in self.agent_list:
                dx = dy = dz = 0
                for neighbour in agent.neighbour_list:
                    if neighbour.id == 'origin':
                        error_r = pow(neighbour.d,2) - (pow(agent.pose.position.x,2)+pow(agent.pose.position.y,2)+pow(agent.pose.position.z,2))
                        dx += 2 * (error_r * agent.pose.position.x)
                        dy += 2 * (error_r * agent.pose.position.y)
                        dz += 2 * (error_r * agent.pose.position.z)
                    else:
                        error_x = agent.pose.position.x - neighbour.pose.position.x
                        error_y = agent.pose.position.y - neighbour.pose.position.y
                        error_z = agent.pose.position.z - neighbour.pose.position.z
                        distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2)
                        dx += (pow(neighbour.d,2) - distance) * error_x
                        dy += (pow(neighbour.d,2) - distance) * error_y
                        dz += (pow(neighbour.d,2) - distance) * error_z
                
                    msg_data = Float64()
                    msg_data.data = abs(neighbour.d - sqrt(distance))
                    neighbour.publisher_data_.publish(msg_data)

                if dx > 0.32:
                    dx = 0.32
                if dx < -0.32:
                    dx = -0.32
                if dy > 0.32:
                    dy = 0.32
                if dy < -0.32:
                    dy = -0.32
                if dz > 0.32:
                    dz = 0.32
                if dz < -0.32:
                    dz = -0.32
                
                goal = Pose()
                goal.position.x = agent.pose.position.x + dx/4
                goal.position.y = agent.pose.position.y + dy/4
                goal.position.z = agent.pose.position.z + dz/4
                self.get_logger().debug('Agent %s: X: %.2f->%.2f Y: %.2f->%.2f Z: %.2f->%.2f' % (agent.id, agent.pose.position.x, goal.position.x, agent.pose.position.y, goal.position.y, agent.pose.position.z, goal.position.z)) 
                agent.publisher_goalpose.publish(goal)
        

def main(args=None):
    rclpy.init(args=args)
    swarm_driver = CentralizedFormationController()
    rclpy.spin(swarm_driver)

    swarm_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()