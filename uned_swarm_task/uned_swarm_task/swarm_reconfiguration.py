import logging
import time
import os
import rclpy
import yaml
import tf_transformations
from threading import Timer
import numpy as np
import matplotlib.pyplot as plt
import random
from math import atan2, cos, sin, sqrt, pi

from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, UInt16, UInt16MultiArray, Float64
from geometry_msgs.msg import Pose, Twist, PointStamped, Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from builtin_interfaces.msg import Time

from scipy.spatial import Delaunay, ConvexHull

class Edge():
    def __init__(self, id, d, head, tail, H):
        self.id = id
        self.d = d
        self.head = head
        self.tail = tail
        self.H = H
        self.neighborE = list()
        self.neighborF = list()
        self.neighborM = list()

class Neighbour():
    def __init__(self, parent, id, d = None):
        self.id = id
        self.parent = parent
        self.parent.neighbours.append(self.id)
        self.d = d
        self.pub_d_ = self.parent.parent.create_publisher(Float64, self.parent.id + '/' + self.id + '/d', 10)


class Agent():
    def __init__(self, parent, id):
        self.parent = parent
        self.id = id
        self.parent.agents.append(self.id)
        self.neighbour_list = list()
        self.neighbours = []
        self.triangles = list()
        self.Enode = list()
        self.pose = Pose()
        self.parent.get_logger().info('New Agent: %s' % self.id)
        self.sub_pose = self.parent.create_subscription(PoseStamped, self.id + '/local_pose', self.pose_callback, 10)
        self.sub_order = self.parent.create_subscription(String, self.id + '/order', self.order_callback, 10)
        self.publisher_order = self.parent.create_publisher(String, self.id + '/order', 10)
        self.N = 0.0

    def get_neighbourhood(self,documents):
        aux = documents['relationship']
        self.relationship = aux.split(', ')
        if documents['type'] == 'distance':
            for rel in self.relationship:
                aux = rel.split('_')
                if not aux[0] == 'origin':
                    robot = Neighbour(self, aux[0], d = float(aux[1]))
                    self.parent.get_logger().info('Agent: %s. Neighbour %s ::: d: %s' % (self.id, aux[0],aux[1]))
                    self.N += 1.0
                    self.neighbour_list.append(robot)

    def clear_neighbourhood(self):
        self.neighbour_list.clear()

    def remove_neighbour(self, id):
        j = 0
        for neighbour in self.neighbour_list:
            if neighbour.id == id:
                self.neighbour_list.pop(j)
                self.neighbours.pop(j)
                self.N -= 1.0
            else:
                j += 1
    
    def add_neighbour(self, id, d):
        robot = Neighbour(self, id, float(d))
        self.parent.get_logger().info('Agent: %s. Neighbour %s ::: d: %s' % (self.id, id,d))
        self.N += 1.0
        self.neighbour_list.append(robot)
        msg = String()
        msg.data = 'add_'+id+'_'+d
        self.publisher_order.publish(msg)

    def pose_callback(self, msg):
        self.pose = msg.pose

    def order_callback(self, msg):
        if msg.data == 'disconnect': # ellipsoid_shield_remove_drone
            j = 0
            for agent in self.parent.agent_list:
                if agent.id == self.id:
                    self.parent.agent_list.pop(j)
                    self.parent.last_pose = agent.pose
                    idx = j
                else:
                    j += 1
            j = 0 
            for agent in self.parent.agent_list:
                l = len(self.parent.agent_list[j].triangles)
                for triangle in reversed(self.parent.agent_list[j].triangles):
                    if idx == triangle[0] or idx == triangle[1] or idx == triangle[2]:
                        self.parent.agent_list[j].triangles.pop(l-1)
                    l -= 1
                j += 1

        elif not msg.data.find("remove") == -1: # ellipsoid_shield_remove_drone
            aux = msg.data.split('_')
            j = 0
            for agent in self.neighbour_list:
                if agent.id == aux[1]:
                    self.parent.pit_list.append(self)
                    self.neighbour_list.pop(j)
                    self.neighbours.pop(j)
                else:
                    j += 1

class SwarmReconfiguration(Node):
    def __init__(self):
        super().__init__('swarm_reconfiguration')
        # Params
        self.declare_parameter('file', 'path')

        # Publisher
        self.publisher_swarm_order = self.create_publisher(String,'swarm/order', 10)
        
        # Subscription
        self.sub_order = self.create_subscription(String, 'swarm/order', self.order_callback, 10)

        self.initialize()

    def initialize(self):
        self.get_logger().info('Swarm Reconfiguration::inicialize() ok.')
        self.agent_list = list()
        self.pit_list = list()
        self.edges_list = list()
        self.agents = []
        self.formation = False
        self.last_pose = Pose()
        config_file = self.get_parameter('file').get_parameter_value().string_value
        with open(config_file, 'r') as file:
            documents = yaml.safe_load(file)
            for robot in documents['Robots']:
                new_robot = Agent(self, documents['Robots'][robot]['name'])
                new_robot.get_neighbourhood(documents['Robots'][robot]['task'])
                self.agent_list.append(new_robot)

        self.get_logger().info('Swarm Reconfiguration::inicialized.')

    def order_callback(self,msg):
        self.get_logger().info('Swarm Reconfiguration::Order: "%s"' % msg.data)
        if msg.data == 'formation_run':
            self.formation = True
            self.run_Delaunay() # ellipsoid_shield_information
        if msg.data == 'reconfiguration':
            self.timer = Timer(2, self.reconfiguration)
            self.timer.start()
            
    def run_Delaunay(self):
        X1 = np.empty(shape=(0, 3))
        for agent in self.agent_list:
            X1 = np.append(X1, [[agent.pose.position.x, agent.pose.position.y, agent.pose.position.z]], axis=0)
        tri = Delaunay(np.array([X1[:,0],X1[:,1]]).T)

        for triangle in tri.simplices:
            aux = self.agent_list[triangle[0]]
            aux.triangles.append(np.array([triangle[0],triangle[1],triangle[2]]))
            self.agent_list[triangle[0]] = aux
            aux = self.agent_list[triangle[1]]
            aux.triangles.append(np.array([triangle[0],triangle[1],triangle[2]]))
            self.agent_list[triangle[1]] = aux
            aux = self.agent_list[triangle[2]]
            aux.triangles.append(np.array([triangle[0],triangle[1],triangle[2]]))
            
    def reconfiguration(self):
        self.get_logger().info('Swarm reconfiguration start.')
        self.get_logger().info('PIT size: %d' % len(self.pit_list))
        
        # ellipsoid_shield_select_closer
        if len(self.pit_list)>2:
            check = 0
            closer = []
            self.get_logger().info('PIT: Check closer point')
            for agent in self.pit_list:
                N = len(agent.neighbour_list)
                T = len(agent.triangles)
                if N == 2 and T == 2:
                    check = 1
                    closer = agent.id

            if not check:
                for agent in self.pit_list:
                    N = len(agent.neighbour_list)
                    T = len(agent.triangles)
                    if N == 3 and T == 4:
                        check = 1
                        closer = agent.id
            
            if not check:
                Dij = 100
                j = 0
                for agent in self.pit_list:
                    if agent.pose.position.z > 0.7:
                        delta = np.array([agent.pose.position.x-self.last_pose.position.x,agent.pose.position.y-self.last_pose.position.y,agent.pose.position.z-self.last_pose.position.z])
                        aux = np.linalg.norm(delta)
                        if aux < Dij:
                            Dij = aux
                            closer = agent.id
                            idx = j
                    j += 1
                    
            self.get_logger().info('Closer: %s idx: %d ' % (closer, idx))

        # ellipsoid_shield_close_pit
        j = 1
        if len(self.pit_list)>2:
            for agent in self.pit_list[idx].neighbour_list:
                for robot in self.pit_list:
                    if agent.id == robot.id and j:
                        N_l = agent
                        j = 0
                    if agent.id == robot.id and not j:
                        N_r = agent

            pit_order = []
            for i in range(len(self.pit_list)-1):
                self.get_logger().info('Nl: %s Nr: %s ' % (N_l.id, N_r.id))
                N_candi = []
                for agent in self.agent_list:
                    if agent.id == N_r.id:
                        for robot in agent.neighbour_list:
                            for dron in self.pit_list:
                                if robot.id == dron.id and not robot.id == N_l.id and not robot.id == closer:
                                    N_candi.append(dron.id)
                N_l = N_r
                if len(N_candi) == 1:
                    for agent in self.pit_list:
                        if agent.id in N_candi:
                            N_r = agent
                elif len(N_candi) == 2:
                    dis2 = 100
                    for j in range(1,len(N_candi)):
                        for agent in self.pit_list:
                            if agent.id == N_candi[i]:
                                if len(agent.neighbour_list)<dis2:
                                    idx = j
                    for agent in self.pit_list:
                        if agent.id in N_candi[idx]:
                            N_r = agent
            
                pit_order.append(N_candi)

                self.get_logger().debug('PIT order: %s' % pit_order)

                for agent in self.pit_list:
                    if agent.id in pit_order[-1]:
                        aux = []
                        for robot in agent.neighbour_list:
                            aux.append(robot.id)
                        if not closer in aux:
                            for aux0 in self.agent_list:
                                if aux0.id == closer:
                                    delta = np.array([agent.pose.position.x-aux0.pose.position.x,agent.pose.position.y-aux0.pose.position.y,agent.pose.position.z-aux0.pose.position.z])
                                    data = np.linalg.norm(delta)
                                    agent.add_neighbour(closer, str(data))
                                    # time.sleep(0.5)
                                    break
                    if agent.id == closer:
                        aux = []
                        for robot in agent.neighbour_list:
                            aux.append(robot.id)
                        for robot in pit_order[-1]:
                            if not robot in aux:
                                for aux0 in self.agent_list:
                                    if aux0.id == robot:
                                        delta = np.array([agent.pose.position.x-aux0.pose.position.x,agent.pose.position.y-aux0.pose.position.y,agent.pose.position.z-aux0.pose.position.z])
                                        data = np.linalg.norm(delta)
                                        agent.add_neighbour(robot, str(data))
                                        break
        
        # Update Distances Metropolis Events
        N = 0
        aux = []
        edges_added = []
        for agent in self.agent_list:
            for robot in agent.neighbour_list:
                for aux in self.agent_list:
                    if aux.id == robot.id and not (robot.id+'_'+agent.id in edges_added):
                        delta = np.array([agent.pose.position.x-aux.pose.position.x,agent.pose.position.y-aux.pose.position.y,agent.pose.position.z-aux.pose.position.z])
                        d = np.linalg.norm(delta)
                        if agent.pose.position.z < 0.7 and aux.pose.position.z < 0.7:
                            H = 1
                        else:
                            H = 0
                        new_edge = Edge(str(len(self.edges_list)+1),d,agent.id,robot.id, H)
                        self.edges_list.append(new_edge)
                        edges_added.append(agent.id+'_'+robot.id)

        self.get_logger().debug('Edges: %s' % edges_added)

        for i in range(len(self.edges_list)):
            for j in range(len(self.edges_list)):
                if (self.edges_list[i].head == self.edges_list[j].head or self.edges_list[i].head == self.edges_list[j].tail or self.edges_list[i].tail == self.edges_list[j].head or self.edges_list[i].tail == self.edges_list[j].tail) and i != j:
                    if not j in self.edges_list[i].neighborE:
                        self.edges_list[i].neighborE.append(j)
                    if self.edges_list[j].H and not j in self.edges_list[i].neighborF:
                        self.edges_list[i].neighborF.append(j)
                    elif not j in self.edges_list[i].neighborM:
                        self.edges_list[i].neighborM.append(j)
            self.get_logger().debug('Edges: E: %s F: %s M: %s' % (self.edges_list[i].neighborE, self.edges_list[i].neighborF, self.edges_list[i].neighborM))

        for agent in self.agent_list:
            for edge in self.edges_list:
                if (agent.id == edge.head or agent.id == edge.tail) and not edge.id in agent.Enode:
                    agent.Enode.append(edge.id)
        
        self.get_logger().info('Swarm reconfiguration end.')

        self.formation = False
        self.timer_task = self.create_timer(1.0, self.iterate)

    def iterate(self):
        if self.formation:
            delta = 0.2
            
            Ne = len(self.edges_list)
            for i in range(Ne):
                if not self.edges_list[i].H:
                    MovingNeighbors_ei = self.edges_list[i].neighborM
                    Nmei = len(MovingNeighbors_ei)

                    Wi = 0.0
                    D = 0.0
                    for j in range(Nmei):
                        idj = MovingNeighbors_ei[j]
                        degj = len(self.edges_list[idj].neighborM)
                        ke = 0.2
                        Wi += ke/(1+max(Nmei,degj))
                        D += (ke/(1+max(Nmei,degj)))*pow(self.edges_list[idj].d,2)
                    Wii = 1 - Wi
                    newDi =  Wii * pow(self.edges_list[i].d,2) + D
                    
                    for node in range(len(self.agent_list)):
                        if self.agent_list[node].pose.position.z>0.7 and self.edges_list[i].id in self.agent_list[node].Enode:
                            if self.agent_list[node].id == self.edges_list[i].head:
                                idj = self.edges_list[i].tail
                            else:
                                idj = self.edges_list[i].head
                            ij = self.agent_list[node].neighbours.index(idj)
                            idjx = self.agents.index(idj)
                            aux0 = np.array([self.agent_list[node].pose.position.x-self.agent_list[idjx].pose.position.x,self.agent_list[node].pose.position.y-self.agent_list[idjx].pose.position.y,self.agent_list[node].pose.position.z-self.agent_list[idjx].pose.position.z])
                            aux1 = np.linalg.norm(aux0)
                            error = self.agent_list[node].neighbour_list[ij].d-aux1
                            errorr = error/self.agent_list[node].neighbour_list[ij].d
                                
                            if abs(errorr)<delta:
                                self.agent_list[node].neighbour_list[ij].d = sqrt(newDi)
                                ix0 = self.agents.index(self.agent_list[node].neighbour_list[ij].id)
                                ix1 = self.agent_list[ix0].neighbours.index(self.agent_list[node].id)
                                self.agent_list[ix0].neighbour_list[ix1].d = sqrt(newDi)
                                self.edges_list[i].d = sqrt(newDi)
                                self.get_logger().debug('%s-%s d: %.2f ' % (self.agent_list[node].id, self.agent_list[node].neighbour_list[ij].id, self.edges_list[i].d))
                                

            msg = Float64()
            for agent in self.agent_list:
                for neighbour in agent.neighbour_list:
                    msg.data = neighbour.d
                    neighbour.pub_d_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    reconfiguration_node = SwarmReconfiguration()
    rclpy.spin(reconfiguration_node)

    reconfiguration_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()