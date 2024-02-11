import rclpy
import yaml
from yaml.loader import SafeLoader
from threading import Timer
import numpy as np
from math import atan2, sqrt

from rclpy.node import Node
from std_msgs.msg import String, Float64, Bool, Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import Pose, PoseStamped, Point, Vector3
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time
import tf_transformations

class PIDController():
    def __init__(self, Kp, Ki, Kd, Td, Nd, UpperLimit, LowerLimit, ai, co):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Td = Td
        self.Nd = Nd
        self.UpperLimit = UpperLimit
        self.LowerLimit = LowerLimit
        self.integral = 0
        self.derivative = 0
        self.error = [0.0, 0.0]
        self.trigger_ai = ai
        self.trigger_co = co
        self.trigger_last_signal = 0.0
        self.noise = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.past_time = 0.0
        self.last_value = 0.0
        self.th = 0.0

    def update(self, dt):
        P = self.Kp * self.error[0]
        self.integral = self.integral + self.Ki*self.error[1]*dt
        self.derivative = (self.Td/(self.Td+self.Nd+dt))*self.derivative+(self.Kd*self.Nd/(self.Td+self.Nd*dt))*(self.error[0]-self.error[1])
        out = P + self.integral + self.derivative
        
        if not self.UpperLimit==0.0:
            # out_i = out
            if out>self.UpperLimit:
                out = self.UpperLimit
            if out<self.LowerLimit:
                out = self.LowerLimit

            # self.integral = self.integral - (out-out_i) * sqrt(self.Kp/self.Ki)
        
        self.error[1] = self.error[0]

        self.last_value = out
        
        return out

    def eval_threshold(self, signal, ref):
        # Noise (Cn)
        mean = signal/len(self.noise)
        for i in range(0,len(self.noise)-2):
            self.noise[i] = self.noise[i+1]
            mean += self.noise[i]/len(self.noise)
        
        self.noise[len(self.noise)-1] = signal

        trigger_cn = 0.0
        for i in range(0,len(self.noise)-1):
            if abs(self.noise[i]-mean) > trigger_cn:
                trigger_cn = self.noise[i]-mean
        trigger_cn = 0.0
        # a
        a = self.trigger_ai * abs(signal - ref)
        if a > self.trigger_ai:
            a = self.trigger_ai

        # Threshold
        self.th = self.trigger_co + a + trigger_cn
        self.inc = abs(abs(ref-signal) - self.trigger_last_signal) 
        # Delta Error
        if (self.inc >= abs(self.th)):
            self.trigger_last_signal = abs(ref-signal)
            return True

        return False


class Neighbour():
    def __init__(self, parent, id, x = None, y = None, z = None, d = None, point = None, vector = None):
        self.id = id
        self.distance = False
        self.parent = parent
        self.last_error = 0.0
        self.last_iae = 0.0
        self.k = 1.0
        self.pose = Pose()
        self.disable = False
        if not id.find("line") == -1:
            self.distance_bool = True
            self.d = 0
            self.point = point
            self.vector = vector
            self.mod=pow(vector.x,2)+pow(vector.y,2)+pow(vector.z,2)
            self.k = 4.0
        else:
            if d == None:
                self.distance_bool = False
                self.x = x
                self.y = y
                self.z = z
                self.parent.parent.get_logger().info('Agent: %s' % self.str_())
            else:
                self.distance_bool = True
                self.d = d
                self.parent.parent.get_logger().info('Agent: %s' % self.str_distance_())
            if self.id == 'origin':
                self.pose.position.x = 0.0
                self.pose.position.y = 0.0
                self.pose.position.z = 0.0
                self.k = 4.0
            self.sub_pose_ = self.parent.parent.create_subscription(PoseStamped, '/' + self.id + '/local_pose', self.gtpose_callback, 10)
        self.sub_d_ = self.parent.parent.create_subscription(Float64, self.parent.id + '/' + self.id + '/d', self.d_callback, 10)
        self.publisher_data_ = self.parent.parent.create_publisher(Float64, self.parent.id + '/' + self.id + '/data', 10)
        self.publisher_error_ = self.parent.parent.create_publisher(Float64, self.parent.id + '/' + self.id + '/error', 10)
        self.publisher_iae_ = self.parent.parent.create_publisher(Float64, self.parent.id + '/' + self.id + '/iae', 10)
        self.publisher_marker_ = self.parent.parent.create_publisher(Marker, self.parent.id + '/' + self.id + '/marker', 10)

    def clear_neighbour(self):
        self.disable = True
        self.parent.parent.destroy_publisher(self.publisher_data_)
        self.parent.parent.destroy_publisher(self.publisher_error_)
        self.parent.parent.destroy_publisher(self.publisher_iae_)
        self.parent.parent.destroy_publisher(self.publisher_marker_)
        self.parent.parent.destroy_subscription(self.sub_pose_)

    def str_(self):
        return ('ID: ' + str(self.id) + ' X: ' + str(self.x) +
                ' Y: ' + str(self.y)+' Z: ' + str(self.z))
    
    def str_distance_(self):
        return ('ID: ' + str(self.id) + ' Distance: ' + str(self.d))

    def d_callback(self, msg):
        self.d = msg.data

    def gtpose_callback(self, msg):
        if not self.disable:
            self.pose = msg.pose

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
            
            if self.distance_bool:
                distance = sqrt(pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2)+pow(p0.z-p1.z,2))
                e = abs(distance - self.d)
            else:
                e = sqrt(pow(self.x-(p0.x-p1.x),2)+pow(self.y-(p0.y-p1.y),2)+pow(self.z-(p0.z-p1.z),2))

            if e > 0.05:
                line.color.r = 1.0
            else:
                if e > 0.025:
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
        self.sub_pose = self.parent.create_subscription(PoseStamped, self.id + '/local_pose', self.gtpose_callback, 10)
        self.publisher_goalpose = self.parent.create_publisher(PoseStamped, self.id + '/goal_pose', 10)
        self.publisher_mrs_data = self.parent.create_publisher(Float64MultiArray, self.id + '/mr_data', 10)
        self.publisher_global_error_ = self.parent.create_publisher(Float64, self.id + '/global_error', 10)
        self.distance_formation_bool = False
        self.pose_formation_bool = False
        self.continuous = False
        self.trigger_ai = 0.01
        self.trigger_co = 0.1
        self.trigger_last_signal = 0.0
        self.N = 0.0

    def get_neighbourhood(self,documents):
        self.controller = documents['controller']
        self.controller_type = self.controller['type']
        self.k = self.controller['gain']
        self.ul = self.controller['upperLimit']
        self.ll = self.controller['lowerLimit']
        self.continuous = self.controller['protocol'] == 'Continuous'
        self.event_x = self.parent.create_publisher(Bool, self.id + '/event_x', 10)
        self.event_y = self.parent.create_publisher(Bool, self.id + '/event_y', 10)
        self.event_z = self.parent.create_publisher(Bool, self.id + '/event_z', 10)
        self.task_period = self.controller['period']
        if not self.continuous:
            self.trigger_ai = self.controller['threshold']['ai']
            self.trigger_co = self.controller['threshold']['co']
        
        if self.controller_type == 'pid':
            self.x_controller = PIDController(self.k, 0.0, 0.0, 0.0, 100, self.ul, self.ll, self.trigger_ai, self.trigger_co)
            self.y_controller = PIDController(self.k, 0.0, 0.0, 0.0, 100, self.ul, self.ll, self.trigger_ai, self.trigger_co)
            self.z_controller = PIDController(self.k, 0.0, 0.0, 0.0, 100, self.ul, self.ll, self.trigger_ai, self.trigger_co)

        aux = documents['relationship']
        self.relationship = aux.split(', ')
        if documents['type'] == 'distance':
                self.distance_formation_bool = True
                for rel in self.relationship:
                    aux = rel.split('_')
                    id = aux[0]
                    if not id.find("line") == -1:
                        p = Point()
                        p.x = float(aux[1])
                        p.y = float(aux[2])
                        p.z = float(aux[3])
                        u = Vector3()
                        u.x = float(aux[4])
                        u.y = float(aux[5])
                        u.z = float(aux[6])
                        robot = Neighbour(self, id, point = p, vector = u)
                        self.parent.get_logger().info('Agent: %s. Neighbour %s ::: Px: %s Py: %s Pz: %s' % (self.id, id, aux[1], aux[2], aux[3]))
                    else:
                        robot = Neighbour(self, id, d = float(aux[1]))
                        self.parent.get_logger().info('Agent: %s. Neighbour %s ::: d: %s' % (self.id, id,aux[1]))
                    self.N += 1.0
                    self.neighbour_list.append(robot)
        elif documents['type'] == 'pose':
                self.pose_formation_bool = True
                for rel in self.relationship:
                    aux = rel.split('_')
                    robot = Neighbour(self, aux[0], x = float(aux[1]), y = float(aux[2]), z = float(aux[3]))
                    self.parent.get_logger().info('Agent: %s. Neighbour %s :::x: %s \ty: %s \tz: %s' % (self.id, aux[0], aux[1], aux[2], aux[3]))
                    self.neighbour_list.append(robot)
                    self.N += 1.0

    def gtpose_callback(self, msg):
        self.pose = msg.pose

    def distance_gradient_controller(self):
        msg_error = Float64()
        msg_error.data = 0.0
        dx = dy = dz = 0
        for agent in self.neighbour_list:
            if not agent.id.find("line") == -1:
                nearest = PoseStamped()
                nearest.header.frame_id = "map"
                gamma = -np.dot([agent.point.x-self.pose.position.x, agent.point.y-self.pose.position.y, agent.point.z-self.pose.position.z],[agent.vector.x, agent.vector.y, agent.vector.z])/agent.mod
                nearest.pose.position.x = agent.point.x + gamma * agent.vector.x
                nearest.pose.position.y = agent.point.y + gamma * agent.vector.y
                nearest.pose.position.z = agent.point.z + gamma * agent.vector.z
                agent.gtpose_callback(nearest)

            error_x = self.pose.position.x - agent.pose.position.x
            error_y = self.pose.position.y - agent.pose.position.y
            error_z = self.pose.position.z - agent.pose.position.z
            distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2)

            dx += - self.k * agent.k * (distance - pow(agent.d,2)) * error_x
            dy += - self.k * agent.k * (distance - pow(agent.d,2)) * error_y
            dz += - self.k * agent.k * (distance - pow(agent.d,2)) * error_z
                    
            msg_data = Float64()
            msg_data.data = sqrt(distance)
            agent.publisher_data_.publish(msg_data)
            error = abs(sqrt(distance) - agent.d)
            msg_data.data = agent.last_iae + (agent.last_error + error) * self.task_period /2
            agent.last_error = error
            agent.publisher_iae_.publish(msg_data)
            agent.last_iae = msg_data.data
            msg_data.data = distance - pow(agent.d,2)
            agent.publisher_error_.publish(msg_data)
            msg_error.data += msg_data.data
            self.parent.get_logger().debug('Agent %s: D: %.2f dx: %.2f dy: %.2f dz: %.2f ' % (agent.id, msg_data.data, dx, dy, dz)) 
        
        goal = Pose()
        if not self.continuous:
            delta=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
            if not self.eval_threshold(0.0, delta):
                goal.position.z = -1000
                return goal
        
        msg = Bool()
        msg.data = True
        self.event_x.publish(msg)

        msg = Float64MultiArray()
        msg.data = [round(dx,3), round(dy,3), round(dz,3), self.N]
        msg.layout.data_offset = 0
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = 'data'
        msg.layout.dim[0].size = 4
        msg.layout.dim[0].stride = 1
        self.publisher_mrs_data.publish(msg)

        if dx > self.ul:
            dx = self.ul
        if dx < self.ll:
            dx = self.ll
        if dy > self.ul:
            dy = self.ul
        if dy < self.ll:
            dy = self.ll
        if dz > self.ul:
            dz = self.ul
        if dz < self.ll:
            dz = self.ll

        goal.position.x = dx
        goal.position.y = dy
        goal.position.z = dz

        self.publisher_global_error_.publish(msg_error)
        
        return goal
    
    def distance_pid_controller(self):
        ## TO-DO: Review
        ex = ey = ez = 0
        for neighbour in self.neighbour_list:
            if not neighbour.id.find("line") == -1:
                nearest = PoseStamped()
                nearest.header.frame_id = "map"
                gamma = -np.dot([neighbour.point.x-self.pose.position.x, neighbour.point.y-self.pose.position.y, neighbour.point.z-self.pose.position.z],[neighbour.vector.x, neighbour.vector.y, neighbour.vector.z])/neighbour.mod
                nearest.pose.position.x = neighbour.point.x + gamma * neighbour.vector.x
                nearest.pose.position.y = neighbour.point.y + gamma * neighbour.vector.y
                nearest.pose.position.z = neighbour.point.z + gamma * neighbour.vector.z
                neighbour.gtpose_callback(nearest)
            error_x = self.pose.position.x - neighbour.pose.position.x
            error_y = self.pose.position.y - neighbour.pose.position.y
            error_z = self.pose.position.z - neighbour.pose.position.z
            distance = sqrt(pow(error_x,2)+pow(error_y,2)+pow(error_z,2))
            e = neighbour.d - distance
            if distance == 0:
                distance = 1.0
            ex += neighbour.k * e * (error_x/distance)
            ey += neighbour.k * e * (error_y/distance)
            ez += neighbour.k * e * (error_z/distance)
                    
            msg_data = Float64()
            msg_data.data = e
            neighbour.publisher_data_.publish(msg_data)

        aux = self.parent.get_clock().now().to_msg()
        time = aux.sec + aux.nanosec*1e-9

        goal = Pose()
        if not (self.x_controller.eval_threshold(0.0, ex) or self.y_controller.eval_threshold(0.0, ey) or self.z_controller.eval_threshold(0.0, ez) or self.continuous):
            goal.position.z = -1000
            return goal
        
        msg = Bool()
        msg.data = True
        # X Controller
        self.x_controller.error[0] = ex
        dtx = time - self.x_controller.past_time
        dx = self.x_controller.update(dtx)
        self.x_controller.past_time = time

        # Y Controller
        self.y_controller.error[0] = ey
        dty = time - self.y_controller.past_time
        dy = self.y_controller.update(dty)
        self.y_controller.past_time = time

        # Z Controller
        self.z_controller.error[0] = ez
        dtz = time - self.z_controller.past_time
        dz = self.z_controller.update(dtz)
        self.z_controller.past_time = time

        goal.position.x = dx
        goal.position.y = dy
        goal.position.z = dz

        self.event_x.publish(msg)
        
        return goal

    def pose_gradient_controller(self):
        ## TO-DO: Review
        dx = dy = dz = 0
        for neighbour in self.neighbour_list:
            error_x = self.pose.position.x - neighbour.pose.position.x
            error_y = self.pose.position.y - neighbour.pose.position.y
            error_z = self.pose.position.z - neighbour.pose.position.z
            distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2)
            d = sqrt(distance)
            dx += self.k * neighbour.k * (pow(neighbour.x,2) - pow(error_x,2)) * error_x/d
            dy += self.k * neighbour.k * (pow(neighbour.y,2) - pow(error_y,2)) * error_y/d
            dz += self.k * neighbour.k * (pow(neighbour.z,2) - pow(error_z,2)) * error_z/d
                    
            msg_data = Float64()
            msg_data.data = sqrt(pow(neighbour.x-error_x,2) + pow(neighbour.y-error_y,2) + pow(neighbour.z-error_z,2))
            neighbour.publisher_data_.publish(msg_data)
        
        goal = Pose()
        if not self.continuous:
            delta=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
            if not self.eval_threshold(0.0, delta):
                goal.position.z = -1000
                return goal

        msg = Bool()
        msg.data = True
        self.event_x.publish(msg)

        if dx > self.ul:
            dx = self.ul
        if dx < self.ll:
            dx = self.ll
        if dy > self.ul:
            dy = self.ul
        if dy < self.ll:
            dy = self.ll
        if dz > self.ul:
            dz = self.ul
        if dz < self.ll:
            dz = self.ll
        goal.position.x = dx
        goal.position.y = dy
        goal.position.z = dz
        
        return goal
    
    def pose_pid_controller(self):
        # TO-DO: Review
        ex = ey = ez = 0
        for neighbour in self.neighbour_list:
            error_x = neighbour.x - (self.pose.position.x - neighbour.pose.position.x)
            error_y = neighbour.y - (self.pose.position.y - neighbour.pose.position.y)
            error_z = neighbour.z - (self.pose.position.z - neighbour.pose.position.z)
            ex += neighbour.k * error_x
            ey += neighbour.k * error_y
            ez += neighbour.k * error_z
                    
            msg_data = Float64()
            msg_data.data = sqrt(pow(error_x,2)+pow(error_y,2)+pow(error_z,2))
            neighbour.publisher_data_.publish(msg_data)

        aux = self.parent.get_clock().now().to_msg()
        time = aux.sec + aux.nanosec*1e-9
        goal = Pose()
        msg = Bool()
        msg.data = False

        # X Controller
        if self.x_controller.eval_threshold(0.0, ex) or self.continuous:
            self.x_controller.error[0] = ex
            dtx = time - self.x_controller.past_time
            goal.position.x = self.x_controller.update(dtx)
            self.x_controller.past_time = time
            msg.data = True
            self.event_x.publish(msg)
        else:
            goal.position.x = 0.0
        
        # Y Controller
        if self.y_controller.eval_threshold(0.0, ey) or self.continuous:
            self.y_controller.error[0] = ey
            dty = time - self.y_controller.past_time
            goal.position.y = self.y_controller.update(dty)
            self.y_controller.past_time = time
            msg.data = True
            self.event_y.publish(msg)
        else:
            goal.position.y = 0.0

        # Z Controller
        if self.z_controller.eval_threshold(0.0, ez) or self.continuous:
            self.z_controller.error[0] = ez
            dtz = time - self.z_controller.past_time
            goal.position.z = self.z_controller.update(dtz)
            self.z_controller.past_time = time
            msg.data = True
            self.event_z.publish(msg)
        else:
            goal.position.z = 0.0

        if not msg.data:
            goal.position.z = -1000
        
        return goal

    def eval_threshold(self, signal, ref):
        '''
        # Noise (Cn)
        mean = signal/len(self.noise)
        for i in range(0,len(self.noise)-2):
            self.noise[i] = self.noise[i+1]
            mean += self.noise[i]/len(self.noise)
        
        self.noise[len(self.noise)-1] = signal

        trigger_cn = 0.0
        for i in range(0,len(self.noise)-1):
            if abs(self.noise[i]-mean) > trigger_cn:
                trigger_cn = self.noise[i]-mean
        '''
        trigger_cn = 0.0

        # a
        a = self.trigger_ai * abs(signal - ref)
        if a > self.trigger_ai:
            a = self.trigger_ai

        # Threshold
        self.th = self.trigger_co + a + trigger_cn
        self.inc = abs(abs(ref-signal) - self.trigger_last_signal) 
        # Delta Error
        if (self.inc >= abs(self.th)):
            self.trigger_last_signal = abs(ref-signal)
            return True

        return False

class CentralizedFormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')
        # Params
        self.declare_parameter('config_file', 'path')

        # Publisher
        self.publisher_status = self.create_publisher(String,'swarm/status', 10)
        self.publisher_order = self.create_publisher(String,'swarm/order', 10)
        self.publisher_time = self.create_publisher(Time,'swarm/time', 10)
        # Subscription
        self.sub_order = self.create_subscription(String, 'swarm/order', self.order_callback, 10)

        self.initialize()

    def initialize(self):
        self.get_logger().info('Formation Controller::inicialize() ok.')
        self.agent_list = list()
        self.formation_bool = False

        # Read Params
        config_file = self.get_parameter('config_file').get_parameter_value().string_value

        with open(config_file, 'r') as file:
            documents = yaml.safe_load(file)
            for robot in documents['Robots']:
                new_robot = Agent(self, documents['Robots'][robot]['name'])
                new_robot.get_neighbourhood(documents['Robots'][robot]['task'])
                self.agent_list.append(new_robot)

        self.timer_task = self.create_timer(documents['Architecture']['node']['period'], self.iterate)

        self.get_logger().info('Formation Controller::inicialized.')

    def order_callback(self,msg):
        self.get_logger().info('Multi-Robot-System::Order: "%s"' % msg.data)
        if msg.data == 'formation_run':
            self.formation_bool = True
        elif msg.data == 'formation_stop':
            self.formation_bool = False
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

    def iterate(self):
        msg = self.get_clock().now().to_msg()
        self.publisher_time.publish(msg)
        if self.formation_bool:
            for agent in self.agent_list:
                cmd = Pose()
                if agent.distance_formation_bool:
                    if agent.controller_type == 'gradient':
                        cmd = agent.distance_gradient_controller()
                    if agent.controller_type == 'pid':
                        cmd = agent.distance_pid_controller()
                elif agent.pose_formation_bool:
                    if agent.controller_type == 'gradient':
                        cmd = agent.pose_gradient_controller()
                    if agent.controller_type == 'pid':
                        cmd = agent.pose_pid_controller()
                
                self.get_logger().debug('Agent %s: X: %.2f->%.2f Y: %.2f->%.2f Z: %.2f->%.2f' % (agent.id, agent.pose.position.x, cmd.position.x, agent.pose.position.y, cmd.position.y, agent.pose.position.z, cmd.position.z)) 
                PoseStamp = PoseStamped()
                PoseStamp.header.frame_id = "map"
                PoseStamp.pose.position.x = agent.pose.position.x + cmd.position.x
                PoseStamp.pose.position.y = agent.pose.position.y + cmd.position.y
                PoseStamp.pose.position.z = agent.pose.position.z + cmd.position.z

                delta=sqrt(pow(cmd.position.x,2)+pow(cmd.position.y,2)+pow(cmd.position.z,2))
                angles = tf_transformations.euler_from_quaternion((agent.pose.orientation.x, agent.pose.orientation.y, agent.pose.orientation.z, agent.pose.orientation.w))
                    
                if delta<0.05:
                    roll = angles[0]
                    pitch = angles[1]
                    yaw = angles[2]
                else:
                    h = sqrt(pow(cmd.position.x,2)+pow(cmd.position.y,2))
                    roll = 0.0
                    pitch = -atan2(cmd.position.z,h)
                    yaw = atan2(cmd.position.y,cmd.position.x)

                if not agent.id.find("dron") == -1:
                    if PoseStamp.pose.position.z < 0.6:
                        PoseStamp.pose.position.z = 0.6
                    elif PoseStamp.pose.position.z > 2.0:
                        PoseStamp.pose.position.z = 2.0
                else:
                    PoseStamp.pose.position.z = 0.0
                    roll = 0.0
                    pitch = 0.0
                   
                q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
                PoseStamp.pose.orientation.x = q[0]
                PoseStamp.pose.orientation.y = q[1]
                PoseStamp.pose.orientation.z = q[2]
                PoseStamp.pose.orientation.w = q[3]
                PoseStamp.header.stamp = self.get_clock().now().to_msg()
                agent.publisher_goalpose.publish(PoseStamp)


def main(args=None):
    rclpy.init(args=args)
    swarm_driver = CentralizedFormationController()
    rclpy.spin(swarm_driver)

    swarm_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()