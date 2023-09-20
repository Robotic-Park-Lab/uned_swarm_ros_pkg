import rclpy
import yaml
from yaml.loader import SafeLoader
from threading import Timer
import numpy as np
from math import atan2, sqrt

from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Pose, PoseStamped, Point
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
    def __init__(self, parent, id, x = None, y = None, z = None, d = None):
        self.id = id
        self.parent = parent
        if d == None:
            self.distance_bool = False
            self.x = x
            self.y = y
            self.z = z
        else:
            self.distance_bool = True
            self.d = d
        self.pose = Pose()
        self.disable = False
        if self.id == 'origin':
            self.pose.position.x = 0.0
            self.pose.position.y = 0.0
            self.pose.position.z = 0.0
            self.k = 2.0
            self.sub_pose_ = self.parent.parent.create_subscription(PoseStamped, '/' + self.id + '/local_pose', self.gtpose_callback, 10)
        else:
            self.k = 1.0
            self.sub_pose_ = self.parent.parent.create_subscription(PoseStamped, '/' + self.id + '/local_pose', self.gtpose_callback, 10)
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
            
            distance = sqrt(pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2)+pow(p0.z-p1.z,2))
            if self.distance_bool:
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
        self.distance_formation_bool = False
        self.pose_formation_bool = False

    def get_neighbourhood(self,documents):
        self.controller = documents['controller']
        if self.controller == 'pid':
            k = 0.5
            if not self.id.find("dron") == -1:
                self.x_controller = PIDController(k, 0.0, 0.0, 0.0, 100, 0.1, -0.1, 0.01, 0.01)
                self.y_controller = PIDController(k, 0.0, 0.0, 0.0, 100, 0.1, -0.1, 0.01, 0.01)
                self.z_controller = PIDController(k, 0.0, 0.0, 0.0, 100, 0.1, -0.1, 0.01, 0.01)
            else:
                self.x_controller = PIDController(k, 0.0, 0.0, 0.0, 100, 1.0, -1.0, 0.1, 0.01)
                self.y_controller = PIDController(k, 0.0, 0.0, 0.0, 100, 1.0, -1.0, 0.1, 0.01)
                self.z_controller = PIDController(0.0, 0.0, 0.0, 0.0, 100, 1.0, -1.0, 0.1, 0.01)
        aux = documents['relationship']
        self.relationship = aux.split(', ')
        if documents['type'] == 'distance':
                self.distance_formation_bool = True
                for rel in self.relationship:
                    aux = rel.split('_')
                    robot = Neighbour(self, aux[0], d = float(aux[1]))
                    self.parent.get_logger().info('Agent: %s. Neighbour %s ::: d: %s' % (self.id, aux[0],aux[1]))
                    self.neighbour_list.append(robot)
        elif documents['type'] == 'pose':
                self.pose_formation_bool = True
                for rel in self.relationship:
                    aux = rel.split('_')
                    robot = Neighbour(self, aux[0], x = float(aux[1]), y = float(aux[2]), z = float(aux[3]))
                    self.parent.get_logger().info('Agent: %s. Neighbour %s :::x: %s \ty: %s \tz: %s' % (self.id, aux[0], aux[1], aux[2], aux[3]))
                    self.neighbour_list.append(robot)

    def gtpose_callback(self, msg):
        self.pose = msg.pose

    def distance_gradient_controller(self):
        dx = dy = dz = 0
        for neighbour in self.neighbour_list:
            error_x = self.pose.position.x - neighbour.pose.position.x
            error_y = self.pose.position.y - neighbour.pose.position.y
            error_z = self.pose.position.z - neighbour.pose.position.z
            distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2)
            d = sqrt(distance)
            dx += neighbour.k * (pow(neighbour.d,2) - distance) * error_x/d
            dy += neighbour.k * (pow(neighbour.d,2) - distance) * error_y/d
            dz += neighbour.k * (pow(neighbour.d,2) - distance) * error_z/d
                    
            msg_data = Float64()
            msg_data.data = abs(neighbour.d - sqrt(distance))
            neighbour.publisher_data_.publish(msg_data)
        
        goal = Pose()
        if not self.id.find("dron") == -1:
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
            goal.position.x = self.pose.position.x + dx/4
            goal.position.y = self.pose.position.y + dy/4
            goal.position.z = self.pose.position.z + dz/4
            if goal.position.z < 0.6:
                goal.position.z = 0.6
            elif goal.position.z > 2.0:
                goal.position.z = 2.0
        else:
            if dx > 4:
                dx = 4
            if dx < -4:
                dx = -4
            if dy > 4:
                dy = 4
            if dy < -4:
                dy = -4
            goal.position.x = self.pose.position.x + dx/4
            goal.position.y = self.pose.position.y + dy/4
        
        return goal
    
    def distance_pid_controller(self):
        ex = ey = ez = 0
        for neighbour in self.neighbour_list:
            error_x = self.pose.position.x - neighbour.pose.position.x
            error_y = self.pose.position.y - neighbour.pose.position.y
            error_z = self.pose.position.z - neighbour.pose.position.z
            distance = sqrt(pow(error_x,2)+pow(error_y,2)+pow(error_z,2))
            e = neighbour.d - distance
            ex += neighbour.k * e * (error_x/distance)
            ey += neighbour.k * e * (error_y/distance)
            ez += neighbour.k * e * (error_z/distance)
                    
            msg_data = Float64()
            msg_data.data = e
            neighbour.publisher_data_.publish(msg_data)

        aux = self.parent.get_clock().now().to_msg()
        time = aux.sec + aux.nanosec*1e-9
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

        goal = Pose()
        goal.position.x = self.pose.position.x + dx
        goal.position.y = self.pose.position.y + dy
        goal.position.z = self.pose.position.z + dz
        if not self.id.find("dron") == -1:
            if goal.position.z < 0.6:
                goal.position.z = 0.6
            elif goal.position.z > 2.0:
                goal.position.z = 2.0
        else:
            goal.position.z = 0.0
        
        return goal

    def pose_gradient_controller(self):
        dx = dy = dz = 0
        for neighbour in self.neighbour_list:
            error_x = self.pose.position.x - neighbour.pose.position.x
            error_y = self.pose.position.y - neighbour.pose.position.y
            error_z = self.pose.position.z - neighbour.pose.position.z
            dx += neighbour.k * (pow(neighbour.x,2) - pow(error_x,2)) * error_x
            dy += neighbour.k * (pow(neighbour.y,2) - pow(error_y,2)) * error_y
            dz += neighbour.k * (pow(neighbour.z,2) - pow(error_z,2)) * error_z
                    
            msg_data = Float64()
            msg_data.data = sqrt(pow(neighbour.x-error_x,2) + pow(error_y,2) + pow(error_z,2))
            neighbour.publisher_data_.publish(msg_data)
        
        goal = Pose()
        if not self.id.find("dron") == -1:
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
            goal.position.x = self.pose.position.x + dx/4
            goal.position.y = self.pose.position.y + dy/4
            goal.position.z = self.pose.position.z + dz/4
            if goal.position.z < 0.6:
                goal.position.z = 0.6
            elif goal.position.z > 2.0:
                goal.position.z = 2.0
        else:
            if dx > 4:
                dx = 4
            if dx < -4:
                dx = -4
            if dy > 4:
                dy = 4
            if dy < -4:
                dy = -4
            goal.position.x = self.pose.position.x + dx/4
            goal.position.y = self.pose.position.y + dy/4
        
        return goal
    
    def pose_pid_controller(self):
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

        goal = Pose()
        goal.position.x = self.pose.position.x + dx
        goal.position.y = self.pose.position.y + dy
        goal.position.z = self.pose.position.z + dz
        if not self.id.find("dron") == -1:
            if goal.position.z < 0.6:
                goal.position.z = 0.6
            elif goal.position.z > 2.0:
                goal.position.z = 2.0
        else:
            goal.position.z = 0.0
            if sqrt(pow(dx,2)+pow(dy,2))<0.04:
                goal.position.x = self.pose.position.x
                goal.position.y = self.pose.position.y
        
        return goal

class CentralizedFormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')
        # Params
        self.declare_parameter('config_file', 'path')
        self.declare_parameter('controller', 'gradient')

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
        self.controller = self.get_parameter('controller').get_parameter_value().string_value
        config_file = self.get_parameter('config_file').get_parameter_value().string_value

        with open(config_file) as f:
            data = yaml.load(f, Loader=SafeLoader)
            for key, robot in data.items():
                new_robot = Agent(self, robot['name'])
                new_robot.get_neighbourhood(robot['task'])
                self.agent_list.append(new_robot)

        self.timer_task = self.create_timer(0.1, self.iterate)

        self.get_logger().info('Formation Controller::inicialized.')

    def order_callback(self,msg):
        self.get_logger().info('Multi-Robot-System::Order: "%s"' % msg.data)
        if msg.data == 'formation_run':
            self.formation_bool = True
            # self.t_stop = Timer(20, self.stop_dataset)
            # self.t_stop.start()
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

    def stop_dataset(self):
        self.formation_bool = False
        msg = String()
        msg.data = 'formation_stop'
        self.publisher_order.publish(msg)
        self.get_logger().info('Multi-Robot-System::Order: "%s"' % msg.data)

    def iterate(self):
        msg = self.get_clock().now().to_msg()
        self.publisher_time.publish(msg)
        if self.formation_bool:
            for agent in self.agent_list:
                cmd = Pose()
                if agent.distance_formation_bool:
                    if agent.controller == 'gradient':
                        cmd = agent.distance_gradient_controller()
                    if agent.controller == 'pid':
                        cmd = agent.distance_pid_controller()
                elif agent.pose_formation_bool:
                    if agent.controller == 'gradient':
                        cmd = agent.pose_gradient_controller()
                    if agent.controller == 'pid':
                        cmd = agent.pose_pid_controller()

                self.get_logger().debug('Agent %s: X: %.2f->%.2f Y: %.2f->%.2f Z: %.2f->%.2f' % (agent.id, agent.pose.position.x, cmd.position.x, agent.pose.position.y, cmd.position.y, agent.pose.position.z, cmd.position.z)) 
                PoseStamp = PoseStamped()
                PoseStamp.header.frame_id = "map"
                PoseStamp.pose.position.x = cmd.position.x
                PoseStamp.pose.position.y = cmd.position.y
                PoseStamp.pose.position.z = cmd.position.z
                delta=sqrt(pow(cmd.position.x-agent.pose.position.x,2)+pow(cmd.position.y-agent.pose.position.y,2)+pow(cmd.position.z-agent.pose.position.z,2))
                angles = tf_transformations.euler_from_quaternion((agent.pose.orientation.x, agent.pose.orientation.y, agent.pose.orientation.z, agent.pose.orientation.w))
                
                if delta<0.1:
                    yaw = angles[2]
                else:
                    yaw = atan2(PoseStamp.pose.position.y-agent.pose.position.y,PoseStamp.pose.position.x-agent.pose.position.x)
                if not agent.id.find("dron01") == -1:
                    yaw = 0
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