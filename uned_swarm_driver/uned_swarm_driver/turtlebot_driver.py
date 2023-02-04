import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from math import sqrt, cos, sin, atan2
from tf_transformations import euler_from_quaternion, quaternion_from_euler


#############################
##  Turtlebot3 Aux. Class  ##
#############################
class TurtlebotDriver(Node):
    def __init__(self):
        super().__init__('turtlebot3_driver')
        # Params
        self.declare_parameter('id', 'turtlebot01')
        self.declare_parameter('config', 'file_path.yaml')
        # Services

        # Publisher
        self.publisher_status_ = self.create_publisher(String,'status', 10)
        self.publisher_pose_ = self.create_publisher(Pose,'/turtlebot01/local_pose', 10)
        self.publisher_twist_ = self.create_publisher(Twist,'cmd_vel', 10)
        # Subscription
        self.sub_order_ = self.create_subscription(String, 'order', self.order_callback, 10)
        self.sub_goal_pose_ = self.create_subscription(Pose, 'goal_pose', self.goalpose_callback, 10)
        self.sub_pose_ = self.create_subscription(Pose, '/turtlebot01/pose', self.pose_callback, 1)
        self.sub_odom_ = self.create_subscription(Odometry, 'odom', self.odom_callback, 1)
        # 
        self.initialize()

        self.timer_iterate = self.create_timer(1.0, self.iterate)

    def initialize(self):
        self.get_logger().info('TurtlebotDriver::inicialize() ok.')
        # Read Params
        self.id = self.get_parameter('id').get_parameter_value().string_value
        config_file = self.get_parameter('config').get_parameter_value().string_value

        with open(config_file, 'r') as file:
            documents = yaml.safe_load(file)
            
        config = documents[self.id]
        self.theta = config['init_theta']
        self.theta_init = self.theta
        self.yaw_init = 0.0

        # Variables
        self.goal_pose = Pose()
        self.pose = Pose()
        self.init_pose = False
        self.init_odom = False
        self.pose_offset = Pose()

    def order_callback(self, msg):
        self.get_logger().info('TurtlebotDriver::Order: "%s"' % msg.data)

    def goalpose_callback(self, msg):
        self.get_logger().info('New Goal pose: X: %.2f Y: %.2f' % (msg.position.x, msg.position.y))
        self.goal_pose = msg

    def pose_callback(self, msg):
        if self.init_odom or True:
            self.pose = msg
            self.publisher_pose_.publish(self.pose)
            # self.pose.orientation = quaternion_from_euler(0.0,0.0, self.theta)
            if not self.init_pose:
                self.init_pose = True
                self.goal_pose = self.pose

    def odom_callback(self, msg):
        if self.init_pose:
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - (q.y * q.y + q.z * q.z)
            yaw = atan2(siny_cosp, cosy_cosp)
            # self.get_logger().info('Yaw: %.3f' % (yaw))
            self.theta = (self.theta_init + (yaw-self.yaw_init))*2
            # self.pose.position.x = -(msg.pose.pose.position.y-self.pose_offset.position.y)
            # self.pose.position.y = -(msg.pose.pose.position.x-self.pose_offset.position.x)
            # self.publisher_pose_.publish(self.pose)
        else:
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - (q.y * q.y + q.z * q.z)
            self.yaw_init = atan2(siny_cosp, cosy_cosp)
            self.init_odom = True
            

    def iterate(self):
        if self.init_pose:
            cmd_vel = Twist()
            cmd_vel = self.position_controller()
            self.get_logger().debug('Local pose: X: %.2f Y: %.2f, Yaw: %.3f' % (self.pose.position.x, self.pose.position.y, self.theta))
            self.get_logger().debug('CMD Vx: %.3f W: %.3f' % (cmd_vel.linear.x, cmd_vel.angular.z))
            self.publisher_twist_.publish(cmd_vel)

    def position_controller(self):
        Kp = 1
        Kw = 0.5
        Vmax = 0.2
        Wmax = 0.5
        out = Twist()
        d = sqrt(pow(self.goal_pose.position.x-self.pose.position.x,2)+pow(self.goal_pose.position.y-self.pose.position.y,2))
        if abs(d)>0.07:
            yaw = self.theta
            # Attractive force
            error_x = (self.goal_pose.position.x-self.pose.position.x)*cos(yaw)+(self.goal_pose.position.y-self.pose.position.y)*sin(yaw)
            error_y = -(self.goal_pose.position.x-self.pose.position.x)*sin(yaw)+(self.goal_pose.position.y-self.pose.position.y)*cos(yaw)

            alfa = atan2(self.goal_pose.position.y-self.pose.position.y,self.goal_pose.position.x-self.pose.position.x)

            w = Kw * sin(alfa-self.theta) # sin(oc)
            
            v = error_x * Kp * Vmax

            if abs(v) > Vmax:
                if v > Vmax:
                    v = Vmax
                else:
                    v = -Vmax

            ## Cmd_Vel
            
            out.linear.x = v
            out.angular.z = w
        else: 
            out.linear.x = 0.0
            out.angular.z = 0.0
        return out 

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_driver = TurtlebotDriver()
    rclpy.spin(turtlebot3_driver)

    turtlebot3_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

