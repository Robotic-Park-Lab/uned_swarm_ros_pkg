import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from math import sqrt, cos, sin, atan2
from tf_transformations import euler_from_quaternion


#############################
##  Turtlebot3 Aux. Class  ##
#############################
class TurtlebotDriver(Node):
    def __init__(self):
        super().__init__('turtlebot3_driver')
        # Params
        self.declare_parameter('id', 'turtlebot01')
        # Services

        # Publisher
        self.publisher_status_ = self.create_publisher(String,'status', 10)
        self.publisher_pose_ = self.create_publisher(Pose,'local_pose', 10)
        self.publisher_twist_ = self.create_publisher(Twist,'cmd_vel', 10)
        # Subscription
        self.sub_order_ = self.create_subscription(String, 'order', self.order_callback, 10)
        self.sub_goal_pose_ = self.create_subscription(Pose, 'goal_pose', self.goalpose_callback, 10)
        self.sub_pose_ = self.create_subscription(Pose, 'pose', self.pose_callback, 1)
        self.sub_odom_ = self.create_subscription(Odometry, 'odom', self.odom_callback, 1)
        # 
        self.initialize()

        self.timer_iterate = self.create_timer(1.0, self.iterate)

    def initialize(self):
        self.get_logger().info('TurtlebotDriver::inicialize() ok.')
        # Read Params
        self.id = self.get_parameter('id').get_parameter_value().string_value

        # Variables
        self.goal_pose = Pose()
        self.pose = Pose()
        self.init_pose = False

    def order_callback(self, msg):
        self.get_logger().info('TurtlebotDriver::Order: "%s"' % msg.data)

    def goalpose_callback(self, msg):
        self.get_logger().info('New Goal pose: X: %.2f Y: %.2f' % (msg.position.x, msg.position.y))
        self.goal_pose = msg

    def pose_callback(self, msg):
        self.pose = msg
        if not self.init_pose:
            self.init_pose = True
            self.goal_pose = self.pose

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        self.publisher_pose_.publish(self.pose)
        if not self.init_pose:
            self.init_pose = True
            self.goal_pose = self.pose

    def iterate(self):
        if self.init_pose:
            cmd_vel = Twist()
            cmd_vel = self.position_controller()
            self.get_logger().debug('Pose Vx: %.3f W: %.3f' % (cmd_vel.linear.x, cmd_vel.angular.z))

    def position_controller(self):
        Kp = 1
        Kw = 1
        Vmax = 0.2
        Wmax = 2.5/2

        d = sqrt(pow(self.goal_pose.position.x-self.pose.position.x,2)+pow(self.goal_pose.position.y-self.pose.position.y,2))
        [roll, pitch, yaw] = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])

        # Attractive force
        error_x = (self.goal_pose.position.x-self.pose.position.x)*cos(yaw)+(self.goal_pose.position.y-self.pose.position.y)*sin(yaw)
        error_y = -(self.goal_pose.position.x-self.pose.position.x)*sin(yaw)+(self.goal_pose.position.y-self.pose.position.y)*cos(yaw)

        alfa = atan2(self.goal_pose.position.y-self.pose.position.y,self.goal_pose.position.x-self.pose.position.x)
        oc = alfa - yaw
        w = Kw * oc # sin(oc)
        
        self.get_logger().debug('alfa: %.2f theta: %.2f oc: %.2f e_x: %.3f e_y: %.3f' % (alfa, yaw, oc, error_x, error_y))
        v = error_x * Kp * Vmax

        if abs(v) > Vmax:
            if v > Vmax:
                v = Vmax
            else:
                v = -Vmax

        if abs(d)<0.01:
            v = 0.0
            w = 0.0
        ## Cmd_Vel
        out = Twist()
        out.linear.x = v
        out.angular.z = w

        return out 

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_driver = TurtlebotDriver()
    rclpy.spin(turtlebot3_driver)

    turtlebot3_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

