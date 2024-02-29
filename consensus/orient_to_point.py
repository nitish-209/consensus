import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
import math
import numpy

class OrientToPoint(Node):
    def __init__(self):
        super().__init__("orient_towards_point")

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.laser_data = LaserScan
        self.odom_data = Odometry
        self.x = 0.0      #position
        self.y = 0.0      #position
        self.z = 0.0      #orientation
        self.w = 0.0      #orientation
        self.yaw_d = 0.0
        self.twist = Twist()
        self.theta = 135   # in degrees

        self.min = 0.0 # Min distance using LIDAR
        self.min_index = 0  #index at which Min distance 

        self.cmd_x = 0.0  #unit: m/s
        self.cmd_z = 0.0  #unit: rad/s

        self.init_laser_state = False
        self.init_odom_state = False


        qos = QoSProfile(depth=10)
        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        # Create Velocity Publisher
        self.vel_pub = self.create_publisher(Twist,"cmd_vel",qos)

        # Create Odometry Subscriber
        self.odom_subscriber = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            qos_profile_sensor_data
            )
        
        self.timer1 = self.create_timer(0.01,self.timer1_callback)

        self.timer2 = self.create_timer(0.5,self.angle_callback)

        self.timer3 = self.create_timer(0.01,self.point_to)

        self.vel_pub_timer = self.create_timer(0.01,self.vel_pub_callback)

    def odom_callback(self,msg):
        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.orientation.z
        self.w = msg.pose.pose.orientation.w
        self.init_odom_state = True
        #self.get_logger().info("Odom callback has started.")

    def vel_pub_callback(self):
        # Increase linear velocity in increments
        if self.twist.linear.x< self.cmd_x:
            self.twist.linear.x += 0.01
        elif self.twist.linear.x > self.cmd_x:
            self.twist.linear.x -= 0.01
        # Increase angular velocity in increments
        if self.twist.angular.z < self.cmd_z:
            self.twist.angular.z += 0.1
        elif self.twist.angular.z > self.cmd_z:
            self.twist.angular.z -= 0.1
        #Publish velocity
        self.vel_pub.publish(self.twist)

    def timer1_callback(self):
        #self.get_logger().info(f"Position from odometry x = {self.x}, y = {self.y}")
        #self.get_logger().info(f"Orientation from odometry z = {self.z}, w = {self.w}")
        siny_cosp = 2 * (self.w * self.z)
        cosy_cosp = 1 - 2 * (self.z * self.z)
        yaw_r = numpy.arctan2(siny_cosp, cosy_cosp)
        self.yaw_d = numpy.arctan2(siny_cosp, cosy_cosp) * 180/math.pi
        self.get_logger().info(f"yaw is {self.yaw_d}")

    def angle_callback(self):
        theta = numpy.arctan2(self.y,self.x) * 180/math.pi
        #print(f"Theta = {theta}")

    def point_to(self):
        e1 = self.theta - self.yaw_d
        e2 = abs(self.theta) + abs(self.yaw_d)
        if numpy.sign(self.theta) * numpy.sign(self.yaw_d) < 0 and e2>180:
            self.cmd_z = numpy.sign(self.yaw_d)*2.82*(360-e2)/180
        else:
            self.cmd_z = 2.82*(e1)/180      
        print(self.cmd_z)
        

def main(args=None):
    rclpy.init(args=args)

    # Start Node ::
    otp = OrientToPoint()

    # Spin Node untill Interrupted ::
    try:
        rclpy.spin(otp)
    except KeyboardInterrupt:
        t1 = Twist()
        t1.linear.x = 0.0
        t1.angular.z = 0.0
        otp.vel_pub.publish(t1)

        otp.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()