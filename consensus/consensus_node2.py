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
        self.odom_data1,self.odom_data2,self.odom_data3,self.odom_data4,self.odom_data5 = Odometry,Odometry,Odometry,Odometry,Odometry
        self.x1, self.x2, self.x3, self.x4, self.x5 = 0.0, 0.0, 0.0, 0.0, 0.0      #position
        self.y1,self.y2,self.y3,self.y4,self.y5 = 0.0, 0.0, 0.0, 0.0, 0.0       #position
        self.z1,self.z2,self.z3,self.z4,self.z5 = 0.0, 0.0, 0.0, 0.0, 0.0       #orientation
        self.w1,self.w2,self.w3,self.w4,self.w5 = 0.0, 0.0, 0.0, 0.0, 0.0       #orientation
        self.yaw_d1,self.yaw_d2,self.yaw_d3,self.yaw_d4,self.yaw_d5 = 0.0, 0.0, 0.0, 0.0, 0.0 
        self.twist1,self.twist2,self.twist3,self.twist4,self.twist5 = Twist(),Twist(),Twist(),Twist(),Twist()
        self.x_con, self.y_con = 1.5, 2.5   # x,y coordinate of Consensus Point
        self.theta1,self.theta2,self.theta3,self.theta4,self.theta5 = 0.0, 0.0, 0.0, 0.0, 0.0 

        self.min = 0.0 # Min distance using LIDAR
        self.min_index = 0  #index at which Min distance 

        self.cmd_x1,self.cmd_x2,self.cmd_x3,self.cmd_x4,self.cmd_x5 = 0.0,0.0,0.0,0.0,0.0  #unit: m/s
        self.cmd_z1,self.cmd_z2,self.cmd_z3,self.cmd_z4,self.cmd_z5 = 0.0,0.0,0.0,0.0,0.0  #unit: rad/s

        self.init_laser_state = False
        self.init_odom_state1,self.init_odom_state2,self.init_odom_state3,self.init_odom_state4,self.init_odom_state5 = False,False,False,False,False


        qos = QoSProfile(depth=10)
        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        # Create Velocity Publisher
        self.vel_pub1 = self.create_publisher(Twist,"burger1/cmd_vel",qos)
        self.vel_pub2 = self.create_publisher(Twist,"burger2/cmd_vel",qos)
        self.vel_pub3 = self.create_publisher(Twist,"burger3/cmd_vel",qos)
        self.vel_pub4 = self.create_publisher(Twist,"burger4/cmd_vel",qos)
        self.vel_pub5 = self.create_publisher(Twist,"burger5/cmd_vel",qos)

        # Create Odometry Subscriber
        self.odom_subscriber1 = self.create_subscription(
            Odometry,
            "burger1/odom",
            self.odom_callback1,
            qos_profile_sensor_data
            )
        self.odom_subscriber2 = self.create_subscription(
            Odometry,
            "burger2/odom",
            self.odom_callback2,
            qos_profile_sensor_data
            )
        self.odom_subscriber3 = self.create_subscription(
            Odometry,
            "burger3/odom",
            self.odom_callback3,
            qos_profile_sensor_data
            )
        self.odom_subscriber4 = self.create_subscription(
            Odometry,
            "burger4/odom",
            self.odom_callback4,
            qos_profile_sensor_data
            )
        self.odom_subscriber5 = self.create_subscription(
            Odometry,
            "burger5/odom",
            self.odom_callback5,
            qos_profile_sensor_data
            )
        

        self.theta_timer = self.create_timer(0.01,self.theta_callback)

        self.timer1 = self.create_timer(0.01,self.timer1_callback)

        self.timer2 = self.create_timer(0.5,self.angle_callback)

        self.timer3 = self.create_timer(0.01,self.vel_calculation)

        self.vel_pub_timer1 = self.create_timer(0.01,self.vel_pub_callback1)
        self.vel_pub_timer2 = self.create_timer(0.01,self.vel_pub_callback2)
        self.vel_pub_timer3 = self.create_timer(0.01,self.vel_pub_callback3)
        self.vel_pub_timer4 = self.create_timer(0.01,self.vel_pub_callback4)
        self.vel_pub_timer5 = self.create_timer(0.01,self.vel_pub_callback5)


        """************************************************************
        ** Callback and other Functions
        ************************************************************"""

    def odom_callback1(self,msg):
        self.odom_data1 = msg
        self.x1 = msg.pose.pose.position.x
        self.y1 = msg.pose.pose.position.y
        self.z1 = msg.pose.pose.orientation.z
        self.w1 = msg.pose.pose.orientation.w
        self.init_odom_state1 = True
        #self.get_logger().info("Odom callback has started.")

    def odom_callback2(self,msg):
        self.odom_data2 = msg
        self.x2 = msg.pose.pose.position.x
        self.y2 = msg.pose.pose.position.y
        self.z2 = msg.pose.pose.orientation.z
        self.w2 = msg.pose.pose.orientation.w
        self.init_odom_state2 = True
        #self.get_logger().info("Odom callback has started.")

    def odom_callback3(self,msg):
        self.odom_data3 = msg
        self.x3 = msg.pose.pose.position.x
        self.y3 = msg.pose.pose.position.y
        self.z3 = msg.pose.pose.orientation.z
        self.w3 = msg.pose.pose.orientation.w
        self.init_odom_state3 = True
        #self.get_logger().info("Odom callback has started.")

    def odom_callback(self,msg):
        self.odom_data4 = msg
        self.x4 = msg.pose.pose.position.x
        self.y4 = msg.pose.pose.position.y
        self.z4 = msg.pose.pose.orientation.z
        self.w4 = msg.pose.pose.orientation.w
        self.init_odom_state4 = True
        #self.get_logger().info("Odom callback has started.")

    def odom_callback5(self,msg):
        self.odom_data5 = msg
        self.x5 = msg.pose.pose.position.x
        self.y5 = msg.pose.pose.position.y
        self.z5 = msg.pose.pose.orientation.z
        self.w5 = msg.pose.pose.orientation.w
        self.init_odom_state5 = True
        #self.get_logger().info("Odom callback has started.")



    def vel_pub_callback1(self):
        # Increase linear velocity in increments
        if self.twist1.linear.x< self.cmd_x:
            self.twist1.linear.x += 0.01
        elif self.twist1.linear.x > self.cmd_x:
            self.twist1.linear.x -= 0.01
        # Increase angular velocity in increments
        if self.twist1.angular.z < self.cmd_z:
            self.twist1.angular.z += 0.1
        elif self.twist1.angular.z > self.cmd_z:
            self.twist1.angular.z -= 0.1
        #Publish velocity
        self.vel_pub1.publish(self.twist1)

    def vel_pub_callback2(self):
        # Increase linear velocity in increments
        if self.twist2.linear.x< self.cmd_x:
            self.twist2.linear.x += 0.01
        elif self.twist2.linear.x > self.cmd_x:
            self.twist2.linear.x -= 0.01
        # Increase angular velocity in increments
        if self.twist2.angular.z < self.cmd_z:
            self.twist2.angular.z += 0.1
        elif self.twist2.angular.z > self.cmd_z:
            self.twist2.angular.z -= 0.1
        #Publish velocity
        self.vel_pub2.publish(self.twist2)

    def vel_pub_callback3(self):
        # Increase linear velocity in increments
        if self.twist3.linear.x< self.cmd_x:
            self.twist3.linear.x += 0.01
        elif self.twist3.linear.x > self.cmd_x:
            self.twist3.linear.x -= 0.01
        # Increase angular velocity in increments
        if self.twist3.angular.z < self.cmd_z:
            self.twist3.angular.z += 0.1
        elif self.twist3.angular.z > self.cmd_z:
            self.twist3.angular.z -= 0.1
        #Publish velocity
        self.vel_pub3.publish(self.twist3)

    def vel_pub_callback1(self):
        # Increase linear velocity in increments
        if self.twist1.linear.x< self.cmd_x:
            self.twist1.linear.x += 0.01
        elif self.twist1.linear.x > self.cmd_x:
            self.twist1.linear.x -= 0.01
        # Increase angular velocity in increments
        if self.twist1.angular.z < self.cmd_z:
            self.twist1.angular.z += 0.1
        elif self.twist1.angular.z > self.cmd_z:
            self.twist1.angular.z -= 0.1
        #Publish velocity
        self.vel_pub1.publish(self.twist1)

    def vel_pub_callback1(self):
        # Increase linear velocity in increments
        if self.twist1.linear.x< self.cmd_x:
            self.twist1.linear.x += 0.01
        elif self.twist1.linear.x > self.cmd_x:
            self.twist1.linear.x -= 0.01
        # Increase angular velocity in increments
        if self.twist1.angular.z < self.cmd_z:
            self.twist1.angular.z += 0.1
        elif self.twist1.angular.z > self.cmd_z:
            self.twist1.angular.z -= 0.1
        #Publish velocity
        self.vel_pub1.publish(self.twist1)





    def theta_callback(self):
        self.theta = numpy.arctan2(self.y_con-self.y, self.x_con-self.x)*180/math.pi
        self.get_logger().info(f"Theta is {self.theta}")


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

    def vel_calculation(self):
        e1 = self.theta - self.yaw_d
        e2 = abs(self.theta) + abs(self.yaw_d)
        if numpy.sign(self.theta) * numpy.sign(self.yaw_d) < 0 and e2>180:
            self.cmd_z = numpy.sign(self.yaw_d)*2.82*(360-e2)/180
        else:
            self.cmd_z = 2.82*(e1)/180      
        print(self.cmd_z)

        d2 = (self.y_con-self.y)**2 + (self.x_con-self.x)**2
        print("d2",d2)
        if d2>0.01:
            self.cmd_x = 0.2
        else:
            self.cmd_x = 0.0
            self.cmd_z = 0.0
        print(self.cmd_x)
        

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