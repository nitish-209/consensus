#!/usr/bin/python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from numpy import sign


class Consensus(Node):

    def __init__(self):
        super().__init__('consensus')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.lin_vel1 = 0.0  # unit: m/s
        self.lin_vel2 = 0.0
        self.lin_vel3 = 0.0



        self.scan_odom1 = Odometry
        self.scan_odom2 = Odometry
        self.scan_odom3 = Odometry

        self.x1 = 0.0
        self.x2 = 0.0
        self.x3 = 0.0

        self.init_odom_state1 = False
        self.init_odom_state2 = False
        self.init_odom_state3 = False


        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub1 = self.create_publisher(Twist, 'tb3_1/cmd_vel', qos)
        self.cmd_vel_pub2 = self.create_publisher(Twist, 'tb3_2/cmd_vel', qos)
        self.cmd_vel_pub3 = self.create_publisher(Twist, 'tb3_3/cmd_vel', qos)

        # Initialise subscribers

        #1 Odometry Data
        self.scan_odom1 = self.create_subscription(
            Odometry,
            'tb3_1/odom',
            self.odom_callback1,
            qos_profile_sensor_data
        )
        self.scan_odom2 = self.create_subscription(
            Odometry,
            'tb3_2/odom',
            self.odom_callback2,
            qos_profile_sensor_data
        )
        self.scan_odom3 = self.create_subscription(
            Odometry,
            'tb3_3/odom',
            self.odom_callback3,
            qos_profile_sensor_data
        )

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.timer = self.create_timer(
            0.1,  # unit: s
            self.consensus_callback)

        self.get_logger().info("Turtlebot3 consensus node has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""

    def odom_callback1(self, msg):
        self.scan_odom1 = msg
        self.x1 = self.scan_odom1.pose.pose.position.x
        self.init_odom_state1 = True

    def odom_callback2(self, msg):
        self.scan_odom2 = msg
        self.x2 = self.scan_odom2.pose.pose.position.x

        self.init_odom_state2 = True

    def odom_callback3(self, msg):
        self.scan_odom3 = msg
        self.x3 = self.scan_odom3.pose.pose.position.x
        self.init_odom_state3 = True



    def consensus_callback(self):
        t1,t2,t3 = Twist(),Twist(),Twist()
        t1.linear.x = (self.x2 + self.x3 - 2*self.x1)
        t2.linear.x = (self.x1 + self.x3 - 2*self.x2)
        t3.linear.x = (self.x1 + self.x2 - 2*self.x3)
        
        if abs(t1.linear.x) > 0.20:
            t1.linear.x = sign(t1.linear.x) * 0.20
        elif abs(t1.linear.x) <0.01:
            t1.linear.x = 0.0
        if abs(t2.linear.x) > 0.22:
            t2.linear.x = sign(t2.linear.x) * 0.20
        elif abs(t2.linear.x) <0.01:
            t2.linear.x = 0.0
        if abs(t3.linear.x) > 0.22:
            t3.linear.x = sign(t3.linear.x) * 0.20
        elif abs(t3.linear.x) <0.01:
            t3.linear.x = 0.0
        print(self.x1,self.x2,self.x3)
        print(t1.linear.x,t2.linear.x,t3.linear.x)

        self.cmd_vel_pub1.publish(t1)
        self.cmd_vel_pub2.publish(t2)
        self.cmd_vel_pub3.publish(t3)



def main(args=None):
    rclpy.init(args=args)
    con = Consensus()
    try:
        rclpy.spin(con)
    except KeyboardInterrupt:
        t1,t2,t3 = Twist(),Twist(),Twist()
        t1.linear.x = 0.0
        t2.linear.x = 0.0
        t3.linear.x = 0.0

        con.cmd_vel_pub1.publish(t1)
        con.cmd_vel_pub2.publish(t2)
        con.cmd_vel_pub3.publish(t3)

        con.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
