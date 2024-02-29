#!/usr/bin/python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from numpy import sign


class Cooperate(Node):

    def __init__(self):
        super().__init__('cooperate_node')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.lin_vel1 = 0.0  # unit: m/s
        self.lin_vel2 = 0.0

        self.d = 0           # distance between two bots

        self.scan_odom1 = Odometry
        self.scan_odom2 = Odometry

        self.x1 = 0.0
        self.x2 = 0.0
        self.y1 = 0.0
        self.y2 = 0.0

        self.init_odom_state1 = False
        self.init_odom_state2 = False


        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub1 = self.create_publisher(Twist, 'tb3_1/cmd_vel', qos)
        self.cmd_vel_pub2 = self.create_publisher(Twist, 'tb3_2/cmd_vel', qos)

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
        # Subscribe to vel of first bot
        self.cmd_sub = self.create_subscription(
            Twist,
            'tb3_1/cmd_vel',
            self.cmd_vel1_callback,
            qos_profile_sensor_data
        )

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.timer = self.create_timer(
            0.1,  # unit: s
            self.cooperate_callback
        )

        self.get_logger().info("Turtlebot3 cooperate node has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""

    def odom_callback1(self, msg):
        self.scan_odom1 = msg
        self.x1 = self.scan_odom1.pose.pose.position.x
        self.y1 = self.scan_odom2.pose.pose.position.y
        self.init_odom_state1 = True

    def odom_callback2(self, msg):
        self.scan_odom2 = msg
        self.x2 = self.scan_odom2.pose.pose.position.x
        self.y2 = self.scan_odom2.pose.pose.position.y
        self.init_odom_state2 = True


    def cooperate_callback(self):
        t1,t2 = Twist(),Twist()
        



        self.cmd_vel_pub1.publish(t1)
        self.cmd_vel_pub2.publish(t2)


def main(args=None):
    rclpy.init(args=args)
    co_op = Cooperate()
    try:
        rclpy.spin(co_op)
    except KeyboardInterrupt:
        t1,t2 = Twist(),Twist()
        t1.linear.x = 0.0
        t2.linear.x = 0.0

        co_op.cmd_vel_pub1.publish(t1)
        co_op.cmd_vel_pub2.publish(t2)

        co_op.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
