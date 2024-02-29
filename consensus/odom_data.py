import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data


class GetOdomData(Node):
    def __init__(self):
        super().__init__("get_odom_daat")
        
        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.odom_data1 = None
        self.odom_data2 = None
        self.count1 = 0
        self.count2 = 0

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        self.odom_subscriber = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            qos_profile_sensor_data
            )
        self.odom_subscriber2 = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback2,
            qos_profile_sensor_data
            )


    def odom_callback(self,msg):
        if not self.odom_data1:
            self.odom_data1 = msg
            self.count1 += 1
            self.get_logger().info(f"Odom subscriber1 is running {self.count1}")
            self.get_logger().info(f"Current pose x = {self.odom_data1.pose.pose.position.x} and y = {self.odom_data1.pose.pose.position.y}")

    def odom_callback2(self,msg):
        if not self.odom_data2:
            self.odom_data2 = msg
            self.count2 += 1
            self.get_logger().info(f"Odom subscriber2 is running {self.count2}")
            self.get_logger().info(f"Current pose x = {self.odom_data2.pose.pose.position.x} and y = {self.odom_data2.pose.pose.position.y}")

def main(args=None):
    rclpy.init(args=args)

    # Create Node::
    get_odom_data = GetOdomData()

    # Spin node untill interrupted and then destroy and shutdown::
    try:
        rclpy.spin(get_odom_data)
    except KeyboardInterrupt:
        get_odom_data.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()