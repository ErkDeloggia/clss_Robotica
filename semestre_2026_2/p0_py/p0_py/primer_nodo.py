#!urs/bin/env python3

import rclpy
from rclpy.node import  Node
from std_msgs.msg import Int64

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.number_ = 2
        self.publicador_ = self.create_publisher(
            msg_type = Int64,
              topic ="topic_1",
                qos_profile= 10)
        self.timer_ = self.create_timer(
            timer_period_sec= 1.0, callback= self.cbck)
        self.get_logger().info("Activate node")
        
    def cbck(self):
        msg = Int64()
        msg.data = self.number_
        self.publicador_.publish(msg)

def main(args= None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

