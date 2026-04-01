#!urs/bin/env python3

import rclpy
from rclpy.node import  Node
from std_msgs.msg import Float64
import math
import time

class NodeSubPubRads(Node):
    def __init__(self):
        super().__init__("rpm_to_rads_converter_node")

        rpm = 0

        #Suscriptor

        self.create_subscription(
            msg_type= Float64,
                    callback= self.sub_cbck,
                        topic="topic_rpm", 
                            qos_profile= 10)
        
        #Publicador
        
        self.publisher_counter_ = self.create_publisher(
            msg_type= Float64,
            topic='topic_rads',
              qos_profile= 10
        )

        self.get_logger().info("Conversor de RPM a Rad/s Activado")
        
    def sub_cbck(self, msg):
        rpm = msg.data

        #Conversion de RPM a Rad/s

        rads = rpm  * ((2 * math.pi)/60)


        new_msg = Float64()
        new_msg.data = float(rads)
        self.publisher_counter_.publish(new_msg)


def main(args= None):
    rclpy.init(args=args)
    node = NodeSubPubRads()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

