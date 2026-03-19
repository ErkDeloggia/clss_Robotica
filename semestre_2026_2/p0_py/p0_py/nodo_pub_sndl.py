#!urs/bin/env python3

import rclpy
from rclpy.node import  Node
from std_msgs.msg import Float64
import math
import time


class RpmNodePub(Node):
    def __init__(self):
        super().__init__("rpm_sin_publisher")

        #Publicador
        self.publicador_ = self.create_publisher(
            msg_type = Float64,
              topic ="topic_rpm",
                qos_profile= 10)
        self.timer_ = self.create_timer(
            timer_period_sec= 1.0, callback= self.cbck)

        self.get_logger().info("Nodo publicador de RPM Activado")

        #parametro de la onda senoidal
        
        self.star_time = time.time()
        self.amplitud = 1.0
        self.frequency = 0.1
        
        
    def cbck(self):
        msg = Float64()
        elapsed_time = time.time() - self.star_time


        #Funcion senoida 

        rpm_sine = self.amplitud * math.sin(2 * math.pi * self.frequency * elapsed_time)
        

        msg.data = abs(rpm_sine)
        self.publicador_.publish(msg)

        


def main(args= None):
    rclpy.init(args=args)
    node = RpmNodePub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
