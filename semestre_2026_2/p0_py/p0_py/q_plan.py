#!urs/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import time
from math import cos, sin, atan2, acos, asin, sqrt, pow


class ScaraControl(Node):
    def __init__(self):
        super().__init__('q_plan_node')

        self.pub_joint01_ = self.create_publisher(msg_type= Float64, 
                        topic='/joint1/cmd_pos',qos_profile= 10)
        
        self.pub_joint02_ = self.create_publisher(msg_type= Float64, 
                        topic='/joint2/cmd_pos',qos_profile= 10)
        
        self.pub_joint03_ = self.create_publisher(msg_type= Float64, 
                        topic='/joint3/cmd_pos',qos_profile= 10)
        
        self.timer_control_ = self.create_timer(timer_period_sec= 1.0, 
                        callback= self.cbck_scara_control)
        
        self.get_logger().info('Nodo controlador scara')


    def cbck_scara_control(self):

        # Definicion de las variables de la posicion inicial
        theta_1 = Float64()
        theta_2 = Float64()
        theta_3 = Float64()

        # Enviar comandos de la posicion inicial
        
        x_i = 0.6
        y_i = -0.5
        theta_i = 0

        x_j = 0.6
        y_j = 0.5
        theta_j = 0

        theta_1_i, theta_2_i, theta_3_i = cin_inv(x_i, y_i, theta_i)

        theta_1_j, theta_2_j, theta_3_j = cin_inv(x_j, y_j, theta_j)

        tf = 10 

        delta_t = 0

        for i in range(1, 11):
            print('Intervalo de tiempo ' + str(i))

            t_sim = delta_t/tf
            theta_1_t = theta_1_i + (10*pow(t_sim,3)-15*pow(t_sim,4)+6*pow(t_sim,5)
                                     )*(theta_1_j-theta_1_i)
            
            theta_2_t = theta_2_j + (10*pow(t_sim,3)-15*pow(t_sim,4)+6*pow(t_sim,5)
                                     )*(theta_2_j-theta_2_i)
            
            theta_3_t = theta_3_i + (10*pow(t_sim,3)-15*pow(t_sim,4)+6*pow(t_sim,5)
                                     )*(theta_3_j-theta_3_i)
            
            theta_1.data = float(theta_1_t)
            self.pub_joint01_.publish(theta_1)
            theta_2.data = float(theta_2_t)
            self.pub_joint02_.publish(theta_2)
            theta_3.data = float(theta_3_t)
            self.pub_joint03_.publish(theta_3)

            delta_t = delta_t + 1
            time.sleep(0.5)


def cin_inv(x_in, y_in, theta_in):
    #Parametros del robot
    L_1 = 0.5
    L_2 = 0.5
    L_3 = 0.3

    x_3_in = x_in - L_3*cos(theta_in)
    y_3_in = y_in - L_3*sin(theta_in)

    theta_2_in = acos((pow(x_3_in,2)+ pow(y_3_in,2)-pow(L_1,2)-pow(L_2,2))/(2*L_1*L_2))

    beta = atan2(y_3_in, x_3_in)

    alpha = acos((pow(x_3_in,2)+pow(y_3_in,2)+pow(L_1,2)-pow(L_2,2))/(2*L_1*sqrt(pow(x_3_in,2)+pow(y_3_in,2))))

    theta_1_in = beta -alpha

    theta_3_in = theta_in -theta_1_in -theta_2_in

    return theta_1_in,theta_2_in,theta_3_in


def main(args= None):
    rclpy.init(args=args)
    node = ScaraControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()