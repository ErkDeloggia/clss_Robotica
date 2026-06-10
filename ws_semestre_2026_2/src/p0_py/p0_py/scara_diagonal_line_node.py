#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import time
from math import cos, sin, atan2, acos, sqrt, pow

class DiagonalTrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('scara_trajectory_planner')


        # Publicadores unificados
        self.pub_joint01 = self.create_publisher(Float64, '/joint1/cmd_pos', 10)
        self.pub_joint02 = self.create_publisher(Float64, '/joint2/cmd_pos', 10)
        self.pub_joint03 = self.create_publisher(Float64, '/joint3/cmd_pos', 10)

        self.timer_control = self.create_timer(1.0, self.cbck_scara_control)
        
        self.get_logger().info('Nodo controlador SCARA - Rango Maximo Extendidos')

    def cbck_scara_control(self):
        # 1. LÍMITES EXTENDIDOS DE LA TRAYECTORIA DIAGONAL
        x_in = 0.3        
        y_in = 0.3        
        theta_in = 0        

        x_fin = 0.8       # Rango ampliado
        y_fin = 0.8       
        theta_fin = 0     

        tf = 10
        delta_t = 0

        # Bucle de simulación paso a paso
        for k in range(1, 11):
            print('Intervalo de tiempo ' + str(k))

            t_sim = delta_t / tf

            # 2. EVALUACIÓN DEL POLINOMIO DE 5.° GRADO
            s = 10*pow(t_sim, 3) - 15*pow(t_sim, 4) + 6*pow(t_sim, 5)

            # 3. INTERPOLACIÓN LINEAL EN EL ESPACIO CARTESIANO
            x_t = x_in + s * (x_fin - x_in)
            y_t = y_in + s * (y_fin - y_in)
            theta_t = theta_in + s * (theta_fin - theta_in)

            # 4. CINEMÁTICA INVERSA EN CADA PASO
            q1_t, q2_t, q3_t = self.cin_inv(x_t, y_t, theta_t)

            # Publicación de datos hacia Gazebo / RViz
            self.pub_joint01.publish(Float64(data=float(q1_t)))
            self.pub_joint02.publish(Float64(data=float(q2_t)))
            self.pub_joint03.publish(Float64(data=float(q3_t)))

            delta_t = delta_t + 1
            time.sleep(0.5)

    def cin_inv(self, x_t_in, y_t_in, theta_t_in):
        L1 = 0.5
        L2 = 0.5
        L3 = 0.3

        # Desacoplamiento de la muñeca
        x_3 = x_t_in - L3 * cos(theta_t_in)
        y_3 = y_t_in - L3 * sin(theta_t_in)

        # Teorema de cosenos
        cos_theta2 = (pow(x_3, 2) + pow(y_3, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2)
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))  
        q2_val = acos(cos_theta2)

        beta = atan2(y_3, x_3)
        
        cos_psi = (pow(x_3, 2) + pow(y_3, 2) + pow(L1, 2) - pow(L2, 2)) / (2 * L1 * sqrt(pow(x_3, 2) + pow(y_3, 2)))
        cos_psi = max(-1.0, min(1.0, cos_psi))  
        psi = acos(cos_psi)

        q1_val = beta - psi
        q3_val = theta_t_in - q1_val - q2_val

        return q1_val, q2_val, q3_val

def main(args=None):
    rclpy.init(args=args)
    node = DiagonalTrajectoryPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
