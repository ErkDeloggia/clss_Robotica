#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from math import cos, sin, atan2, acos, pow, sqrt

class ScaraControl(Node):
    def __init__(self):
        super().__init__('q_plan_node')
        
        # Publicadores unificados para Gazebo/Rviz
        self.pub_joint01 = self.create_publisher(Float64, '/joint1/cmd_pos', 10)
        self.pub_joint02 = self.create_publisher(Float64, '/joint2/cmd_pos', 10)
        self.pub_joint03 = self.create_publisher(Float64, '/joint3/cmd_pos', 10)
        
        # --- DEFINICIÓN DE LAS POSICIONES CARTESIANAS ---
        # Coordenadas iniciales (Ajusta aquí a 0.3 si deseas usar tu nueva prueba)
        self.x_i = 0.3
        self.y_i = 0.3
        self.theta_i = 0.0
        
        # Coordenadas finales (Ajusta aquí a 0.8 si deseas usar tu nueva prueba)
        self.x_j = 0.8
        self.y_j = 0.8
        self.theta_j = 0.0
        
        # Parámetros de tiempo y trayectoria
        self.tf = 10.0
        self.delta_t = 0.0
        self.paso_actual = 1
        self.total_pasos = 10  # Rango del 1 al 10 de tu bucle for
        
        # --- RESOLUCIÓN DE CINEMÁTICA INVERSA INICIAL Y FINAL ---
        # Se ejecuta una sola vez al arrancar el nodo
        self.theta_1_i, self.theta_2_i, self.theta_3_i = self.cin_inv(self.x_i, self.y_i, self.theta_i)
        self.theta_1_j, self.theta_2_j, self.theta_3_j = self.cin_inv(self.x_j, self.y_j, self.theta_j)
        
        # Objetos de mensaje persistentes (Evita crearlos en cada ciclo)
        self.theta1_msg = Float64()
        self.theta2_msg = Float64()
        self.theta3_msg = Float64()
        
        # Temporizador: Se ejecuta cada 0.5 segundos (Reemplaza al time.sleep(0.5))
        self.timer_control = self.create_timer(0.5, self.cbck_scara_control)
        self.get_logger().info('Nodo controlador scara inicializado con cinemática inversa integrada.')

    def cbck_scara_control(self):
        if self.paso_actual <= self.total_pasos:
            print('Intervalo de tiempo ' + str(self.paso_actual))
            
            # Tiempo normalizado
            t_sim = self.delta_t / self.tf
            
            # --- INTERPOLACIÓN POLINOMIAL DE 5TO GRADO (Tu fórmula exacta) ---
            theta_1_t = self.theta_1_i + (10*pow(t_sim,3) - 15*pow(t_sim,4) + 6*pow(t_sim,5)) * (self.theta_1_j - self.theta_1_i)
            theta_2_t = self.theta_2_i + (10*pow(t_sim,3) - 15*pow(t_sim,4) + 6*pow(t_sim,5)) * (self.theta_2_j - self.theta_2_i)
            theta_3_t = self.theta_3_i + (10*pow(t_sim,3) - 15*pow(t_sim,4) + 6*pow(t_sim,5)) * (self.theta_3_j - self.theta_3_i)
            
            # Asignación correcta a la propiedad .data
            self.theta1_msg.data = float(theta_1_t)
            self.theta2_msg.data = float(theta_2_t)
            self.theta3_msg.data = float(theta_3_t)
            
            # Publicación hacia el simulador
            self.pub_joint01.publish(self.theta1_msg)
            self.pub_joint02.publish(self.theta2_msg)
            self.pub_joint03.publish(self.theta3_msg)
            
            # Incrementos de estado
            self.delta_t = self.delta_t + 1.0
            self.paso_actual += 1
        else:
            self.get_logger().info('Trayectoria finalizada correctamente.')
            self.timer_control.destroy()

    def cin_inv(self, x_in, y_in, theta_in):
        # Parámetros físicos del robot
        L1 = 0.5
        L2 = 0.5
        L3 = 0.3
        
        # Desacoplamiento de muñeca
        x_3_in = x_in - L3 * cos(theta_in)
        y_3_in = y_in - L3 * sin(theta_in)
        
        # Ley de Cosenos
        theta_2_in = acos((pow(x_3_in, 2) + pow(y_3_in, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2))
        beta = atan2(y_3_in, x_3_in)
        
        # Ángulo alfa con protección de paréntesis
        alpha = acos((pow(x_3_in, 2) + pow(y_3_in, 2) + pow(L1, 2) - pow(L2, 2)) / (2 * L1 * sqrt(pow(x_3_in, 2) + pow(y_3_in, 2))))
        
        # Configuración codo arriba
        theta_1_in = beta - alpha
        theta_3_in = theta_in - theta_1_in - theta_2_in
        
        return theta_1_in, theta_2_in, theta_3_in

def main(args=None):
    rclpy.init(args=args)
    node = ScaraControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
