#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from math import cos, sin, atan2, acos, sqrt, pow

class DiagonalTrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('scara_trajectory_planner')
        
        # Publicadores unificados adaptados a la estructura real del RoArm
        self.pub_joint01 = self.create_publisher(Float64, '/cintura/cmd_pos', 10)
        self.pub_joint02 = self.create_publisher(Float64, '/brazo/cmd_pos', 10)
        self.pub_joint03 = self.create_publisher(Float64, '/antebrazo/cmd_pos', 10)
        
        # Variables globales de control para el ciclo infinito
        self.direccion_ida = True 
        self.delta_t = 0.0
        
        # Configuración de frecuencia: 20 Hz (Ejecución cada 0.05 segundos para máxima fluidez)
        self.dt = 0.05 
        self.timer_control = self.create_timer(self.dt, self.cbck_scara_control)
        self.get_logger().info('Nodo controlador optimizado - Movimiento continuo y fluido iniciado')

    def cbck_scara_control(self):
        # 1. LIMITES REALES EN EL PLANO VERTICAL (VALIDADOS EN MATLAB)
        x_in = 0.20
        y_in = 0.25
        theta_in = 0
        
        x_fin = 0.32
        y_fin = 0.37
        theta_fin = 0
        
        tf = 5.0 # 5 segundos por tramo
        
        # Evaluamos el avance en este instante de tiempo
        t_sim = self.delta_t / tf
        
        # 2. EVALUACIÓN DEL POLINOMIO DE 5.° GRADO
        s = 10*pow(t_sim, 3) - 15*pow(t_sim, 4) + 6 * pow(t_sim, 5)
        
        # 3. INTERPOLACIÓN LINEAL ASIGNANDO SENTIDO SEGÚN LA FASE ACTUAL
        if self.direccion_ida:
            x_t = x_in + s * (x_fin - x_in)
            y_t = y_in + s * (y_fin - y_in)
            theta_t = theta_in + s * (theta_fin - theta_in)
        else:
            x_t = x_fin + s * (x_in - x_fin)
            y_t = y_fin + s * (y_in - y_fin)
            theta_t = theta_fin + s * (theta_in - theta_fin)
        
        # 4. CINEMÁTICA INVERSA EN CADA PASO (Plano Vertical)
        q_cintura, q_hombro, q_codo = self.cin_inv(x_t, y_t, theta_t)
        
        # Publicación limpia de los datos hacia los tópicos de simulación
        self.pub_joint01.publish(Float64(data=float(q_cintura)))
        self.pub_joint02.publish(Float64(data=float(q_hombro)))
        self.pub_joint03.publish(Float64(data=float(q_codo)))
        
        # Incrementamos el reloj interno en cada ciclo del timer
        self.delta_t += self.dt
        
        # Si completamos los 5 segundos de la fase actual, reiniciamos el reloj e invertimos ruta
        if self.delta_t >= tf:
            self.delta_t = 0.0
            self.direccion_ida = not self.direccion_ida
            self.get_logger().info('Cambiando de fase en la trayectoria lineal diagonal.')

    def cin_inv(self, x_t_in, y_t_in, theta_t_in):
        L1 = 0.0494  # Altura de la base
        L2 = 0.2385  # Longitud del brazo
        L3 = 0.236   # Longitud del antebrazo
        
        x_plano = x_t_in
        y_plano = y_t_in - L1
        
        cos_theta3 = (pow(x_plano, 2) + pow(y_plano, 2) - pow(L2, 2) - pow(L3, 2)) / (2 * L2 * L3)
        cos_theta3 = max(-1.0, min(1.0, cos_theta3))
        q2_val = acos(cos_theta3)
        
        beta = atan2(y_plano, x_plano)
        
        cos_psi = (pow(x_plano, 2) + pow(y_plano, 2) + pow(L2, 2) - pow(L3, 2)) / (2 * L2 * sqrt(pow(x_plano, 2) + pow(y_plano, 2)))
        cos_psi = max(-1.0, min(1.0, cos_psi))
        psi = acos(cos_psi)
        
        q1_val = beta - psi
        q0_val = 0.0
        q3_val = theta_t_in - q1_val - q2_val
        
        return q0_val, q1_val, q2_val

def main(args=None):
    rclpy.init(args=args)
    node = DiagonalTrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

