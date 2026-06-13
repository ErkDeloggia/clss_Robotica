#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class TrajectoryTest(Node):
    def __init__(self):
        super().__init__('trajectory_test')
        
        # Publicadores individuales para los tópicos reales de tu robot
        self.pub_j1 = self.create_publisher(Float64, '/joint1/cmd_pos', 10)
        self.pub_j2 = self.create_publisher(Float64, '/joint2/cmd_pos', 10)
        self.pub_j3 = self.create_publisher(Float64, '/joint3/cmd_pos', 10)
        
        # Posiciones actuales de control (empiezan en Home = 0)
        self.current_p1 = 0.0
        self.current_p2 = 0.0
        self.current_p3 = 0.0

        # Objetivos finales a alcanzar
        self.target_p1 = 0.0
        self.target_p2 = 0.0
        self.target_p3 = 0.0

        # CONFIGURACIÓN DE VELOCIDAD
        # Menor valor = Más lento. Mayor valor = Más rápido.
        self.speed = 0.01 

        # Temporizadores autónomos
        # 1. Cambia las metas globales cada 6 segundos
        self.state_timer = self.create_timer(6.0, self.change_targets)
        self.toggle = True

        # 2. Bucle de control de alta frecuencia (cada 20ms) para mover suavemente
        self.control_timer = self.create_timer(0.02, self.update_and_publish)
        
        self.get_logger().info('Controlador interpolado iniciado. Movimiento lento activado.')

    def change_targets(self):
        """Cambia el objetivo final del robot alternando estados"""
        if self.toggle:
            self.target_p1 = 1.0   # Meta Articulación 1
            self.target_p2 = 0.5   # Meta Articulación 2
            self.target_p3 = 0.2   # Meta Articulación 3
            self.get_logger().info('Cambiando meta: Hacia Posición A')
        else:
            self.target_p1 = 0.0   # Meta Home
            self.target_p2 = 0.0
            self.target_p3 = 0.0
            self.get_logger().info('Cambiando meta: Regresando a Home')
        
        self.toggle = not self.toggle

    def update_and_publish(self):
        """Aproxima las posiciones actuales a las metas de forma lenta paso a paso"""
        # Interpolación para Joint 1
        diff1 = self.target_p1 - self.current_p1
        if abs(diff1) > self.speed:
            self.current_p1 += self.speed if diff1 > 0 else -self.speed
        else:
            self.current_p1 = self.target_p1

        # Interpolación para Joint 2
        diff2 = self.target_p2 - self.current_p2
        if abs(diff2) > self.speed:
            self.current_p2 += self.speed if diff2 > 0 else -self.speed
        else:
            self.current_p2 = self.target_p2

        # Interpolación para Joint 3
        diff3 = self.target_p3 - self.current_p3
        if abs(diff3) > self.speed:
            self.current_p3 += self.speed if diff3 > 0 else -self.speed
        else:
            self.current_p3 = self.target_p3

        # Crear y empaquetar los mensajes modificados ligeramente
        msg1, msg2, msg3 = Float64(), Float64(), Float64()
        msg1.data = self.current_p1
        msg2.data = self.current_p2
        msg3.data = self.current_p3

        # Publicar en alta frecuencia
        self.pub_j1.publish(msg1)
        self.pub_j2.publish(msg2)
        self.pub_j3.publish(msg3)

def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher_node = TrajectoryTest()
    try:
        rclpy.spin(trajectory_publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        trajectory_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
