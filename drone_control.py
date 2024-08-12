#!/usr/bin/env python3

#controlar o drone
    #interface do usuário
        #comando de posição
        #entrada em modo offboard
        #notificações da situação do trajeto
    #movimentação do drone
        #uso da camera para interpretar o ambiente
            #avaliação de trajetoria
            #monitoramento de obstáculos
        #configuração de movimentação
            #graus de liberdade
            #rotações
            #velocidade
                #angular
                #linear
        #devio de obstaculos detectados
            #reavaliação de trajetoria

import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode


class Drone(Node):
    def __init__(self):
        rclpy.init()
        rclpy.create_node("DWA_DRONE_CONTROL")
        self.x = 0
        self.y = 0
        self.z = 0
        self.u = 0
        self.v = 0
        self.w = 0
        self.theta = 0







if __name__ == "__main__":
    try:
        drone = Robot()
        #drone.user_interface()
    except rclpy.ROSInterruptException:
        pass