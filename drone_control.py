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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus


class Drone(Node):
    def __init__(self):
        super().__init__("DWA_DRONE_CONTROL")
        self.x = 0
        self.y = 0
        self.z = 0
        self.u = 0
        self.v = 0
        self.w = 0
        self.theta = 0
        self.phi = 0


        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        

        #PUBLISHERS_START
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        #PUBLISHERS_END

        self.timer = self.create_timer(0.1, self.vehicle_command)

        self.arm = self.vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1= 1.0)

    

    def vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)





    """def user_interface(self):
        input("X: ", self.x)
        input("Y: ", self.y)
        input("Z: ", self.z)
        position_setpoint(self.x, self.y, self.z)

    def position_setpoint(self, x:float, y:float, z:float):
        msg = TrajectorySetpoint
        msg.position = [x,y,z]
        msg.yaw = 1.57079  
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)"""
    


def main (args = None) -> None:
    print('Deve estar funfando')
    rclpy.init(args=args)
    drone = Drone()
    rclpy.spin(drone)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
      
