#!/usr/bin/env python3

import rclpy  #importa a biblioteca do ros
from rclpy.node import Node  #criação de nó
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus #controle do drone pelo px4

#OffboardControlMode -> É quem vai fazer a troca entre o modo onboard e offboard
#TrajectorySetPoint -> Vetor que vai dizer pra onde o drone deve ir
#VehicleCommand -> Vai transmitir os comandos para o drone
#VehicleLocalPosition -> Informa a posição atual do drone
#VehicleStatus -> Dá informações sobre o estado atual do drone

class OffboardControl(Node): 
    '''nó que será utilizado  para o controle automatico do veículo'''
    
    def __init__(self) -> None: # init é a primeira instacia executada quando esse objeto é criado. Antes mesmo da interação com o usuario. Tudo aqui é criado primeiro
        super().__init__('controle_onboard')

        #Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)

        #publishers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)


       #subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        
        #Valor inicial das variáveis
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_pointer = -5.0
        self.takeoff_pointer2 = -5.0
        self.i = 0
        self.j = 0
        self.coordenadas = [0, 0, 0]


        #timer para publicação de informação
        self.timer = self.create_timer(0.1, self.timer_callback)

    
    def vehicle_local_position_callback(self, vehicle_local_position):
        '''retorno para o topico vehicle_local_position'''
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        '''retorno para o topico vehicle_status'''
        self.vehicle_status = vehicle_status
    
    def arm(self):
        '''comando de armar o drone'''
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1= 1.0)
        self.get_logger().info('Comando "armar" enviado')
    
    def disarm(self):
        '''comando de desarmar o drone'''
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1= 0.0)
        self.get_logger().info('Comando "desarmar" enviado')

    def engange_offboard_mode(self):
        '''comando para entrar no modo offboard'''
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1 = 1.0, param2 = 6.0)
        self.get_logger().info("Entrando em modo offboar")
    
    def land(self):
        '''comando para entrar em modo de aterrisagem'''
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Entrando em modo de aterrisagem")
    
    def publish_offboard_control_heartbeat_signal(self):
        '''publica dados do modo offboard'''
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
    
    def publish_position_setpoint(self, x: float, y: float, z: float):
        """publica a trajetoria do veículo"""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publicando posição {[-x, -y, -z]}")

    def publish_velocity_setpoint(self, u: float, v: float, w:float):
        msg = TrajectorySetpoint()
        msg.velocity = [u, v, w]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        '''publica os comandos do veículo'''
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        '''função de retorno'''
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engange_offboard_mode()
            self.arm()
        if self.i == 0:
            self.coordenadas[0] = -float(input("Entre com o valor de x: "))
            self.coordenadas[1] = -float(input("Entre com o valor de y: "))
            self.coordenadas[2] = -float(input("Entre com o valor de z: "))
            self.i = 1


        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.i == 1:
            self.publish_position_setpoint(self.coordenadas[0], self.coordenadas[1], self.coordenadas[2])        
            
        if self.vehicle_local_position.x <= self.coordenadas[0] and self.j == 0:
            self.j += 1
        
        if self.vehicle_local_position.y <= self.coordenadas[1] and self.j == 1:
            self.j += 1
        
        if self.vehicle_local_position.z <= self.coordenadas[2] and self.j == 2:
            self.j +=1

        if self.j == 3:
            self.land()
            exit(0)
        
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
        


def main (args = None) -> None:
    print('Iniciando controle offboard...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
