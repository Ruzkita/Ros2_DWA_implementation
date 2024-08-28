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
        self.x = [0.0, 5.0, 3.0]
        self.y = [0.0, 2.0, 3.0]
        self.z = [5.0, 5.0, 3.0]
        self.u = 0.0
        self.v = 0.0
        self.w = 0.0
        self.i = 0
        self.j = 0
        self.theta = 0.0
        self.phi = 0.0
        self.threshold = 0.5
        self.offboard_counter = 0.0


        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        

        #PUBLISHERS_START

        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)

        #PUBLISHERS_END

        #SUBSCRIBERS_START

        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)

        #SUBSCRIBERS_END

        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        self.timer = self.create_timer(0.1, self.timer_callback)
    

    def vehicle_command(self, command, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
    
    def arm_vehicle(self):
        self.vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1= 1.0)
    
    def engange_offboard(self):
        self.vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1 = 1.0, param2 = 6.0)
        #self.get_logger().info("Entrando em modo offboar")
    
    def trajectory_setpoint(self, x:float, y:float, z:float):
        msg = TrajectorySetpoint()
        msg.position = [-x, -y, -z]
        msg.yaw = 1.57079
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Indo para {[x, y, z]} com i = {self.i} e j = {self.j} z = {self.vehicle_local_position.z}")

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status
    
    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position
    
    def offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    
    def timer_callback(self) -> None:

        self.offboard_control_mode()

        if self.offboard_counter == 10:
            self.engange_offboard()
            self.arm_vehicle()
        
        if self.offboard_counter < 11:
            self.offboard_counter += 1

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

            self.trajectory_setpoint(self.x[self.i], self.y[self.i], self.z[self.i])

            if (-self.x[self.i] - self.threshold < self.vehicle_local_position.x < -self.x[self.i] + self.threshold) and self.j == 0:
                self.j += 1
            if (-self.y[self.i] - self.threshold < self.vehicle_local_position.y < -self.y[self.i] + self.threshold) and self.j == 1:
                self.j += 1
            if (-self.z[self.i] - self.threshold < self.vehicle_local_position.z < -self.z[self.i] + self.threshold) and self.j == 2:
                self.j += 1
        
            if self.j == 3 and self.i < len(self.x) - 1:
                self.i += 1
                self.j = 0


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
      
