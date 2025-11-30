#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Robot1ControlManager(Node):
  def __init__(self):
    super().__init__("robot1_control_manager")
    
    # SUSCRIPTORES
    # Recibir estados del robot desde hardware interface
    self.joint_state_subscriber = self.create_subscription(
      JointState, "/robot1/joint_states",
      self.joint_state_callback, 10
    )
    
    # Recibir comandos del controlador (manipulator_controller)
    self.joint_goals_subscriber = self.create_subscription(
      JointState, "/robot1/joint_goals",
      self.joint_goal_callback, 10
    )
    
    # PUBLICADORES
    # Enviar información a la interfaz de hardware
    self.hardware_command_publisher = self.create_publisher(
      JointState, "/robot1/joint_hardware_objectives", 10
    )
    
    self.get_logger().info("Robot 1 - Control Manager inicializado")
    
  def joint_state_callback(self, msg: JointState):
    """Callback para recibir estado actual del robot"""
    # Aquí podrías procesar el estado actual si fuera necesario
    # Por ejemplo, aplicar filtros, validaciones, etc.
    pass

  def joint_goal_callback(self, msg: JointState):
    """Callback para recibir comandos de posición deseada"""
    # En esta sección, se envían comandos a la interfaz de hardware
    # En este ejemplo sencillo, envía el mensaje tal cual lo recibe
    # En un sistema real, aquí podrías:
    # - Aplicar límites de seguridad
    # - Convertir comandos de posición a comandos de torque
    # - Implementar control PID
    # - Interpolar comandos
    
    self.hardware_command_publisher.publish(msg)

def main(args=None):
  try:
    rclpy.init(args=args)
    node = Robot1ControlManager()
    rclpy.spin(node)
  except KeyboardInterrupt as e:
    print("Robot 1 - Control Manager stopped")
  finally: 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()