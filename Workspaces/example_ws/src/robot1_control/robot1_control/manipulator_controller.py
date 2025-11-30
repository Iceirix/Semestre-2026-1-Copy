#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PointStamped
from .kinematics import Robot1Kinematics
from .dynamics import Robot1Dynamics

class Robot1ManipulatorController(Node):
  def __init__(self):
    super().__init__("robot1_manipulator_controller")
    
    # Crear objetos de cinemática y dinámica
    self.robot_kinematics = Robot1Kinematics()
    self.robot_kinematics.redirect_print(self.get_logger().info)
    self.robot_kinematics.direct_kinematics()
    
    self.robot_dynamics = Robot1Dynamics()
    self.robot_dynamics.define_kinematics(self.robot_kinematics)
    self.robot_dynamics.define_dynamics()
    
    # Variable de control para trayectoria activa
    self.moving = False
    
    # Perfil de calidad de información
    qos_profile = QoSProfile(
      reliability=ReliabilityPolicy.BEST_EFFORT, 
      history=HistoryPolicy.KEEP_LAST,
      depth=10
    )
    
    # SUSCRIPTORES
    # Recibir información de una posición deseada (via mensaje Twist)
    self.end_effector_goal_subscriber = self.create_subscription(
      Twist, "/robot1/end_effector_goal", 
      self.end_effector_callback, 10
    )
    
    # Recibir información de una posición clickeada en RViz
    self.clicked_point_subscriber = self.create_subscription(
      PointStamped, "/robot1/clicked_point", 
      self.clicked_point_callback, 10
    )
    
    # Recibir información de posición actual de las juntas
    self.joint_states_subscriber = self.create_subscription(
      JointState, "/robot1/joint_states", 
      self.joint_states_callback, 10
    )
    
    # PUBLICADORES
    # Enviar información de las juntas al controller manager
    self.joint_goals_publisher = self.create_publisher(
      JointState, "/robot1/joint_goals", 10
    )
    
    self.get_logger().info("Robot 1 - Controlador inicializado (plano XY)")
    
  def end_effector_callback(self, msg: Twist):
    """Callback para recibir objetivo del efector final via Twist"""
    if self.moving:
      self.get_logger().warning("Robot 1 - Trayectoria en progreso. Mensaje rechazado")
      return
      
    self.moving = True
    self.get_logger().info("Robot 1 - Punto objetivo recibido")
    
    # Extraer posición objetivo (x, y, alpha) del mensaje Twist
    # linear.x = x, linear.y = y, angular.z = alpha
    self.robot_kinematics.trajectory_generator(
      self.current_joint_states.position,
      [msg.linear.x, msg.linear.y, msg.angular.z], 
      3
    )
    
    # Implementar modelo de cinemática inversa
    self.robot_kinematics.inverse_kinematics()
    
    # Iniciar publicación periódica
    self._start_trajectory_publisher()
    
  def clicked_point_callback(self, msg: PointStamped):
    """Callback para recibir puntos clickeados en RViz"""
    if self.moving:
      self.get_logger().warning("Robot 1 - Trayectoria en progreso. Mensaje rechazado")
      return
      
    self.moving = True
    self.get_logger().info("Robot 1 - Punto objetivo clickeado")
    
    # Para Robot 1 (plano XY): usar x, y del punto clickeado
    # Alpha (orientación) = 0
    self.robot_kinematics.trajectory_generator(
      self.current_joint_states.position,
      [msg.point.x, msg.point.y, 0.0], 
      3
    )
    
    # Implementar modelo de cinemática inversa
    self.robot_kinematics.inverse_kinematics()
    
    # Implementar dinámica
    self.robot_dynamics.lagrange_effort_generator()
    
    # Mostrar gráficas
    self.robot_kinematics.ws_graph()
    self.robot_kinematics.q_graph()
    self.robot_dynamics.effort_graph()
    
    # Iniciar publicación periódica
    self._start_trajectory_publisher()
  
  def _start_trajectory_publisher(self):
    """Iniciar timer para publicar trayectoria"""
    self.count = 0
    self.joint_goals = JointState()
    self.joint_goals.name = ["shoulder_joint_r1", "arm_joint_r1", "forearm_joint_r1"]
    
    self.get_logger().info("Robot 1 - Publicando trayectoria de las juntas")
    self.position_publisher_timer = self.create_timer(
      self.robot_kinematics.dt, 
      self.trajectory_publisher_callback
    )
    
  def trajectory_publisher_callback(self):
    """Callback periódico para publicar puntos de la trayectoria"""
    # Marca de tiempo
    self.joint_goals.header.stamp = self.get_clock().now().to_msg()
    
    # Obtener valores de las juntas de las matrices de muestreo
    th1 = float(self.robot_kinematics.q_m[0, self.count])
    th2 = float(self.robot_kinematics.q_m[1, self.count])
    th3 = float(self.robot_kinematics.q_m[2, self.count])
    
    # Asignar valor al mensaje
    self.joint_goals.position = [th1, th2, th3]
    
    # Publicar
    self.joint_goals_publisher.publish(self.joint_goals)
    
    self.count += 1
    
    # Verificar si terminó la trayectoria
    if self.count >= len(self.robot_kinematics.q_m[0, :]):
      self.count = 0
      self.position_publisher_timer.cancel()
      self.get_logger().info("Robot 1 - Trayectoria finalizada")
      self.moving = False
    
  def joint_states_callback(self, msg: JointState):
    """Callback para recibir estado actual de las juntas"""
    self.current_joint_states = msg

def main(args=None):
  try:
    rclpy.init(args=args)
    node = Robot1ManipulatorController()
    rclpy.spin(node)
  except KeyboardInterrupt as e:
    print("Robot 1 - Node stopped")
  finally: 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
