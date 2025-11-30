from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
  # Rutas de paquete "robot2_description" y archivos
  description_path = get_package_share_directory("robot2_description")
  model_path = os.path.join(description_path, "urdf", "robot2.urdf")
  rviz_conf_path = os.path.join(description_path, "rviz", "robot2_config.rviz")
  
  # Modelo URDF como parámetro
  robot_description = {"robot_description": Command(["xacro ", model_path])}

  # Nodos de control
  controller_manager_node = Node(
    package='robot2_control',
    executable="controller_manager",
    namespace='robot2'
  )

  manipulator_controller_node = Node(
    package='robot2_control',
    executable="manipulator_controller",
    namespace='robot2'
  )

  hardware_interface_node = Node(
    package='robot2_control',
    executable="hardware_interface",
    namespace='robot2'
  )

  # Visualización
  rviz_node = Node(
    package='rviz2',
    executable="rviz2",
    arguments=["-d", rviz_conf_path],
    namespace='robot2'
  )
  
  rsp_node = Node(
    package='robot_state_publisher',
    executable="robot_state_publisher",
    parameters=[robot_description],
    namespace='robot2'
  )
  
  return LaunchDescription([
    controller_manager_node,
    manipulator_controller_node,
    hardware_interface_node,
    rviz_node, 
    rsp_node
  ])