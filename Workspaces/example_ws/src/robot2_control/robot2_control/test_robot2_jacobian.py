#!/usr/bin/env python3
"""
Script de prueba para verificar el Jacobiano del Robot 2 - VERSIÓN CORREGIDA
"""
from sympy import *
import numpy as np

# Variables
theta_0_1, theta_1_2, theta_2_3 = symbols("theta_0_1, theta_1_2, theta_2_3")
l0 = 0.05
l1 = 0.20
l2 = 0.30

# NUEVA CINEMÁTICA CORREGIDA
# T_0_1: Solo rotación de base sobre eje Z + elevación
R_z1 = Matrix([[cos(theta_0_1), -sin(theta_0_1), 0, 0],
               [sin(theta_0_1), cos(theta_0_1), 0, 0],
               [0, 0, 1, l0],
               [0, 0, 0, 1]])
T_0_1 = R_z1

# T_1_2: Rotación 90° en X + traslación + rotación theta_1_2
R_x = Matrix([[1, 0, 0, 0],
              [0, 0, -1, 0],
              [0, 1, 0, 0],
              [0, 0, 0, 1]])

T_trans_1 = Matrix([[1, 0, 0, l1],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

R_z2 = Matrix([[cos(theta_1_2), -sin(theta_1_2), 0, 0],
               [sin(theta_1_2), cos(theta_1_2), 0, 0],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])

T_1_2 = R_x * T_trans_1 * R_z2

# T_2_3: Traslación + rotación theta_2_3
T_trans_2 = Matrix([[1, 0, 0, l2],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

R_z3 = Matrix([[cos(theta_2_3), -sin(theta_2_3), 0, 0],
               [sin(theta_2_3), cos(theta_2_3), 0, 0],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])

T_2_3 = T_trans_2 * R_z3

# Cinemática directa
T_0_p = simplify(T_0_1 * T_1_2 * T_2_3)

x_0_p = T_0_p[0, 3]
y_0_p = T_0_p[1, 3]
z_0_p = T_0_p[2, 3]

xi_0_p = Matrix([
  [x_0_p], 
  [y_0_p], 
  [z_0_p]
])

print("="*60)
print("CINEMÁTICA DIRECTA - ROBOT 2")
print("="*60)
print("\nVector de posición del efector final:")
pprint(xi_0_p)
print("\n" + "="*60 + "\n")

# Jacobiano
J = Matrix.hstack(diff(xi_0_p, theta_0_1), 
                  diff(xi_0_p, theta_1_2), 
                  diff(xi_0_p, theta_2_3))

print("Jacobiano:")
pprint(J)
print("\n" + "="*60 + "\n")

# Determinante
det_J = J.det()
det_simp = simplify(det_J)
print("Determinante del Jacobiano (simplificado):")
pprint(det_simp)
print("\n" + "="*60 + "\n")

# Test en varias configuraciones
configs = [
    {"theta_0_1": 0.0, "theta_1_2": 0.0, "theta_2_3": 0.0, "nombre": "Configuración home"},
    {"theta_0_1": 0.1, "theta_1_2": 0.1, "theta_2_3": 0.1, "nombre": "Configuración inicial"},
    {"theta_0_1": 0.5, "theta_1_2": 0.8, "theta_2_3": -0.5, "nombre": "Configuración arbitraria"},
    {"theta_0_1": 0.0, "theta_1_2": pi/4, "theta_2_3": pi/4, "nombre": "45° shoulder y arm"},
]

print("PRUEBAS DE CONFIGURACIÓN:")
print("="*60)
for config in configs:
    config_vals = {theta_0_1: config["theta_0_1"], 
                   theta_1_2: config["theta_1_2"], 
                   theta_2_3: config["theta_2_3"]}
    
    J_eval = J.subs(config_vals)
    det_eval = J_eval.det()
    pos_eval = xi_0_p.subs(config_vals)
    
    print(f"\n{config['nombre']}:")
    print(f"  q = [{config['theta_0_1']:.3f}, {config['theta_1_2']:.3f}, {config['theta_2_3']:.3f}]")
    print(f"  Posición: x={float(pos_eval[0]):.4f}, y={float(pos_eval[1]):.4f}, z={float(pos_eval[2]):.4f}")
    print(f"  det(J) = {float(det_eval):.6f}")
    print(f"  Singular: {'SÍ ❌' if abs(float(det_eval)) < 1e-6 else 'NO ✅'}")

print("\n" + "="*60)

# Probar cinemática inversa
print("\nPRUEBA DE CINEMÁTICA INVERSA:")
print("="*60)

# Configuración objetivo
target_pos = {"x": 0.3, "y": 0.2, "z": 0.2}
q_current = {"theta_0_1": 0.1, "theta_1_2": 0.1, "theta_2_3": 0.1}

print(f"\nPosición objetivo: x={target_pos['x']}, y={target_pos['y']}, z={target_pos['z']}")
print(f"Configuración actual: q={[q_current['theta_0_1'], q_current['theta_1_2'], q_current['theta_2_3']]}")

# Calcular velocidad cartesiana necesaria
xi_current = xi_0_p.subs({theta_0_1: q_current["theta_0_1"],
                          theta_1_2: q_current["theta_1_2"],
                          theta_2_3: q_current["theta_2_3"]})

xi_dot_desired = Matrix([[target_pos["x"] - float(xi_current[0])],
                         [target_pos["y"] - float(xi_current[1])],
                         [target_pos["z"] - float(xi_current[2])]])

print(f"\nVelocidad cartesiana deseada: {[float(xi_dot_desired[i]) for i in range(3)]}")

# Calcular Jacobiano en configuración actual
J_current = J.subs({theta_0_1: q_current["theta_0_1"],
                    theta_1_2: q_current["theta_1_2"],
                    theta_2_3: q_current["theta_2_3"]})

# Calcular pseudoinversa
try:
    J_pinv = J_current.pinv()
    q_dot = J_pinv * xi_dot_desired
    
    print(f"\nVelocidades de juntas calculadas:")
    print(f"  q_dot = [{float(q_dot[0]):.6f}, {float(q_dot[1]):.6f}, {float(q_dot[2]):.6f}]")
    print(f"\n✅ Cinemática inversa FUNCIONAL")
    print(f"  Nota: q_dot[2] = {float(q_dot[2]):.6f} (NO debería ser exactamente 0)")
    
except Exception as e:
    print(f"❌ Error: {e}")

print("\n" + "="*60)