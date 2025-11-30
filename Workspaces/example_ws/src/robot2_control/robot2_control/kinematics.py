#!/usr/bin/env python3
from sympy import *
import matplotlib.pyplot as plt

class Robot2Kinematics():
  """Robot 2: Manipulador RRR con base rotatoria (movimiento 3D)"""
  def __init__(self):
    pass
    
  def direct_kinematics(self):
    print("Robot 2: Definiendo variables del modelo en sympy")
    self.theta_0_1, self.theta_1_2, self.theta_2_3 = symbols("theta_0_1, theta_1_2, theta_2_3")
    self.l0 = 0.05  # Altura de base rotatoria (5cm)
    self.l1 = 0.20  # Longitud eslabón 1 (20cm)
    self.l2 = 0.35  # Longitud eslabón 2 (35cm) - CORREGIDO
    
    # ROBOT 2: Base rotatoria + RR planar
    # La base rota en Z, luego el manipulador RR se mueve en un plano vertical
    
    # T_0_1: Rotación de base en Z + elevación
    # Después de rotar, el sistema queda listo para que el brazo RR se mueva en plano XZ
    self.T_0_1 = Matrix([
        [cos(self.theta_0_1), -sin(self.theta_0_1), 0, 0],
        [sin(self.theta_0_1), cos(self.theta_0_1), 0, 0],
        [0, 0, 1, self.l0],
        [0, 0, 0, 1]
    ])
    
    # Rotación de 90° en X para orientar el brazo RR verticalmente
    R_x_90 = Matrix([
        [1, 0, 0, 0],
        [0, 0, -1, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])
    
    # T_1_2: Primer eslabón del RR (20cm) con rotación theta_1_2 en Z local
    T_link1 = Matrix([
        [1, 0, 0, self.l1],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    R_z_joint2 = Matrix([
        [cos(self.theta_1_2), -sin(self.theta_1_2), 0, 0],
        [sin(self.theta_1_2), cos(self.theta_1_2), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    self.T_1_2 = R_x_90 * T_link1 * R_z_joint2
    
    # T_2_3: Segundo eslabón del RR (35cm) con rotación theta_2_3 en Z local
    T_link2 = Matrix([
        [1, 0, 0, self.l2],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    R_z_joint3 = Matrix([
        [cos(self.theta_2_3), -sin(self.theta_2_3), 0, 0],
        [sin(self.theta_2_3), cos(self.theta_2_3), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    self.T_2_3 = T_link2 * R_z_joint3
    
    # Transformación completa al final del eslabón 2
    T_0_p = simplify(self.T_0_1 * self.T_1_2 * self.T_2_3)

    # Posición del efector final (final del eslabón 2)
    x_0_p = T_0_p[0, 3]
    y_0_p = T_0_p[1, 3]
    z_0_p = T_0_p[2, 3]
    
    self.xi_0_p = Matrix([
      [x_0_p], 
      [y_0_p], 
      [z_0_p]
    ])
    
    # Calcular Jacobiano
    self.J = Matrix.hstack(diff(self.xi_0_p, self.theta_0_1), 
                           diff(self.xi_0_p, self.theta_1_2), 
                           diff(self.xi_0_p, self.theta_2_3))
    
    print("Robot 2: Jacobiano calculado")
    
    # Verificar determinante
    try:
      det_J = self.J.det()
      # Evaluar en configuración de prueba
      test_config = {self.theta_0_1: 0.1, self.theta_1_2: 0.5, self.theta_2_3: 0.3}
      det_test = float(det_J.subs(test_config))
      print(f"Robot 2: det(J) en config prueba = {det_test:.6f}")
      
      # Si el determinante es muy pequeño, usar pseudoinversa
      if abs(det_test) < 1e-6:
        print("Robot 2: Jacobiano singular, usando pseudoinversa")
        self.J_inv = self.J.pinv()
      else:
        try:
          self.J_inv = self.J.inv()
          print("Robot 2: Usando inversa estándar del Jacobiano ✅")
        except:
          print("Robot 2: Usando pseudoinversa del Jacobiano")
          self.J_inv = self.J.pinv()
    except Exception as e:
      print(f"Robot 2: Error verificando determinante: {e}")
      print("Robot 2: Usando pseudoinversa del Jacobiano (por seguridad)")
      self.J_inv = self.J.pinv()
    
    # Vector de velocidades del espacio de trabajo
    self.x_dot, self.y_dot, self.z_dot = symbols("x_dot, y_dot, z_dot")
    self.xi_dot = Matrix([[self.x_dot], 
                          [self.y_dot], 
                          [self.z_dot]])
    
    # Ecuación de cinemática inversa
    self.q_dot = self.J_inv * self.xi_dot
    print("Robot 2: Variables definidas")
    
  def trajectory_generator(self, q_in = [0.1, 0.1, 0.1], xi_fn = [0.3, 0.3, 0.2], duration = 4):
    self.freq = 30
    print("Robot 2: Definiendo trayectoria")
    
    # Polinomio de interpolación de 5to orden
    self.t, a0, a1, a2, a3, a4, a5 = symbols("t, a0, a1, a2, a3, a4, a5")
    self.lam = a0 + a1*self.t + a2*(self.t)**2 + a3*(self.t)**3 + a4*(self.t)**4 + a5*(self.t)**5
    self.lam_dot = diff(self.lam, self.t)
    self.lam_dot_dot = diff(self.lam_dot, self.t)
    
    # Condiciones de frontera
    ec1 = self.lam.subs(self.t, 0)
    ec2 = self.lam.subs(self.t, duration) - 1
    ec3 = self.lam_dot.subs(self.t, 0)
    ec4 = self.lam_dot.subs(self.t, duration)
    ec5 = self.lam_dot_dot.subs(self.t, 0)
    ec6 = self.lam_dot_dot.subs(self.t, duration)
    
    terms = solve([ec1, ec2, ec3, ec4, ec5, ec6], [a0, a1, a2, a3, a4, a5], dict = True)
    self.lam_s          = self.lam.subs(terms[0])
    self.lam_dot_s      = self.lam_dot.subs(terms[0])
    self.lam_dot_dot_s  = self.lam_dot_dot.subs(terms[0])
    
    # Posición inicial del efector final
    xi_in = self.xi_0_p.subs({
      self.theta_0_1: q_in[0],
      self.theta_1_2: q_in[1],
      self.theta_2_3: q_in[2]
    })
    
    # Ecuaciones de posición, velocidad y aceleración
    xi = xi_in + Matrix([
      [self.lam_s * (xi_fn[0] - xi_in[0])],
      [self.lam_s * (xi_fn[1] - xi_in[1])],
      [self.lam_s * (xi_fn[2] - xi_in[2])]
    ])
    
    xi_dot = Matrix([
      [self.lam_dot_s * (xi_fn[0] - xi_in[0])],
      [self.lam_dot_s * (xi_fn[1] - xi_in[1])],
      [self.lam_dot_s * (xi_fn[2] - xi_in[2])]
    ])
    
    xi_dot_dot = Matrix([
      [self.lam_dot_dot_s * (xi_fn[0] - xi_in[0])],
      [self.lam_dot_dot_s * (xi_fn[1] - xi_in[1])],
      [self.lam_dot_dot_s * (xi_fn[2] - xi_in[2])]
    ])
    
    # Muestreo
    self.samples = int(self.freq * duration + 1)
    self.dt = 1/self.freq
    self.xi_m         = Matrix.zeros(3, self.samples)
    self.xi_dot_m     = Matrix.zeros(3, self.samples)
    self.xi_dot_dot_m = Matrix.zeros(3, self.samples)
    self.t_m = Matrix.zeros(1, self.samples)
    
    self.t_m[0, 0] = 0
    for a in range(self.samples - 1):
      self.t_m[0, a+1] = self.t_m[0, a] + self.dt
    
    # Crear funciones lambda
    xi_lam         = lambdify([self.t], xi, modules='numpy')
    xi_dot_lam     = lambdify([self.t], xi_dot, modules='numpy')
    xi_dot_dot_lam = lambdify([self.t], xi_dot_dot, modules='numpy')
    
    for a in range(self.samples):
      self.xi_m[:, a]         = xi_lam(float(self.t_m[0,a]))
      self.xi_dot_m[:, a]     = xi_dot_lam(float(self.t_m[0,a]))
      self.xi_dot_dot_m[:, a] = xi_dot_dot_lam(float(self.t_m[0,a]))
    
    self.q_in = q_in

  def inverse_kinematics(self):
    print("Robot 2: Modelando cinemática inversa")
    self.q_m         = Matrix.zeros(3, self.samples)
    self.q_dot_m     = Matrix.zeros(3, self.samples)
    self.q_dot_dot_m = Matrix.zeros(3, self.samples)
    
    self.q_m[:, 0]          = Matrix([[self.q_in[0]], [self.q_in[1]], [self.q_in[2]]])
    self.q_dot_m[:, 0]      = Matrix.zeros(3, 1)
    self.q_dot_dot_m[:, 0]  = Matrix.zeros(3, 1)
    
    self.q_dot_lam = lambdify([self.x_dot, self.y_dot, self.z_dot,
                               self.theta_0_1, self.theta_1_2, self.theta_2_3], 
                               self.q_dot, modules='numpy')
    
    # Diagnóstico: Verificar que q_dot no sea cero constantemente
    print(f"Robot 2: Posición inicial q = {self.q_in}")
    
    for a in range(self.samples - 1):
      self.q_m[:, a+1] = self.q_m[:, a] + self.q_dot_m[:, a] * self.dt
      
      try:
        self.q_dot_m[:, a+1] = self.q_dot_lam(float(self.xi_dot_m[0, a]), 
                                              float(self.xi_dot_m[1, a]), 
                                              float(self.xi_dot_m[2, a]),
                                              float(self.q_m[0, a]), 
                                              float(self.q_m[1, a]), 
                                              float(self.q_m[2, a]))
        
        # Diagnóstico cada 30 muestras
        if a % 30 == 0:
          print(f"Robot 2: Muestra {a}: q_dot = [{float(self.q_dot_m[0, a+1]):.4f}, {float(self.q_dot_m[1, a+1]):.4f}, {float(self.q_dot_m[2, a+1]):.4f}]")
          
      except Exception as e:
        print(f"Robot 2: Error en muestra {a}: {e}")
        self.q_dot_m[:, a+1] = Matrix.zeros(3, 1)
      
      self.q_dot_dot_m[:, a+1] = (self.q_dot_m[:, a+1] - self.q_dot_m[:, a]) / self.dt
    
    print(f"Robot 2: Posición final q = [{float(self.q_m[0, -1]):.4f}, {float(self.q_m[1, -1]):.4f}, {float(self.q_m[2, -1]):.4f}]")
    print("Robot 2: Trayectoria de las juntas generada")

  def ws_graph(self):
    fig, (p_g, v_g, a_g) = plt.subplots(nrows=1, ncols=3)
    fig.suptitle("Robot 2 - Espacio de trabajo (XYZ)")
    
    p_g.set_title("Posiciones")
    p_g.plot(self.t_m.T, self.xi_m[0, :].T, color = "RED", label="x")
    p_g.plot(self.t_m.T, self.xi_m[1, :].T, color = "GREEN", label="y")
    p_g.plot(self.t_m.T, self.xi_m[2, :].T, color = "BLUE", label="z")
    p_g.legend()

    v_g.set_title("Velocidades")
    v_g.plot(self.t_m.T, self.xi_dot_m[0, :].T, color = "RED")
    v_g.plot(self.t_m.T, self.xi_dot_m[1, :].T, color = "GREEN")
    v_g.plot(self.t_m.T, self.xi_dot_m[2, :].T, color = "BLUE")

    a_g.set_title("Aceleraciones")
    a_g.plot(self.t_m.T, self.xi_dot_dot_m[0, :].T, color = "RED")
    a_g.plot(self.t_m.T, self.xi_dot_dot_m[1, :].T, color = "GREEN")
    a_g.plot(self.t_m.T, self.xi_dot_dot_m[2, :].T, color = "BLUE")
    
    plt.show()

  def q_graph(self):
    fig, (p_g, v_g, a_g) = plt.subplots(nrows=1, ncols=3)
    fig.suptitle("Robot 2 - Espacio de las juntas")
    
    p_g.set_title("Posiciones")
    p_g.plot(self.t_m.T, self.q_m[0, :].T, color = "RED", label="q1 (base)")
    p_g.plot(self.t_m.T, self.q_m[1, :].T, color = "GREEN", label="q2")
    p_g.plot(self.t_m.T, self.q_m[2, :].T, color = "BLUE", label="q3")
    p_g.legend()

    v_g.set_title("Velocidades")
    v_g.plot(self.t_m.T, self.q_dot_m[0, :].T, color = "RED")
    v_g.plot(self.t_m.T, self.q_dot_m[1, :].T, color = "GREEN")
    v_g.plot(self.t_m.T, self.q_dot_m[2, :].T, color = "BLUE")

    a_g.set_title("Aceleraciones")
    a_g.plot(self.t_m.T, self.q_dot_dot_m[0, :].T, color = "RED")
    a_g.plot(self.t_m.T, self.q_dot_dot_m[1, :].T, color = "GREEN")
    a_g.plot(self.t_m.T, self.q_dot_dot_m[2, :].T, color = "BLUE")
    
    plt.show()

  def trans_homo_xz(self, x=0, z=0, gamma=0, alpha=0)->Matrix:
    """Transformación homogénea para plano XZ"""
    R_z = Matrix([ [cos(alpha), -sin(alpha), 0], [sin(alpha), cos(alpha), 0],[0, 0, 1]])
    R_x = Matrix([ [1, 0, 0], [0, cos(gamma), -sin(gamma)],[0, sin(gamma), cos(gamma)]])

    p_x = Matrix([[x],[0],[0]])
    p_z = Matrix([[0],[0],[z]])

    T_x = Matrix.vstack(Matrix.hstack(R_x, p_x), Matrix([[0,0,0,1]]))
    T_z = Matrix.vstack(Matrix.hstack(R_z, p_z), Matrix([[0,0,0,1]]))
    return T_x * T_z
  
  def trans_homo(self, x, y, z, gamma, beta, alpha):
    """Transformación homogénea general"""
    R_z = Matrix([ [cos(alpha), -sin(alpha), 0], [sin(alpha), cos(alpha), 0],[0, 0, 1]])
    R_y = Matrix([ [cos(beta), 0, sin(beta)], [0, 1, 0],[-sin(beta), 0, cos(beta)]])
    R_x = Matrix([ [1, 0, 0], [0, cos(gamma), -sin(gamma)],[0, sin(gamma), cos(gamma)]])

    p_x = Matrix([[x],[0],[0]])
    p_y = Matrix([[0],[y],[0]])
    p_z = Matrix([[0],[0],[z]])
    
    T_x = Matrix.vstack(Matrix.hstack(R_x, p_x), Matrix([[0,0,0,1]]))
    T_y = Matrix.vstack(Matrix.hstack(R_y, p_y), Matrix([[0,0,0,1]]))
    T_z = Matrix.vstack(Matrix.hstack(R_z, p_z), Matrix([[0,0,0,1]]))
    return T_x * T_y * T_z
    
  def redirect_print(self, new_print):
    global print
    print = new_print

def main():
  robot = Robot2Kinematics()
  robot.direct_kinematics()
  robot.trajectory_generator()
  robot.inverse_kinematics()
  robot.ws_graph()
  robot.q_graph()

if __name__ == "__main__":
  main()