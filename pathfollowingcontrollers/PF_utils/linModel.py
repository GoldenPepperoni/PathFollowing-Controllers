import numpy as np
import control
import matplotlib as plt

from pathfollowingcontrollers.PF_utils.abstractions import *

# Longitudinal State Space linearised model
A_long = np.array([[-0.0127632, 0.353551, 0, -9.81],
          [-1.24324, -16.4517, 11.8219, 0],
          [0, -7.87678, -21.3803, 0],
          [0, 0, 1, 0]])
B_long = np.array([[0.02104349],
          [-28.0431],
          [-284.0878],
          [0]])
C_long = np.array([[0, 0, 0, 1]])
D_long = np.zeros((np.shape(C_long)[0], np.shape(B_long)[1]))

# Lateral State Space linearised model
A_lat = np.array([[-0.618709, -0.251147, -15.2152, 9.811],
         [-1.41853, -20.1165, 2.34814, 0],
         [2.18038, -3.20227, -1.9908, 0],
         [0, 1, 0, 0]])
B_lat = np.array([[2.673794, 6.571011],
         [160.6404, 1.970345],
         [18.60043, -29.287260],
         [0, 0]])
C_lat = np.array([[0, 0, 0, 1],
                  [0, 0, 1, 0]])
D_lat = np.zeros((np.shape(C_lat)[0], np.shape(B_lat)[1]))



# Longitudinal model with speed control
# A_long = np.array([[-0.0127632, 0.353551, 0, -9.81],
#           [-1.24324, -16.4517, 11.8219, 0],
#           [0, -7.87678, -21.3803, 0],
#           [0, 0, 1, 0]])
# B_long = np.array([[0.02104349,  0.5],
#           [-28.0431,  0.1],
#           [-284.0878, 0],
#           [0, 0]])
# C_long = np.array([[0, 0, 0, 1],
#                    [1, 0, 0, 0]])
# D_long = np.zeros((np.shape(C_long)[0], np.shape(B_long)[1]))


# Create State-Space system
ss_long = control.ss(A_long, B_long, C_long, D_long)
ss_lat = control.ss(A_lat, B_lat, C_lat, D_lat)

# Check controllability
Controllability_long = np.linalg.matrix_rank(control.ctrb(A_long, B_long)) == np.linalg.matrix_rank(A_long)
Controllability_lat = np.linalg.matrix_rank(control.ctrb(A_lat, B_lat)) == np.linalg.matrix_rank(A_lat)
print(f"Longitudinal model is controllable: {Controllability_long}")
print(f"Lateral model is controllable: {Controllability_lat}")

# Check observability
Observability_long = np.linalg.matrix_rank(control.obsv(A_long, C_long)) == 4
Observability_lat = np.linalg.matrix_rank(control.obsv(A_lat, C_lat)) == 4
print(f"Longitudinal model is observable: {Observability_long}")
print(f"Lateral model is observable: {Observability_lat}")

# Eigen analysis for longitudinal model
eig_long = control.damp(ss_long)
poles_long, zeros_long = control.pzmap(ss_long, plot=False)
plotZP(poles_long, zeros_long, "Longitudinal Poles and Zeros")

# Eigen analysis for lateral model
eig_lat = control.damp(ss_lat)
poles_lat, zeros_lat = control.pzmap(ss_lat, plot=False)
plotZP(poles_lat, zeros_lat, "Lateral Poles and Zeros")

# Longitudinal weighting matrices
p_long = 100
Q_long = p_long*np.matmul(np.transpose(C_long), C_long)
R_long = np.eye(np.shape(C_long)[0])
# Longitudinal LQR gains
K_long, _, _ = control.lqr(A_long, B_long, Q_long, R_long)
Nbar_long = precom(A_long, B_long, C_long, K_long)


# Lateral weighting matrices
p_lat = 30
Q_lat = p_lat*np.matmul(np.transpose(C_lat), C_lat)
R_lat = np.eye(np.shape(C_lat)[0])
# Lateral LQR gains
K_lat, _, _ = control.lqr(A_lat, B_lat, Q_lat, R_lat)
Nbar_lat = precom(A_lat, B_lat, C_lat, K_lat)