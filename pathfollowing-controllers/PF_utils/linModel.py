import numpy as np

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

