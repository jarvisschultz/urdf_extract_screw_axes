import numpy as np
from math import cos, sin, radians
import modern_robotics as mr

###############################
# PHYSICAL CONSTANTS (METERS) #
###############################
# Note, we use Example 4.5 to build screw representation, but we've substituted
# more accurate measurements from Figure 4.11
L1 = 0.42500
L2 = 0.39225
H1 = 0.089159
H2 = 0.09465
W1 = 0.13585 - 0.1197 + 0.093
W2 = 0.0823


#####################################
# SPATIAL SCREWS FROM BASE TO TOOL0 #
#####################################
Slist = np.array([[0, 0, 1, 0, 0, 0],
                  [0, 1, 0, -H1, 0, 0],
                  [0, 1, 0, -H1, 0, L1],
                  [0, 1, 0, -H1, 0, L1 + L2],
                  [0, 0, -1, -W1, L1 + L2, 0],
                  [0, 1, 0, H2 - H1, 0, L1 + L2]])
Slist = Slist.T
M0 = np.array([[-1, 0, 0, L1 + L2],
               [0, 0, 1, W1 + W2],
               [0, 1, 0, H1 - H2],
               [0, 0, 0, 1]])


#########################################
# CONVERT SPATIAL SCREWS TO BODY SCREWS #
#########################################
Blist = np.zeros(Slist.shape)
adj_M0_inv = mr.Adjoint(mr.TransInv(M0))
for i, s in enumerate(Slist.T):
    Blist[:, i] = np.dot(adj_M0_inv, s)
