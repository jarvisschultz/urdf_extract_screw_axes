import numpy as np
from math import cos, sin, radians
import modern_robotics as mr

###############################
# PHYSICAL CONSTANTS (METERS) #
###############################
L = 1.0


##################################
# SPATIAL SCREWS FROM {s} TO {b} #
##################################
Slist = np.array([[0, 0, 1, 0, 0, 0],
                  [1, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 1, 0],
                  [0, 1, 0, 0, 0, 0],
                  [1, 0, 0, 0, 0, -1*L],
                  [0, 1, 0, 0, 0, 0]])
Slist = Slist.T
M0 = np.array([[1, 0, 0, 0],
               [0, 1, 0, 2*L],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])


#########################################
# CONVERT SPATIAL SCREWS TO BODY SCREWS #
#########################################
Blist = np.zeros(Slist.shape)
adj_M0_inv = mr.Adjoint(mr.TransInv(M0))
for i, s in enumerate(Slist.T):
    Blist[:,i] = np.dot(adj_M0_inv, s)


