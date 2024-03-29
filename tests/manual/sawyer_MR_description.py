import numpy as np
from math import cos, sin, radians
import modern_robotics as mr

################################
# STORE RE-USED TRIG FUNCTIONS #
################################
s10 = sin(radians(10))
c10 = cos(radians(10))

#######################################
# BODY SCREWS FROM BASE TO RIGHT_HAND #
#######################################
Blist = np.array([[s10, -c10, 0., -1.0155 * c10, -1.0155 * s10, -0.1603],
                  [-c10, -s10, 0., -0.9345 * s10, 0.9345 * c10, 0.],
                  [0., 0., 1., -0.0322 * s10, 0.0322 * c10, 0.],
                  [-c10, -s10, 0., -0.5345 * s10, 0.5345 * c10, 0.],
                  [0., 0., 1., 0.1363 * s10, -0.1363 * c10, 0.],
                  [-c10, -s10, 0., -0.1345 * s10, 0.1345 * c10, 0.],
                  [0., 0., 1., 0., 0., 0.]])
Blist = Blist.T


M0 = np.array([[0., 0., 1., 1.0155],
                   [-c10, -s10, 0., 0.1603],
                   [s10, -c10, 0., 0.317],
                   [0., 0., 0., 1.]])


#########################################
# CONVERT BODY SCREWS TO SPATIAL SCREWS #
#########################################
Slist = np.zeros(Blist.shape)
for i, s in enumerate(Blist.T):
    Slist[:,i] = np.dot(mr.Adjoint(M0), s)
