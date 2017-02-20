import numpy as np
import modern_robotics as mr

sq2 = np.sqrt(2)/2.


######################################
# RIGHT ARM SPATIAL SCREWS WRT /base #
######################################
# Joint 1 
w1 = np.array([0,0,1])
q1 = np.array([0.06402724, -0.25902738, 0])
v1 = -np.cross(w1,q1)
S1 = np.append(w1,v1)
# Joint 2 
w2 = np.array([sq2, sq2, 0])
q2 = np.array([0.11281752, -0.30781784, 0.399976])
v2 = -np.cross(w2,q2)
S2 = np.append(w2,v2)
# Joint 3 
w3 = np.array([sq2, -sq2, 0])
q3 = np.array([0.18494228, -0.37994287, 0.399976])
v3 = -np.cross(w3,q3)
S3 = np.append(w3,v3)
# Joint 4 
w4 = np.array([sq2, sq2, 0])
q4 = np.array([0.3705009, -0.56550217, 0.330976])
v4 = -np.cross(w4,q4)
S4 = np.append(w4,v4)
# Joint 5 
w5 = np.array([sq2, -sq2, 0])
q5 = np.array([0.44374996, -0.63875149, 0.330976])
v5 = -np.cross(w5,q5)
S5 = np.append(w5,v5)
# Joint 6 
w6 = np.array([sq2, sq2, 0])
q6 = np.array([0.63516341, -0.83016565, 0.320976])
v6 = -np.cross(w6,q6)
S6 = np.append(w6,v6)
# Joint 7 
w7 = np.array([sq2, -sq2, 0])
q7 = np.array([0.71716997, -0.91217251, 0.320976])
v7 = -np.cross(w7,q7)
S7 = np.append(w7,v7)

# assemble screws:
joints = [S1, S2, S3, S4, S5, S6, S7]
Slist_right = np.array(joints).T
M0_brh = np.zeros((4,4))
M0_brh[0:3,0:3] = np.array([[ 0,  sq2,  sq2],
                            [ 0,  sq2, -sq2],
                            [-1,  0,    0]])
M0_brh[0:3,-1] = np.array([0.7974618, -0.99246463, 0.320976])
M0_brh[3,3] = 1

###############################
# BODY SCREWS WRT /right_hand #
###############################
M0_rhb = mr.TransInv(M0_brh)
Blist_right = np.zeros(Slist_right.shape)
for i,s in enumerate(Slist_right.T):
    Blist_right[:,i] = np.dot(mr.Adjoint(M0_rhb), s)



######################################
# LEFT ARM SPATIAL SCREWS WRT /base #
######################################
# Joint 1 
w1 = np.array([0,0,1])
q1 = np.array([0.06402724, 0.25902738, 0])
v1 = -np.cross(w1,q1)
S1 = np.append(w1,v1)
# Joint 2 
w2 = np.array([-sq2, sq2, 0])
q2 = np.array([0.11281752, 0.30781784, 0.399976])
v2 = -np.cross(w2,q2)
S2 = np.append(w2,v2)
# Joint 3 
w3 = np.array([sq2, sq2, 0])
q3 = np.array([0.18494228, 0.37994287, 0.399976])
v3 = -np.cross(w3,q3)
S3 = np.append(w3,v3)
# Joint 4 
w4 = np.array([-sq2, sq2, 0])
q4 = np.array([0.3705009, 0.56550217, 0.330976])
v4 = -np.cross(w4,q4)
S4 = np.append(w4,v4)
# Joint 5 
w5 = np.array([sq2, sq2, 0])
q5 = np.array([0.44374996, 0.63875149, 0.330976])
v5 = -np.cross(w5,q5)
S5 = np.append(w5,v5)
# Joint 6 
w6 = np.array([-sq2, sq2, 0])
q6 = np.array([0.63516341, 0.83016565, 0.320976])
v6 = -np.cross(w6,q6)
S6 = np.append(w6,v6)
# Joint 7 
w7 = np.array([sq2, sq2, 0])
q7 = np.array([0.71716997, 0.91217251, 0.320976])
v7 = -np.cross(w7,q7)
S7 = np.append(w7,v7)

# assemble screws:
joints = [S1, S2, S3, S4, S5, S6, S7]
Slist_left = np.array(joints).T
M0_blh = np.zeros((4,4))
M0_blh[0:3,0:3] = np.array([[ 0, -sq2, sq2],
                            [ 0,  sq2, sq2],
                            [-1,  0,   0]])
M0_blh[0:3,-1] = np.array([0.7974618, 0.99246463, 0.320976])
M0_blh[3,3] = 1


###############################
# BODY SCREWS WRT /left_hand #
###############################
M0_lhb = mr.TransInv(M0_blh)
Blist_left = np.zeros(Slist_left.shape)
for i,s in enumerate(Slist_left.T):
    Blist_left[:,i] = np.dot(mr.Adjoint(M0_lhb), s)
