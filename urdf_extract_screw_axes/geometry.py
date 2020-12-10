from math import pi
import numpy as np
from functools import reduce


def matmult(*x):
    """
    Shortcut for standard matrix multiplication.
    matmult(A,B,C) returns A*B*C.
    """
    return reduce(np.dot, x)


def rotx(t):
    ct = np.cos(t)
    st = np.sin(t)
    return np.array([
        [1, 0, 0],
        [0, ct, -st],
        [0, st, ct]
    ])


def roty(t):
    ct = np.cos(t)
    st = np.sin(t)
    return np.array([
        [ct, 0, st],
        [0, 1, 0],
        [-st, 0, ct]
    ])


def rotz(t):
    ct = np.cos(t)
    st = np.sin(t)
    return np.array([
        [ct, -st, 0],
        [st, ct, 0],
        [0, 0, 1]
    ])


def eul2so3_zyzr(phi, theta, psi):
    return matmult(rotz(phi), roty(theta), rotz(psi))


def eul2so3_xyzr(alpha, beta, gamma):
    return matmult(rotx(alpha), roty(beta), rotz(gamma))


def so3andp2se3(R, p):
    g = np.vstack((
        np.hstack((R, p.reshape(3, 1))),
        np.array([0, 0, 0, 1])))
    return g


def se32so3andp(g):
    return np.array(g[0:3, 0:3]), np.array(g[0:3, -1]).flatten()


def hat(w):
    return np.array([[0, -w[2], w[1]],
                     [w[2], 0, -w[0]],
                     [-w[1], w[0], 0]])


def hat6(v):
    return np.r_[np.c_[hat([v[0], v[1], v[2]]), [v[3], v[4], v[5]]],
                 np.zeros((1, 4))]


def unhat(what):
    return np.array([what[2][1], what[0][2], what[1][0]])


def unhat6(vhat):
    return np.r_[[vhat[2][1], vhat[0][2], vhat[1][0]],
                 [vhat[0][3], vhat[1][3], vhat[2][3]]]


def adjoint(g):
    R, p = se32so3andp(g)
    return np.r_[np.c_[R, np.zeros((3, 3))],
                 np.c_[matmult(hat(p), R), R]]


def matrixlog3(R):
    if np.isclose(np.linalg.norm(R - np.eye(3)), 0.0):
        return np.zeros(3, 3)
    elif np.isclose(np.trace(R) + 1, 0.0):
        if not np.isclose(1 + R[2][2], 0.0):
            omg = (1.0 / np.sqrt(2 * (1 + R[2][2]))) \
                * np.array([R[0][2], R[1][2], 1 + R[2][2]])
        elif not np.isclose(1 + R[1][1], 0.0):
            omg = (1.0 / np.sqrt(2 * (1 + R[1][1]))) \
                * np.array([R[0][1], 1 + R[1][1], R[2][1]])
        else:
            omg = (1.0 / np.sqrt(2 * (1 + R[0][0]))) \
                * np.array([1 + R[0][0], R[1][0], R[2][0]])
        return hat(pi * omg)
    else:
        arg = (np.trace(R) - 1) / 2.0
        np.clip(arg, -1, 1)
        theta = np.arccos(arg)
        return theta / 2.0 / np.sin(theta) * (R - np.array(R).T)


def matrixlog6(g):
    R = np.array(g)[0:3, 0:3]
    p = np.array(g)[0:3, -1]
    if np.isclose(np.linalg.norm(R - np.eye(3)), 0.0):
        return np.r_[np.c_[np.zeros((3, 3)),
                           [g[0][3], g[1][3], g[2][3]]],
                     [[0, 0, 0, 0]]]
    else:
        arg = (np.trace(R) - 1) / 2.0
        np.clip(arg, -1, 1)
        theta = np.arccos(arg)
        omgmat = matrixlog3(R)
        return np.r_[np.c_[omgmat,
                           np.dot(np.eye(3) - omgmat / 2.0
                                  + (1.0 / theta - 1.0 / np.tan(theta / 2.0) / 2)
                                  * np.dot(omgmat, omgmat) / theta, [g[0][3],
                                                                     g[1][3],
                                                                     g[2][3]])],
                     [[0, 0, 0, 0]]]


def se3_inverse(g):
    R, p = se32so3andp(g)
    R_i = np.array(R).T
    g_i = np.array(g).copy()
    g_i[0:3, 0:3] = R_i
    g_i[0:3, -1] = -matmult(R_i, p).ravel()
    return g_i
