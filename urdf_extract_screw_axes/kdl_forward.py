import numpy as np

import PyKDL
from kdl_parser_py import urdf


class KDLForward(object):
    def __init__(self, model, base, end):
        is_ok, self._tree = urdf.treeFromUrdfModel(model, quiet=True)
        if not is_ok:
            raise ValueError("Could not construct a KDL tree from URDF model")
        self._base = base
        self._end = end
        self._chain = self._tree.getChain(self._base, self._end)
        self._fk_solver = PyKDL.ChainFkSolverPos_recursive(self._chain)
        self._jac_solver = PyKDL.ChainJntToJacSolver(self._chain)
        return

    def forward(self, qarr, ee_link=None):
        g = self._do_kdl_fk(qarr, ee_link)
        if g is None:
            print "[ERROR] could not compute forward kinematics"
        return g

    def jacobian(self, qarr):
        return self._do_kdl_jacobian(qarr)

    def _joint_list_to_kdl(self, q):
        if q is None:
            return None
        if isinstance(q, np.matrix) and q.shape[1] == 0:
            q = q.T.tolist()[0]
        q_kdl = PyKDL.JntArray(len(q))
        for i, q_i in enumerate(q):
            q_kdl[i] = q_i
        return q_kdl

    def _kdl_to_numpy(self, m):
        mat = np.array(np.zeros((m.rows(), m.columns())))
        for i in range(m.rows()):
            for j in range(m.columns()):
                mat[i, j] = m[i, j]
        return mat

    def _do_kdl_jacobian(self, q):
        j_kdl = PyKDL.Jacobian(self._chain.getNrOfJoints())
        q_kdl = self._joint_list_to_kdl(list(q))
        self._jac_solver.JntToJac(q_kdl, j_kdl)
        return self._kdl_to_numpy(j_kdl)

    def _do_kdl_fk(self, q, ee_link=None):
        endeffec_frame = PyKDL.Frame()
        if ee_link is None:
            chain = self._chain
            fk_solver = self._fk_solver
        else:
            chain = self._tree.getChain(self._base, ee_link)
            fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
        qmod = q[0:chain.getNrOfJoints()]
        kinematics_status = fk_solver.JntToCart(self._joint_list_to_kdl(qmod),
                                                endeffec_frame)
        if kinematics_status >= 0:
            p = endeffec_frame.p
            M = endeffec_frame.M
            return np.array([
                [M[0, 0], M[0, 1], M[0, 2], p.x()],
                [M[1, 0], M[1, 1], M[1, 2], p.y()],
                [M[2, 0], M[2, 1], M[2, 2], p.z()],
                [0, 0, 0, 1]])
        else:
            return None
