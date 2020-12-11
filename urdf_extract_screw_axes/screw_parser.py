import numpy as np
from urdf_parser_py.urdf import URDF
from kdl_forward import KDLForward
import geometry

class ScrewParser(object):
    """
    Create this class to build either spatial or body screws at an arbitrary
    configuration
    """
    def __init__(self, urdf, base_link, end_link):
        """
        Constructor
        @param urdf URDF object of robot... created from urdf_parser_py.urdf.URDF
        @param base_link Name of the root link of the kinematic chain
        @param end_link Name of the end link of the kinematic chain
        """
        self._robot = urdf
        self._base = base_link
        self._end = end_link
        self._n = -1
        self._kdl_model = None
        self._joint_names = None
        self._build_chain()
        return

    @property
    def base(self):
        return self._base

    @base.setter
    def base(self, value):
        # check that value is in the URDF
        if value in self._robot.link_map.keys():
            self._base = value
            self._build_chain()
        else:
            raise ValueError("base frame '%s' is not in URDF; be sure to pass a link name"%value)
        return

    @property
    def end(self):
        return self._end

    @end.setter
    def end(self, value):
        # check that the value is in the URDF and that the base is a parent:
        if value in self._robot.link_map.keys():
            self._base = value
            self._build_chain()
        else:
            raise ValueError("end frame '%s' is not in URDF; be sure to pass a link name"%value)
        return

    def _build_chain(self):
        self._kdl_model = KDLForward(self._robot, self._base, self._end)
        self._joint_names = self._robot.get_chain(self._base, self._end, links=False, fixed=False)
        self._n = len(self._joint_names)
        # chain = self._robot.get_chain(self._base, self._end, joints=True, links=False, fixed=True)
        # joint_names = []
        # n = 0
        # for j in chain:
        #     if j.type != 'fixed':
        #         joint_names.append(j)
        #         n += 1
        # self._chain = chain
        # self._joint_names = joint_names
        # self._n = n
        return

    def get_spatial_description(self):
        """
        Return the screw axes expressed in the spatial frame (self.base), and the
        SE(3) transform from the base to the end effector at the zero
        configuration.
        """
        Slist = np.zeros((6, self._n))
        # M0 = np.zeros((4,4))
        # M0[-1,1] = 1
        M0 = np.array(self._kdl_model.forward(np.zeros(self._n)))
        for i,j in enumerate(self._joint_names):
            joint = self._robot.joint_map[j]
            g_base_child = np.array(self._kdl_model.forward(np.zeros(self._n), joint.child))
            if joint.type in ['revolute', 'continuous']:
                axis_child = np.array(joint.axis)
                axis_base = np.dot(g_base_child[0:3,0:3], axis_child)
                q_vec = g_base_child[0:3,-1]
                v_vec = -np.cross(axis_base, q_vec)
                Slist[:,i] = np.hstack((axis_base, v_vec))
            elif joint.type == 'prismatic':
                axis_child = np.array(joint.axis)
                axis_base = np.dot(g_base_child[0:3,0:3], axis_child)
                Slist[:,i] = np.hstack(([0,0,0], axis_base))
            else:
                print "[ERROR] on joint {0:d} (name = {1:s} type = {2:s})".format(i, joint.name, joint.type)
                raise ValueError("Currently only support revolute, continuous, and prismatic joints")
        return M0, Slist

    def get_body_description(self):
        M0, Slist = self.get_spatial_description()
        Blist = np.zeros(Slist.shape)
        M0_inv = geometry.se3_inverse(M0)
        for i,s in enumerate(Slist.T):
            Blist[:,i] = np.dot(geometry.adjoint(M0_inv), s)
        return M0, Blist
