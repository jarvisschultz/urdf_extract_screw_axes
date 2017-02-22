import numpy as np
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
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
        self._chain = None
        self._free_chain = None
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
        self._chain = KDLKinematics(self._robot, self._base, self._end)
        self._free_chain = self._chain.get_joint_names()
        self._n = self._chain.num_joints
        # chain = self._robot.get_chain(self._base, self._end, joints=True, links=False, fixed=True)
        # free_chain = []
        # n = 0
        # for j in chain:
        #     if j.type != 'fixed':
        #         free_chain.append(j)
        #         n += 1
        # self._chain = chain
        # self._free_chain = free_chain
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
        M0 = np.array(self._chain.forward(np.zeros(self._n)))
        for i,j in enumerate(self._free_chain):
            joint = self._robot.joint_map[j]
            g_base_child = np.array(self._chain.forward(np.zeros(self._n), joint.child))
            if joint.type in ['revolute', 'continuous']:
                axis_child = np.array(joint.axis)
                axis_base = np.dot(g_base_child[0:3,0:3], axis_child)
                q_vec = g_base_child[0:3,-1]
                v_vec = -np.cross(axis_base, q_vec)
                Slist[:,i] = np.hstack((axis_base, v_vec))
            # elif j.type is 'prismatic':
            else:
                print "[ERROR] on joint {0:d} (name = {1:s} type = {2:s})".format(i, j.name, j.type)
                raise ValueError("Currently only support revolute and prismatic joints")
        return M0, Slist

    def get_body_description(self):
        M0, Slist = self.get_spatial_description()
        Blist = np.zeros(Slist.shape)
        M0_inv = geometry.se3_inverse(M0)
        for i,s in enumerate(Slist.T):
            Blist[:,i] = np.dot(geometry.adjoint(M0_inv), s)
        return M0, Blist
