import numpy as np
from urdf_parser_py.urdf import URDF

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

    @property.setter
    def end(self, value):
        # check that the value is in the URDF and that the base is a parent:
        if value in self._robot.link_map.keys():
            self._base = value
            self._build_chain()
        else:
            raise ValueError("end frame '%s' is not in URDF; be sure to pass a link name"%value)
        
    def _build_chain(self):
        chain = self._robot.get_chain(self._base, self._end, joints=True, links=False, fixed=True)
        free_chain = []
        n = 0
        for j in chain:
            if j.type != 'fixed':
                free_chain.append(j)
                n += 1
        self._chain = chain
        self._free_chain = free_chain
        self._n = n
        return

    def get_spatial_description(self):
        """
        Return the screw axes expressed in the spatial frame (self.base), and the
        SE(3) transform from the base to the end effector at the zero
        configuration.
        """
        Slist = np.zeros((6, self._n))
        M0 = np.zeros((4,4))
        M0[-1,1] = 1
        for j in self._chain:
    


    
    
