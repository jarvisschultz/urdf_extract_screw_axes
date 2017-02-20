import modern_robotics as mr
from urdf_parser_py.urdf import URDF
import urdf_extract_screw_axes.screw_parser as uesa
import os
import sys

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "manual"))

TOLERANCE = 1e-5
NUM = 20

def baxter_spatial_screw_test():
    import baxter_MR_description as bmr
    fname = "urdf/baxter.urdf"
    robot = URDF.from_xml_file(fname)
    s = uesa.ScrewParser(robot, "base", "right_hand")
    M0, Slist = s.get_spatial_description()
    for i,s in enumerate

    return
