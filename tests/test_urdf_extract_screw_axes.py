import modern_robotics as mr
from urdf_parser_py.urdf import URDF
import urdf_extract_screw_axes.screw_parser as uesa
import numpy as np
import os
import sys
import contextlib

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "manual"))

TOLERANCE = 1e-5
NUM = 20

@contextlib.contextmanager
def printoptions(*args, **kwargs):
    orig = np.get_printoptions()
    np.set_printoptions(*args, **kwargs)
    yield
    np.set_printoptions(**orig)

def arrprint(g):
    with printoptions(formatter={'float': '{: 0.3f}'.format}, suppress=True):
        print g
    return


def run_spatial_screw(fname, base, end, Slist_manual, M_manual, decimal=5):
    print "============================================================"
    fullname = os.path.join(os.path.dirname(__file__),fname)
    robot = URDF.from_xml_file(fullname)
    s = uesa.ScrewParser(robot, "base", "right_hand")
    M0, Slist = s.get_spatial_description()
    print "File = ",fname
    print "Base = ",base
    print "End = ",end
    for i, (s_this,s_manual) in enumerate(zip(Slist.T, Slist_manual.T)):
        print "Joint",i
        print "Manual   ",
        arrprint(s_manual)
        print "Automatic",
        arrprint(s_this)
        print
        np.testing.assert_array_almost_equal(s_this, s_manual, decimal=decimal)
    print "SE(3) Test:"
    print "Manual   ",
    arrprint(M_manual)
    print "Automatic",
    arrprint(M0)
    print
    np.testing.assert_array_almost_equal(M0, M_manual, decimal=decimal)
    print "============================================================"
    return


def run_body_screw(fname, base, end, Blist_manual, M_manual, decimal=5):
    print "============================================================"
    fullname = os.path.join(os.path.dirname(__file__),fname)
    robot = URDF.from_xml_file(fullname)
    s = uesa.ScrewParser(robot, "base", "right_hand")
    M0, Blist = s.get_body_description()
    print "File = ",fname
    print "Base = ",base
    print "End = ",end
    for i, (s_this,s_manual) in enumerate(zip(Blist.T, Blist_manual.T)):
        print "Joint",i
        print "Manual   ",
        arrprint(s_manual)
        print "Automatic",
        arrprint(s_this)
        print
        np.testing.assert_array_almost_equal(s_this, s_manual, decimal=decimal)
    print "SE(3) Test:"
    print "Manual   ",
    arrprint(M_manual)
    print "Automatic",
    arrprint(M0)
    print
    np.testing.assert_array_almost_equal(M0, M_manual, decimal=decimal)
    print "============================================================"
    return


def baxter_screw_test():
    import baxter_MR_description as bmr
    fname = "urdf/baxter.urdf"
    run_spatial_screw(fname, "base", "right_hand", bmr.Slist_right, bmr.M0_brh)
    run_body_screw(fname, "base", "right_hand", bmr.Blist_right, bmr.M0_brh)
    return

