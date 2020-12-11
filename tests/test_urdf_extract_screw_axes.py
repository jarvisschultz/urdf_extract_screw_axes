import modern_robotics as mr
from urdf_parser_py.urdf import URDF
import urdf_extract_screw_axes.screw_parser as uesa
from urdf_extract_screw_axes.utils import suppress_stdout_stderr
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
    fullname = os.path.join(os.path.dirname(__file__), fname)
    with suppress_stdout_stderr():
        robot = URDF.from_xml_file(fullname)
    s = uesa.ScrewParser(robot, base, end)
    M0, Slist = s.get_spatial_description()
    print "File = ", fname
    print "Base = ", base
    print "End = ", end
    for i, (s_this, s_manual) in enumerate(zip(Slist.T, Slist_manual.T)):
        print "Joint", i, "spatial screw axes"
        print "    Manual   ",
        arrprint(s_manual)
        print "    Automatic",
        arrprint(s_this)
        print
        np.testing.assert_array_almost_equal(s_this, s_manual, decimal=decimal)
    print "SE(3) Test:"
    print "    Manual   "
    arrprint(M_manual)
    print "    Automatic"
    arrprint(M0)
    print
    np.testing.assert_array_almost_equal(M0, M_manual, decimal=decimal)
    print "============================================================"
    return


def run_body_screw(fname, base, end, Blist_manual, M_manual, decimal=5):
    print "============================================================"
    fullname = os.path.join(os.path.dirname(__file__), fname)
    with suppress_stdout_stderr():
        robot = URDF.from_xml_file(fullname)
    s = uesa.ScrewParser(robot, base, end)
    M0, Blist = s.get_body_description()
    print "File = ", fname
    print "Base = ", base
    print "End = ", end
    for i, (s_this, s_manual) in enumerate(zip(Blist.T, Blist_manual.T)):
        print "Joint", i, "body screw axes"
        print "    Manual   ",
        arrprint(s_manual)
        print "    Automatic",
        arrprint(s_this)
        print
        np.testing.assert_array_almost_equal(s_this, s_manual, decimal=decimal)
    print "SE(3) Test:"
    print "    Manual   "
    arrprint(M_manual)
    print "    Automatic"
    arrprint(M0)
    print
    np.testing.assert_array_almost_equal(M0, M_manual, decimal=decimal)
    print "============================================================"
    return


def run_fk_spatial(fname, base, end, Slist_manual, M_manual, decimal=5, num=100):
    fullname = os.path.join(os.path.dirname(__file__), fname)
    with suppress_stdout_stderr():
        robot = URDF.from_xml_file(fullname)
    s = uesa.ScrewParser(robot, base, end)
    M0, Slist = s.get_spatial_description()
    for i in range(num):
        q = np.random.uniform(-np.pi, np.pi, Slist.shape[-1])
        g_manual = mr.FKinSpace(M_manual, Slist_manual, q)
        g_auto = mr.FKinSpace(M0, Slist, q)
        if not np.all(np.isclose(g_manual, g_auto, atol=1e-5)):
            print "============================================================"
            print "FK Spatial Test:"
            print "File = ", fname
            print "Base = ", base
            print "End = ", end
            print "Configuration: ",
            arrprint(q)
            print "    SE(3) from base to end manual:   "
            arrprint(g_manual)
            print "    SE(3) from base to end auto:   "
            arrprint(g_auto)
            print
            np.testing.assert_array_almost_equal(g_manual, g_auto, decimal=decimal)
            print "============================================================"
    return


def run_fk_body(fname, base, end, Blist_manual, M_manual, decimal=5, num=100):
    fullname = os.path.join(os.path.dirname(__file__), fname)
    with suppress_stdout_stderr():
        robot = URDF.from_xml_file(fullname)
    s = uesa.ScrewParser(robot, base, end)
    M0, Blist = s.get_body_description()
    for i in range(num):
        q = np.random.uniform(-np.pi, np.pi, Blist.shape[-1])
        g_manual = mr.FKinBody(M_manual, Blist_manual, q)
        g_auto = mr.FKinBody(M0, Blist, q)
        if not np.all(np.isclose(g_manual, g_auto, atol=1e-5)):
            print "============================================================"
            print "FK Body Test:"
            print "File = ", fname
            print "Base = ", base
            print "End = ", end
            print "Configuration: ",
            arrprint(q)
            print "SE(3) from base to end manual:   "
            arrprint(g_manual)
            print "SE(3) from base to end auto:   "
            arrprint(g_auto)
            print
            np.testing.assert_array_almost_equal(g_manual, g_auto, decimal=decimal)
            print "============================================================"
    return


def baxter_right_hand_screw_test():
    import baxter_MR_description as bmr
    fname = "urdf/baxter.urdf"
    run_spatial_screw(fname, "base", "right_hand", bmr.Slist_right, bmr.M0_brh)
    run_body_screw(fname, "base", "right_hand", bmr.Blist_right, bmr.M0_brh)
    return


def baxter_left_hand_screw_test():
    import baxter_MR_description as bmr
    fname = "urdf/baxter.urdf"
    run_spatial_screw(fname, "base", "left_hand", bmr.Slist_left, bmr.M0_blh)
    run_body_screw(fname, "base", "left_hand", bmr.Blist_left, bmr.M0_blh)
    return


def sawyer_screw_test():
    import sawyer_MR_description as smr
    fname = "urdf/sawyer.urdf"
    run_spatial_screw(fname, "base", "right_hand", smr.Slist, smr.M0)
    run_body_screw(fname, "base", "right_hand", smr.Blist, smr.M0)
    return


def sawyer_fk_test():
    import sawyer_MR_description as smr
    fname = "urdf/sawyer.urdf"
    run_fk_spatial(fname, "base", "right_hand", smr.Slist, smr.M0)
    run_fk_body(fname, "base", "right_hand", smr.Blist, smr.M0)
    return


def rrprrr_fk_test():
    import rrprrr_MR_description as rmr
    fname = "urdf/rrprrr.urdf"
    run_fk_spatial(fname, "space_frame", "body_frame", rmr.Slist, rmr.M0)
    run_fk_body(fname, "space_frame", "body_frame", rmr.Blist, rmr.M0)
    return

def ur5_fk_test():
    import ur5_MR_description as umr
    fname = "urdf/ur5.urdf"
    run_fk_spatial(fname, "base_link", "ee_link", umr.Slist, umr.M0)
    run_fk_body(fname, "base_link", "ee_link", umr.Blist, umr.M0)
    return
