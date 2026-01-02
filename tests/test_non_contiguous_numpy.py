import numpy as np 
from ur_analytic_ik import ur3, ur3e, ur5, ur5e, ur7e, ur8long, ur10, ur10e, ur12e, ur15, ur16e, ur18, ur20, ur30

import pytest 


UR_MODULES = [
    ur3, ur3e, ur5, ur5e, ur7e, ur8long,
    ur10, ur10e, ur12e, ur15, ur16e,
    ur18, ur20, ur30,
]

@pytest.mark.parametrize("ur", UR_MODULES)
def test_non_contiguous(ur):
    """
    test if the IK functions work for non-contiguous arrays. 
    This is not trivial as Eigen expects Row-major arrays, and numpy can create non-contiguous views which causes issues when passing them to Eigen.
    """
    rx = np.array([-1,0,0,0])
    ry = np.array([0,1,0,0])
    rz = np.array([0,0,-1,0])
    t = np.array([0.2,0.2,0.1,1])

    pose = np.array([rx, ry, rz, t]).T
    joint_solutions = ur.inverse_kinematics(pose)

    # Normalize the module name (handle extension module names like 'ur_analytic_ik_ext.ur3')
    module_short = ur.__name__.split('.')[-1]
    if module_short in ('ur3', 'ur3e', 'ur5', 'ur5e', 'ur7e'):
        assert len(joint_solutions) == 8, f"Expected 8 solutions for {ur.__name__}, got {len(joint_solutions)}"
    else:
        assert len(joint_solutions) >= 4, f"Expected at least 4 solutions for {ur.__name__}, got {len(joint_solutions)}"


    for joints in joint_solutions:
        eef_pose = np.array(ur.forward_kinematics(*joints.squeeze()))
        assert np.allclose(eef_pose, pose), f"Expected {pose}, got {eef_pose} for one of the joint configurations"
