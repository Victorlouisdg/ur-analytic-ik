import numpy as np 
from ur_analytic_ik import ur5e,ur3e, ur10e

import pytest 

@pytest.mark.parametrize("robot", [ur5e, ur3e, ur10e])
def test_non_contiguous(robot):
    """
    test if the IK functions work for non-contiguous arrays. 
    This is not trivial as Eigen expects Row-major arrays, and numpy can create non-contiguous views which causes issues when passing them to Eigen.
    """
    rx = np.array([-1,0,0,0])
    ry = np.array([0,1,0,0])
    rz = np.array([0,0,-1,0])
    t = np.array([0.2,0.2,0.1,1])

    pose = np.array([rx, ry, rz, t]).T
    joint_solutions = robot.inverse_kinematics(pose)
    assert len(joint_solutions) == 8, f"Expected 8 solutions, got {len(joint_solutions)}"

    for joints in joint_solutions:
        eef_pose = np.array(robot.forward_kinematics(*joints.squeeze()))
        assert np.allclose(eef_pose, pose), f"Expected {pose}, got {eef_pose} for one of the joint configurations"
