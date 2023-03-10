"""Tests for the inverse kinematics. 

Forward kinematics (FK) outputs one end-effector (EEF) pose for one joint configuration i.e. FK has an unique solution.

Inverse kinematics (IK) is different. 
For the UR robots, a single EEF pose can have up to 8 solutions for the joint angles.

In the following, we use q to denote 6 joint angles and X to denote an EEF pose.

There's several properties that we want to test for the IK:
* Consistency: The FK of all solutions of an IK call should be the same.
* Correctness: FK(IK(X)) = X for all IK solutions. 
* Inclusion: q should be one of the solutions of IK(FK(q))
* Completeness: The IK should return all possible solutions.
* Strictness: Unreachable EEF poses should return no solutions.

Additionally there's some conventions that we want to follow:
* Range: The solutions should be in the range [0, 2pi[

We also have a test with an axis-aligned EEF pose because some solvers seem to have problems with that.
See: https://github.com/cambel/ur_ikfast/issues/4

To get an EEF pose X that is guaranteed to be reachable, we can run X = F(q) for a q with the joint limits.

Note: If there's a NaN a row of the IK solution, it means that that row should be ignored
"""
import pytest
from ur_analytic_ik import ur5e
import numpy as np


@pytest.fixture(autouse=True)
def seed():
    """Fixture that will run before each test in this module to make them deterministic."""
    np.random.seed(0)


def test_consistency():
    for _ in range(10000):
        random_joints = np.random.uniform(-2 * np.pi, 2 * np.pi, 6)
        eef_pose = ur5e.forward_kinematics(*random_joints)
        joint_solutions = ur5e.inverse_kinematics(np.array(eef_pose))

        if len(joint_solutions) <= 1:
            continue  # A single solution is always consistent with itself

        eef_pose0 = ur5e.forward_kinematics(*joint_solutions[0].T)
        for joints in joint_solutions[1:]:
            eefpose = ur5e.forward_kinematics(*joints.T)
            assert np.allclose(eef_pose0, eefpose)


def test_correctness():
    for _ in range(10000):
        random_joints = np.random.uniform(-2 * np.pi, 2 * np.pi, 6)
        original_eef_pose = np.array(ur5e.forward_kinematics(*random_joints))
        joint_solutions = ur5e.inverse_kinematics(original_eef_pose)

        for joints in joint_solutions:
            eef_pose = np.array(ur5e.forward_kinematics(*joints.T))
            assert np.allclose(eef_pose, original_eef_pose)


def test_inclusion():
    for _ in range(10000):
        random_joints = np.random.uniform(-np.pi, np.pi, 6)  # Smaller range for equality check to work
        eef_pose = ur5e.forward_kinematics(*random_joints)
        joint_solutions = ur5e.inverse_kinematics(np.array(eef_pose))

        # random_joints should be one of the solutions
        assert np.any([np.allclose(random_joints, joints) for joints in joint_solutions])


def test_strictness():
    unreachable_pose = np.identity(4)
    unreachable_pose[0, 3] = 5.0
    joint_solutions = ur5e.inverse_kinematics(unreachable_pose)
    assert len(joint_solutions) == 0


# Not sure yet if/how  we can test completeness.
# def test_completeness():
#     pass


def test_range():
    for _ in range(10000):
        random_joints = np.random.uniform(2 * np.pi, 2 * np.pi, 6)
        eef_pose = ur5e.forward_kinematics(*random_joints)
        joint_solutions = ur5e.inverse_kinematics(np.array(eef_pose))

        for joints in joint_solutions:
            print(joints)
            assert np.all(joints >= -np.pi)
            assert np.all(joints <= np.pi)


def test_axis_aliged_eef_pose():
    easily_reachable_pose = np.identity(4)
    X = np.array([-1.0, 0.0, 0.0])
    Y = np.array([0.0, 1.0, 0.0])
    Z = np.array([0.0, 0.0, -1.0])
    top_down_orientation = np.column_stack([X, Y, Z])

    translation = np.array([-0.2, -0.2, 0.2])

    easily_reachable_pose[:3, :3] = top_down_orientation
    easily_reachable_pose[:3, 3] = translation

    joint_solutions = ur5e.inverse_kinematics(np.array(easily_reachable_pose))

    # We chose this "easy" pose where all UR robots should have 8 IK solutions
    assert len(joint_solutions) == 8

    for joints in joint_solutions:
        eef_pose = np.array(ur5e.forward_kinematics(*joints.T))
        assert np.allclose(easily_reachable_pose, eef_pose)


def test_closest():
    for _ in range(10000):
        random_joints = np.random.uniform(-2 * np.pi, 2 * np.pi, 6)
        original_eef_pose = np.array(ur5e.forward_kinematics(*random_joints))

        joint_solutions = ur5e.inverse_kinematics(original_eef_pose)

        random_closest_joints = np.random.uniform(-np.pi, np.pi, 6)  # Smaller range for equality check to work
        joint_solutions_closest = ur5e.inverse_kinematics_closest(original_eef_pose, *random_closest_joints)
        joint_solutions_closest = joint_solutions_closest[0]  # We know there should be at least one solution
        closest_distance = np.linalg.norm(joint_solutions_closest - random_closest_joints)

        # Test that closest solution is actually the closest of all solutions.
        for joints in joint_solutions:
            assert closest_distance <= np.linalg.norm(joints - random_closest_joints)


def test_with_tcp():
    tcp_transform = np.identity(4)
    tcp_transform[2, 3] = 0.2

    zeros = np.zeros(6)

    tcp_pose = ur5e.forward_kinematics_with_tcp(*zeros, tcp_transform)
    joint_solutions = ur5e.inverse_kinematics_with_tcp(np.array(tcp_pose), tcp_transform)

    # Check whether all joints are close to zero or two pi.
    two_pi = 2.0 * np.pi * np.ones(6)
    assert np.any([np.logical_or(np.isclose(zeros, joints), np.isclose(two_pi, joints)) for joints in joint_solutions])
