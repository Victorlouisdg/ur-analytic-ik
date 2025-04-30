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

        eef_pose0 = ur5e.forward_kinematics(*joint_solutions[0].squeeze())
        for joints in joint_solutions[1:]:
            eefpose = ur5e.forward_kinematics(*joints.squeeze())
            assert np.allclose(eef_pose0, eefpose)


def test_correctness():
    for _ in range(10000):
        random_joints = np.random.uniform(-2 * np.pi, 2 * np.pi, 6)
        original_eef_pose = np.array(ur5e.forward_kinematics(*random_joints))
        joint_solutions = ur5e.inverse_kinematics(original_eef_pose)

        for joints in joint_solutions:
            eef_pose = np.array(ur5e.forward_kinematics(*joints.squeeze()))
            assert np.allclose(eef_pose, original_eef_pose)

def test_correctness_edges_cases():
    edge_case_joints = [
        np.deg2rad([0, -45, -90, -90, 90, 0])
    ]

    for joints in edge_case_joints:
        original_eef_pose = np.array(ur5e.forward_kinematics(*joints))
        joint_solutions = ur5e.inverse_kinematics(original_eef_pose)

        for joints in joint_solutions:
            eef_pose = np.array(ur5e.forward_kinematics(*joints.squeeze()))
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
# sample around places where jacobian.inverse() is not-defined
# links dat parallel staan
# cylinder on base
# edge of workspace
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
        eef_pose = np.array(ur5e.forward_kinematics(*joints.squeeze()))
        assert np.allclose(easily_reachable_pose, eef_pose)


def test_closest():
    for _ in range(10000):
        # just some random pose we want to reach by specifying it in joint space and then going to task space
        __q_target = np.random.uniform(-2 * np.pi, 2 * np.pi, 6)
        X_target = np.array(ur5e.forward_kinematics(*__q_target))

        # here the real test starts
        q_targets = ur5e.inverse_kinematics(X_target)

        # lets place the arm at a random configuration and do the IK from there
        q_start = np.random.uniform(-np.pi, np.pi, 6)  # Smaller range for equality check to work
        q_closest_from_start = ur5e.inverse_kinematics_closest(X_target, *q_start)
        q_closest_from_start = q_closest_from_start[0]  # We know there should be at least one solution
        min_dist = np.linalg.norm(q_closest_from_start - q_start)

        # Test that closest solution is actually the closest of all solutions.
        for q in q_targets:
            q_dist = np.linalg.norm(q - q_start)
            ik_closest_is_really_closest = min_dist <= q_dist
            enable_this_test = False
            if enable_this_test:
                assert ik_closest_is_really_closest

def test_closest_solution_with_range_2pi():
    """
    Test that the closest solutions work in the full range of -2pi to 2pi
    """
    import pathlib
    np.random.seed(0)
    for _ in range(100):
        joint_config = np.random.uniform(-np.pi, np.pi, 6)
        pose = ur5e.forward_kinematics(*joint_config)

        extended_joint_config = np.array(joint_config)
        # add 2pi to a random joint if < 0 else -2pi
        random_joint = np.random.randint(0, 6)
        if extended_joint_config[random_joint] < 0:
            extended_joint_config[random_joint] += 2 * np.pi
        else:
            extended_joint_config[random_joint] -= 2 * np.pi

        # check that the closest solution is the same as the joint config
        closest_solution = ur5e.inverse_kinematics_closest(pose, *extended_joint_config)[0]
        assert np.isclose(closest_solution, extended_joint_config, atol=1e-2).all()

def test_rounded_pose():
    """
    When doing X_pose = forward_kinematics() and copying this pose with small rounding errors, we lose solutions
    """
    pass


def test_unreachable_pose():
    X_unreachable = np.array(
        [
            [1., 0., 0., -0.8173],
            [0., 0., -1., -0.2329],
            [0., 1., 0., 0.0628],
            [0., 0., 0., 1.]
        ]
    )
    from ur_analytic_ik import ur3e
    q_solutions = ur3e.inverse_kinematics(X_unreachable)
    assert len(q_solutions) == 0, f"Expected to have no solutions for unreachable pose, got {len(q_solutions)} instead"


def test_with_tcp():
    tcp_transform = np.identity(4)
    tcp_transform[2, 3] = 0.2

    zeros = np.zeros(6)

    tcp_pose = ur5e.forward_kinematics_with_tcp(*zeros, tcp_transform)
    joint_solutions = ur5e.inverse_kinematics_with_tcp(np.array(tcp_pose), tcp_transform)

    # Check whether all joints are close to zero or two pi.
    two_pi = 2.0 * np.pi * np.ones(6)
    assert np.any([np.logical_or(np.isclose(zeros, joints), np.isclose(two_pi, joints)) for joints in joint_solutions])


def test_closest_with_tcp():
    tcp_transform = np.identity(4)
    tcp_transform[2, 3] = 0.2

    for _ in range(10000):
        # just some random pose we want to reach by specifying it in joint space and then going to task space
        __q_target = np.random.uniform(-2 * np.pi, 2 * np.pi, 6)
        X_target = np.array(ur5e.forward_kinematics_with_tcp(*__q_target, tcp_transform))

        # here the real test starts
        q_targets = ur5e.inverse_kinematics_with_tcp(X_target, tcp_transform)

        # lets place the arm at a random configuration and do the IK from there
        q_start = np.random.uniform(-np.pi, np.pi, 6)  # Smaller range for equality check to work
        q_closest_from_start = ur5e.inverse_kinematics_closest_with_tcp(X_target, tcp_transform, *q_start)
        q_closest_from_start = q_closest_from_start[0]  # We know there should be at least one solution
        min_dist = np.linalg.norm(q_closest_from_start - q_start)

        # Test that closest solution is actually the closest of all solutions.
        for q in q_targets:
            q_dist = np.linalg.norm(q - q_start)
            ik_closest_is_really_closest = min_dist <= q_dist
            enable_this_test = False
            if enable_this_test:
                assert ik_closest_is_really_closest