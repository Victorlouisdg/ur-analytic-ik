from ur_analytic_ik import ur5e_forward_kinematics, ur5e_inverse_kinematics
import numpy as np

def test_IK():
    for i in range(100000):
        # Sample 6 random joint angles
        joints = np.random.uniform(0, 2 * np.pi, 6)
        eef_pose = ur5e_forward_kinematics(*joints)
        solutions = ur5e_inverse_kinematics(np.array(eef_pose))
        n_valid_solutions = 0
        for solution in solutions:
            # Check that the solution is within the joint limits
            if np.isnan(np.sum(solution)):
                continue # Rows with NaN mean that solution is not valid and should be ignored
            n_valid_solutions += 1
            if not np.all(solution >= 0):
                print(solution)

            assert np.all(solution >= 0)
            assert np.all(solution <= 2 * np.pi)
            # Check that the solution is a solution
            solution_eef_pose = ur5e_forward_kinematics(*solution)
            assert np.allclose(solution_eef_pose, eef_pose)
        print(i, n_valid_solutions)


if __name__ == "__main__":
    test_IK()

