def test_arguments_w_default_values():
    """
    test if the FK function works with default arguments
    """
    import numpy as np
    from ur_analytic_ik import ur5e
    joint_config = np.array([0,0,0,0,0,0])
    fk_pose = ur5e.forward_kinematics(joint_config)

    assert isinstance(fk_pose, np.ndarray), f"Expected a numpy array, got {type(fk_pose)}"
    assert fk_pose.shape == (4,4), f"Expected a 4x4 matrix, got {fk_pose.shape}"

def test_arguments_w_tcp_pose():
    """
    test if the FK function works with a custom TCP pose
    """
    import numpy as np
    from ur_analytic_ik import ur5e
    joint_config = np.array([0,0,0,0,0,0])
    tcp_pose = np.array([[1,0,0,0.1],
                          [0,1,0,0.1],
                          [0,0,1,0.1],
                          [0,0,0,1]])
    fk_pose = ur5e.forward_kinematics(joint_config, tcp_pose)

    assert isinstance(fk_pose, np.ndarray), f"Expected a numpy array, got {type(fk_pose)}"
    assert fk_pose.shape == (4,4), f"Expected a 4x4 matrix, got {fk_pose.shape}"
