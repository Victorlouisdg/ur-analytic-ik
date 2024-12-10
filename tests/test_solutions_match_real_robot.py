"""use a set of (joint_config, pose) pairs  collected from a real robot to test if the IK/FK solutions correspond to the real robot.
"""

import numpy as np
import pytest
from ur_analytic_ik import ur5e

def read_poses(file):
    with  open(file, "r") as f:
        lines = f.readlines()
    joint_config_pose_pairs = []
    for line in lines:
        if len(line) == 1:
            continue
        joint_config, pose = line.split(";")[0], line.split(";")[1]
        joint_config = joint_config.split(",")
        # strip the brackets
        joint_config[0] = joint_config[0][1:]
        joint_config[-1] = joint_config[-1][:-1]
        joint_config = [float(j) for j in joint_config]
        
        pose = pose.split(",")
        # remove all brackets
        for i in range(len(pose)):
            pose[i] = pose[i].replace("[", "").replace("]", "")
        pose = [float(p) for p in pose]
        pose = np.array(pose).reshape(4, 4)
        joint_config_pose_pairs.append((joint_config, pose))

    return joint_config_pose_pairs

def test_solutions_match_real_ur5e():
    import pathlib
    file_path = pathlib.Path(__file__).parent / "data/ur5e_poses.txt"
    joint_config_pose_pairs = read_poses(str(file_path))
    for joint_config, pose in joint_config_pose_pairs:
      
        joint_solution = ur5e.inverse_kinematics_closest(pose, *joint_config)[0]
        assert np.isclose(joint_solution, joint_config,atol=1e-2).all()


        fk_pose = ur5e.forward_kinematics(*joint_config)
        assert np.linalg.norm(fk_pose[:3,3] - pose[:3,3]) < 3e-3 # 3mm tolerance
        