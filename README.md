UR Analytic IK
================
C++ implementation with Python bindings of analytic forward and inverse kinematics for the Universal Robots based on [Alternative Inverse Kinematic Solution of the UR5 Robotic Arm](https://link.springer.com/chapter/10.1007/978-3-030-90033-5_22).

> This project is still very experimental, the API will likely still change.

Installation
------------

> Don't forget to activate your venv or conda environment.

Clone this repository, then
```bash
cd ur-analytic-ik
pip install .
```


Usage
-----
Afterwards, you should be able to issue the FK and IK functions like this:

```python
import numpy as np
from ur_analytic_ik import ur5e

eef_pose = np.identity(4)
X = np.array([-1.0, 0.0, 0.0])
Y = np.array([0.0, 1.0, 0.0])
Z = np.array([0.0, 0.0, -1.0])
top_down_orientation = np.column_stack([X, Y, Z])
translation = np.array([-0.2, -0.2, 0.2])

eef_pose[:3, :3] = top_down_orientation
eef_pose[:3, 3] = translation

solutions = ur5e.inverse_kinematics(eef_pose)
```

More examples:
```python
import numpy as np
from ur_analytic_ik import ur3e

joints = np.zeros(6)
eef_pose = np.identity(4)
eef_pose[2, 3] = 0.4
tcp_transform = np.identity(4)
tcp_transform[2, 3] = 0.1

ur3e.forward_kinematics(0, 0, 0, 0, 0, 0)
ur3e.forward_kinematics(*joints)
tcp_pose = ur3e.forward_kinematics_with_tcp(*joints, tcp_transform)

joint_solutions = ur3e.inverse_kinematics(eef_pose)
joint_solutions = ur3e.inverse_kinematics_closest(eef_pose, *joints)
joint_solutions = ur3e.inverse_kinematics_with_tcp(eef_pose, tcp_transform)
```


Testing
-------
In the root directory of this repo, to run the tests:
```
pytest -v
```

Development
--------------------
Some linux users have eigen installed at /usr/include/eigen3 instead of /usr/include/Eigen. Symlink it:
```
sudo ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen
sudo ln -sf /usr/include/eigen3/unsupported /usr/include/unsupported
```

Welcome Improvements
--------------------

### Python API
Adding an IK function that returns the closest solution and accepts a TCP transform.

Reducing the amount of separate IK functions, e.g. replacing:
```python
ur3e.inverse_kinematics_with_tcp(eef_pose)
# with
ur3e.inverse_kinematics(eef_pose, tcp=tcp_transform)
```
The same holds for functions ending with `_closest()`.

### Performance
Currently IK runs at about 10 μs / EEF pose on my laptop.
However, before I implemented the filtering of the solutions, it was closer to 3 μs.
Part of this is because I adapted the bindings in `ur_analytic_ik_ext.cpp` to return vectors with the solutions.

### Code Quality
* Adding more technical documentation.
* `ur_analytic_ik_ext.cpp` should be made much more readable.
* Reducing some duplication e.g. when defining the IK/FK functions and bindings for the different robots.
