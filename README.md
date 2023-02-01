UR Analytic IK
================
C++ implementation with Python bindings of analytic forward and inverse kinematics for the Universal Robots based on [Alternative Inverse Kinematic Solution of the UR5 Robotic Arm](https://link.springer.com/chapter/10.1007/978-3-030-90033-5_22).

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
from ur_analytic_ik import ur5e_inverse_kinematics

# Known EEF pose for the UR5e that should have (0, 0, 0, 0, 0, 0) as solution for the joints:
eef_rotation0 = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
eef_translation0 = np.array([-0.8172, -0.2329, 0.0628])

solutions = ur5e_inverse_kinematics(eef_rotation0, eef_translation0)
```
