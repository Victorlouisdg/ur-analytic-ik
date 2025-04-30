UR Analytic IK
================
C++ implementation with Python bindings of analytic forward and inverse kinematics for the Universal Robots based on [Alternative Inverse Kinematic Solution of the UR5 Robotic Arm](https://link.springer.com/chapter/10.1007/978-3-030-90033-5_22).

> This project is still very experimental, the API will likely still change.


The main advantages of using analytic IK:
- extremely fast. FK calls from python take +- 4 µs, IK calls +- 18 µs. 
- finds all solutions at once, allowing you to select the most convenient one.
- no need to provide initial guess, as opposed to numerical IK solutions.


> Warning: this repo uses the default DH-parameters for the UR robots. But every robot is slightly different and is factory-calibrated to provide very accurate DH parameters, which are also used by the robot controlbox. From a few tests we have run, using the default DH-parameters typically results in 1-2mm differences in the FK for a given joint configuration. See [here](notebooks/compare_to_real_robot.ipynb) for details. If you need very high precision, you might want to use your robot's DH-parameters for FK/IK. 



Installation
------------

pre-built wheels are availabe on PyPI and can be installed with pip:

```bash
pip install ur_analytic_ik
```

To install from source, see the Developer section.

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
joint_solutions = ur3e.inverse_kinematics_closest_with_tcp(eef_pose, tcp_transform, *joints)
```




Development
--------------------

This codebase uses [nanobind]() to provide python bindings for the FK/IK functions.

## building
**python package building** 

This is the easiest option. It leverages scikit-build to create a python package and build the bindings. This flow is based on https://github.com/wjakob/nanobind_example

- Create a conda environment for the project:  `conda env create -f environment.yaml` 
- to create the python package, including the bindings: `pip install .` (this uses scikit-build to build the C++ from the top-level CMakelist.txt)
- you can now import the library in python.


**C++ building**

if you want to build the C++ code without building the bindings or creating a python package:

- make sure you have a C++ compiler available.
- make sure you have the [Eigen]() package available, if not run `apt install libeigen3-dev`.

Some linux users have eigen installed at /usr/include/eigen3 instead of /usr/include/Eigen. Symlink it:
```
sudo ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen
sudo ln -sf /usr/include/eigen3/unsupported /usr/include/unsupported
```
- run `cmake -S . -B` & `cmake --build build` from the `src/` dir. 
- execute `./build/main`


## testing

run `pytest -v .`

Tests are also automatically executed in github for each commit.

Wheels are built automatically for all PRs, you can check them on [test PyPI]().


## Releasing

- bump the version in the `pyproject.toml` file. We use [semantic versioning](). Use pre-releases if you want to test changes.
- create a new tag, corresponding to the version: `git tag vX.Y.Z-...` 
- push the tag `git push --tag`, this will already trigger a build of the wheels on [test PyPI](https://test.pypi.org/project/ur-analytic-ik/)
- once you have verified the wheels work and are built properly, create a new release with the same name as the semantic version for the tag on github. This will trigger an upload to [PyPI](https://pypi.org/project/ur-analytic-ik/).



Welcome Improvements
--------------------

## Python API
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
