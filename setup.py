import sys, re, os

try:
    from skbuild import setup
    import nanobind
except ImportError:
    print("The preferred way to invoke 'setup.py' is via pip, as in 'pip "
          "install .'. If you wish to run the setup script directly, you must "
          "first install the build dependencies listed in pyproject.toml!",
          file=sys.stderr)
    raise

setup(
    name="ur_analytic_ik",
    version="0.0.1",
    author="Victor-Louis De Gusseme",
    author_email="victorlouisdg@gmail.com",
    description="C++ implementation with Python bindings of analytic forward and inverse kinematics for the Universal Robots.",
    url="https://github.com/Victorlouisdg/ur-analytic-ik",
    license="MIT",
    packages=['ur_analytic_ik'],
    cmake_install_dir="ur_analytic_ik",
    include_package_data=True,
    python_requires=">=3.8",
    install_requires=["numpy"],
)
