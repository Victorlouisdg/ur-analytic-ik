import sys, re, os

try:
    from skbuild import setup
    import nanobind
except ImportError:
    print(
        "The preferred way to invoke 'setup.py' is via pip, as in 'pip "
        "install .'. If you wish to run the setup script directly, you must "
        "first install the build dependencies listed in pyproject.toml!",
        file=sys.stderr,
    )
    raise

with open("README.md", "r") as fh:
    long_description = fh.read()


setup(
    packages=["ur_analytic_ik"],
    cmake_install_dir="ur_analytic_ik",
    include_package_data=True,
    long_description=long_description,
    long_description_content_type="text/markdown",
)
