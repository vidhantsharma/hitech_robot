#!/usr/bin/env python3
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Ensure the correct package structure
d = generate_distutils_setup(
    packages=['hitech_robot_functionality'],
    package_dir={'hitech_robot_functionality': 'src/hitech_robot_functionality'}
)

setup(**d)
