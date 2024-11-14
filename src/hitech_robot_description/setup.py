#!/usr/bin/env python3
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Ensure the correct package structure
d = generate_distutils_setup(
    packages=['hitech_robot_description'],
    package_dir={'hitech_robot_description': 'src/hitech_robot_description'}
)

setup(**d)
