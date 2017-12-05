## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# requirements = [line.strip() for line in open("requirements.txt")]

d = generate_distutils_setup()

d['name'] = "graspit_moveit_controller"
d['description'] = "Code for interfacing with Moveit! and Graspit! grasps using pick and place"
d['packages'] = ['graspit_moveit_controller']
d['package_dir'] = {'': 'src'}

setup(**d)
