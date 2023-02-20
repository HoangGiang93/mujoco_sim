## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
import os

os.system('make -f USD.mk')

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['pxr', 'mujoco'],
    package_dir={'pxr': 'build/USD-23.02/lib/python/pxr', 'mujoco': 'build/mujoco-2.3.2/mujoco'},
)

setup(**setup_args)