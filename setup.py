# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['psc'],
    scripts=['scripts/integration_test.py',
             'scripts/get_crazyflie_names.py'],
    package_dir={'': 'src'},
)

setup(**setup_args)
