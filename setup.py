## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


# fetches values from package.xml
setup_args = generate_distutils_setup(
	packages=['windshape'],
	package_dir={'': 'src'},
	install_requires=['numpy', 'python-mysqldb']
)
setup(**setup_args)
