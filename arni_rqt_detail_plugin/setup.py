from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
	packages=['arni_rqt_detail_plugin'],
	package_dir={'': 'src'},
	scripts=['scripts/arni_rqt_detail_plugin']
)

setup(**setup_args)
