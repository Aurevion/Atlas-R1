from setuptools import find_packages
from setuptools import setup

setup(
    name='atlasr1_controller',
    version='0.0.0',
    packages=find_packages(
        include=('atlasr1_controller', 'atlasr1_controller.*')),
)
