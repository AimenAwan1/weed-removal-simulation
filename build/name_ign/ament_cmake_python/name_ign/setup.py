from setuptools import find_packages
from setuptools import setup

setup(
    name='name_ign',
    version='0.0.0',
    packages=find_packages(
        include=('name_ign', 'name_ign.*')),
)
