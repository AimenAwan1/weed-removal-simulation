from setuptools import find_packages
from setuptools import setup

setup(
    name='name_description',
    version='0.0.0',
    packages=find_packages(
        include=('name_description', 'name_description.*')),
)
