import os

from setuptools import find_namespace_packages, setup, find_packages

with open("README.md", 'r') as f:
    long_description = f.read()

setup(
    name='pathfollowingcontrollers',
    version='1.0',
    description='',
    license="LICENSE.txt",
    long_description=long_description,
    author='Jim Wong',
    author_email='wongjim8@gmail.com',
    url="https://github.com/GoldenPepperoni",
    packages=find_packages(),
    install_requires=['control', 'pyPS4Controller', 'matplotlib',
    'PyFlyt@git+https://github.com/GoldenPepperoni/PyFlyt.git@master#egg=PyFlyt']
)

