import os

from setuptools import find_namespace_packages, setup


def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join("..", path, filename))
    return paths

with open("README", 'r') as f:
    long_description = f.read()

setup(
    name='pathfollowing-controllers',
    version='1.0',
    description='',
    license="LICENSE.txt",
    long_description=long_description,
    author='Jim Wong',
    author_email='wongjim8@gmail.com',
    url="https://github.com/GoldenPepperoni",
    packages=[
        package for package in find_namespace_packages() if package.startswith("pathfollowing-controllers")
    ],
    install_requires=['PyFlyt', 'control', 'pyPS4Controller'],
    dependency_links = ['git+https://github.com/GoldenPepperoni/PyFlyt.git@master#egg=PyFlyt']
)