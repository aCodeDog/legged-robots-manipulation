from setuptools import find_packages
from distutils.core import setup

setup(
    name='loco_manipulation_gym',
    version='1.0.0',
    author='Zifan Wang',
    license="BSD-3-Clause",
    packages=find_packages(),
    author_email='wang_zifan@outlook.com',
    description='loco_manipulation lib  for Legged Robots',
    install_requires=['isaacgym',
                      'rsl-rl',
                      'matplotlib']
)