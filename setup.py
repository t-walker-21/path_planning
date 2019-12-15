#!/usr/bin/env python

from distutils.core import setup

setup(name='path_planning',
      version='1.0',
      description='Simple Robotic Path Planning Library',
      author='Tevon Walker',
      author_email='twalker81@gatech.edu',
      packages=['path_planning'],
      package_dir={'':'src'},
      install_requires=['numpy', 'opencv-python']
     )
