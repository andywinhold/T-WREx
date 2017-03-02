#!/usr/bin/env python
# -*- coding: utf-8 -*-

try:
	from setuptools import setup
except ImportError:
	from distutils.core import setup

setup(
	name='T-WREx',
	version='0.1dev',
	description="Thermal Water Extraction for Asteroid Study",
	author='Andrew Winhold',
	author_email='andrew.winhold@asu.edu',
	license='GNU GENERAL PUBLIC LICENSE',
	long_description=open('README.md').read(),
	entry_points={
		"console_scripts": [
			"test_command = twrex.test_command:main",
		]
	}	
)