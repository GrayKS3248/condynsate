# -*- coding: utf-8 -*-
###############################################################################
#DEPENDENCIES
###############################################################################
from setuptools import setup, find_packages


###############################################################################
#SETUP DESCRIPTION
###############################################################################
setup(name="condynsate",
    version="0.0.1",
    description="meshcat-based visualizer coupled with pybullet-based simulator",
    url="https://github.com/GrayKS3248/SIIP",
    author="Grayson Schaer",
    author_email="gschaer2@illinois.edu",
    license="MIT",
    packages=find_packages('src'),
    package_dir={"": "src"},
    install_requires=[
      "numpy >= 1.24.3",
      "keyboard >= 0.13.5",
      "matplotlib >= 3.7.1",
      "pybullet >= 3.2.5",
      "meshcat >= 0.3.2"
    ],
    classifiers=[
        "Development Status :: 2 - Pre-Alpha",
        "License :: MIT License"
    ],
    zip_safe=False,
    include_package_data=True
)