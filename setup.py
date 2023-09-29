from setuptools import setup

setup(name="condynsate",
      version="0.0.1",
      description="meshcat visualization and pybullet simulation",
      url="https://github.com/GrayKS3248/SIIP",
      author="Grayson Schaer",
      author_email="gschaer2@illinois.edu",
      license="MIT",
      packages=['condynsate'],
      install_requires=["numpy >= 1.24.3",
                        "keyboard >= 0.13.5",
                        "matplotlib >= 3.7.1",
                        "pybullet >= 3.2.5",
                        "meshcat >= 0.3.2"],
      zip_safe=False,
      include_package_data=True)