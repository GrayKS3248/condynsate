Shield: [![CC BY-SA 4.0][cc-by-sa-shield]][cc-by-sa]

This work is licensed under a
[Creative Commons Attribution-ShareAlike 4.0 International License][cc-by-sa].

[![CC BY-SA 4.0][cc-by-sa-image]][cc-by-sa]

[cc-by-sa]: http://creativecommons.org/licenses/by-sa/4.0/
[cc-by-sa-image]: https://licensebuttons.net/l/by-sa/4.0/88x31.png
[cc-by-sa-shield]: https://img.shields.io/badge/License-CC%20BY--SA%204.0-lightgrey.svg

# IMPORTANT NOTE

This repository is now archived and read-only as it is the legacy version of condynsate. No bug fixes will be made. For the actively developed version, we reccomend viewing [the current repository](https://github.com/condynsate/condynsate) instead.

# condynsate Legacy

**condynsate** (**con**trol and **dyn**amics simul**at**or) is a python-based, open-source educational tool built by [G. Schaer](http://bretl.csl.illinois.edu/people) at the University of Illinois at Urbana-Champaign under the Grainger College of Engineering 2023-2025 Strategic Instructional Innovations Program: [Computational Tools for Dynamics and Control grant](https://ae3.grainger.illinois.edu/programs/siip-grants/64459). It is designed to aid the education of control and dynamics to aerospace, mechanical, and robotics engineering students by

1. providing a simulation environment in which students can see and interact with controlled and uncontrolled dynamic systems in familiar and [beneficial ways](https://doi.org/10.3390/educsci13070747),
2. serving as a platform for introductory Python programming, and
3. equipping instructors with a streamlined method of implementing custom in-class demonstrations and lab demos without the need for physical equipment.

Built on [PyBullet](https://pybullet.org/wordpress/), [MeshCat](https://github.com/meshcat-dev/meshcat-python/), and [Tk](https://www.tcl-lang.org/), it implements nonlinear simulation of [stl](https://en.wikipedia.org/wiki/STL_(file_format)/) or [obj](https://en.wikipedia.org/wiki/Wavefront_.obj_file/) rigid bodies and\or [urdf](http://wiki.ros.org/urd/) articulated bodies. A browser-based 3D viewer visualizes the simulation and the evolution of individual states are plotted, all in real-time. By simultaneously enabling keyboard interactivity, condynsate projects are designed to feel more like video games, familiar and fun, rather than conventional lab demos all while providing similar educational benefits.

# Installation
## Windows
### Source
A C++ compiler for C++ 2003 is needed. On Windows, we recommend using the Desktop development with C++ workload for [Microsoft C++ Build Tools 2022](https://visualstudio.microsoft.com/visual-cpp-build-tools/). Additionally, [python>=3.8](https://www.python.org/), [pip](https://pip.pypa.io/en/stable/), and [git](https://git-scm.com/) are also required. We strongly recommend installing condynsate in a virtual environment:

```powershell
C:\Users\username> python -m venv .venv
C:\Users\username> .venv\Scripts\activate.bat
```

To clone the repository:

```powershell
(.venv) C:\Users\username> git clone https://github.com/condynsate/condynsate.git
```

To install:

```powershell
(.venv) C:\Users\username> cd condynsate
(.venv) C:\Users\username> pip install -e .
```

When done, to deactivate the virtual environment:

```powershell
(.venv) C:\Users\username> deactivate
```





