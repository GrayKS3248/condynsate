==========
condynsate
==========

**condynsate** (\ **con**\ trol and **dyn**\ amics simul\ **at**\ or) is a python-based, open-source educational tool built by `G. Schaer`_ at the University of Illinois at Urbana-Champaign under the Grainger College of Engineering 2023-2025 Strategic Instructional Innovations Program: `Computational Tools for Dynamics and Control grant`_. It is designed to aid the education of control and dynamics to aerospace, mechanical, and robotics engineering students by

1. providing a simulation environment in which students can see and interact with controlled and uncontrolled dynamic systems in familiar and `beneficial ways`_,
2. serving as a platform for introductory Python programming, and
3. equipping instructors with a streamlined method of implementing custom in-class demonstrations and lab demos without the need for physical equipment.

Built on `PyBullet`_, `MeshCat`_, and `OpenCV`_, it implements nonlinear simulation of `stl`_ or `obj`_ rigid bodies and\or `urdf`_ articulated bodies. A browser-based 3D viewer visualizes the simulation and the evolution of individual states are plotted, all in real-time. By simultaneously enabling keyboard interactivity, condynsate projects are designed to feel more like video games, familiar and fun, rather than conventional lab demos all while providing similar educational benefits. 

.. _PyBullet: https://pybullet.org/wordpress/
.. _Meshcat: https://github.com/meshcat-dev/meshcat-python/
.. _OpenCV: https://opencv.org/
.. _urdf: http://wiki.ros.org/urd/
.. _stl: https://en.wikipedia.org/wiki/STL_(file_format)/
.. _obj: https://en.wikipedia.org/wiki/Wavefront_.obj_file/
.. _G. Schaer: http://bretl.csl.illinois.edu/people
.. _Computational Tools for Dynamics and Control grant: https://ae3.grainger.illinois.edu/programs/siip-grants/64459
.. _beneficial ways: https://doi.org/10.3390/educsci13070747


Contents
--------

.. toctree::

   installation

   package

   tutorials