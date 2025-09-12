The condynsate Package
======================
Classes
-------
The Animator Class
~~~~~~~~~~~~~~~~~~
.. autoclass:: condynsate.Animator
   :no-members:

Creating the Figure
^^^^^^^^^^^^^^^^^^^
.. automethod:: condynsate.Animator.create_figure
.. automethod:: condynsate.Animator.add_subplot

The Animation Sequence
^^^^^^^^^^^^^^^^^^^^^^
.. automethod:: condynsate.Animator.add_subplot_point
.. automethod:: condynsate.Animator.step
.. automethod:: condynsate.Animator.flush_events
.. automethod:: condynsate.Animator.reset_plots





The Simulator Class
~~~~~~~~~~~~~~~~~~~
.. autoclass:: condynsate.Simulator
   :no-members:

Adjusting the Physics Engine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. automethod:: condynsate.Simulator.set_gravity

Loading URDF Objects in Phyics Engine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. automethod:: condynsate.Simulator.load_urdf

Running a Simulation
^^^^^^^^^^^^^^^^^^^^
.. automethod:: condynsate.Simulator.step
.. automethod:: condynsate.Simulator.reset

Interacting with URDF Objects in the Physics Engine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Adjusting a URDF's Joint's Parameters
*************************************
.. automethod:: condynsate.Simulator.set_joint_force_sensor
.. automethod:: condynsate.Simulator.set_joint_lin_ang_damp
.. automethod:: condynsate.Simulator.set_joint_damping
.. automethod:: condynsate.Simulator.set_joint_friction_params
.. automethod:: condynsate.Simulator.set_joint_contact_params

Setting and Reading a URDF's Joint's State
******************************************
.. automethod:: condynsate.Simulator.set_joint_position
.. automethod:: condynsate.Simulator.set_joint_velocity
.. automethod:: condynsate.Simulator.get_joint_state
.. automethod:: condynsate.Simulator.get_joint_axis

Adjusting and Reading a URDF's Links's Parameters
*************************************************
.. automethod:: condynsate.Simulator.set_link_mass
.. automethod:: condynsate.Simulator.get_link_mass

Setting and Reading a URDF's Links's State
******************************************
.. automethod:: condynsate.Simulator.set_base_state
.. automethod:: condynsate.Simulator.get_link_state
.. automethod:: condynsate.Simulator.get_base_state

Applying Forces and Torques to a URDF
*************************************
.. automethod:: condynsate.Simulator.set_joint_torque
.. automethod:: condynsate.Simulator.apply_force_to_link
.. automethod:: condynsate.Simulator.apply_force_to_com
.. automethod:: condynsate.Simulator.apply_external_torque

Getting the Center of Mass of a URDF
************************************
.. automethod:: condynsate.Simulator.get_center_of_mass

Using the Visualizer
^^^^^^^^^^^^^^^^^^^^
Loading URDF Objects in the Visualizer
**************************************
.. automethod:: condynsate.Simulator.add_urdf_to_visualizer

Adjusting Appearance of URDF Objects
************************************
.. automethod:: condynsate.Simulator.set_link_color
.. automethod:: condynsate.Simulator.set_color_from_pos
.. automethod:: condynsate.Simulator.set_color_from_vel
.. automethod:: condynsate.Simulator.set_color_from_torque
.. automethod:: condynsate.Simulator.set_color_from_mass

Manipulating the Visualizer
***************************
.. automethod:: condynsate.Simulator.transform_camera
.. automethod:: condynsate.Simulator.set_background
.. automethod:: condynsate.Simulator.set_spotlight
.. automethod:: condynsate.Simulator.set_posx_pt_light
.. automethod:: condynsate.Simulator.set_negx_pt_light
.. automethod:: condynsate.Simulator.set_ambient_light
.. automethod:: condynsate.Simulator.set_fill_light

Using the Animator
^^^^^^^^^^^^^^^^^^
.. automethod:: condynsate.Simulator.add_subplot
.. automethod:: condynsate.Simulator.open_animator_gui
.. automethod:: condynsate.Simulator.add_subplot_point
.. automethod:: condynsate.Simulator.reset_plots

Using the Keyboard
^^^^^^^^^^^^^^^^^^
.. automethod:: condynsate.Simulator.is_pressed
.. automethod:: condynsate.Simulator.await_keypress
.. automethod:: condynsate.Simulator.iterate_val





The Keys Class
~~~~~~~~~~~~~~
.. autoclass:: condynsate.Keys
   :members:





The Visualizer Class
~~~~~~~~~~~~~~~~~~~~
.. autoclass:: condynsate.Visualizer
   :no-members:

Adjusting the Appearance of the Viewer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. automethod:: condynsate.Visualizer.set_grid
.. automethod:: condynsate.Visualizer.set_axes
.. automethod:: condynsate.Visualizer.set_background
.. automethod:: condynsate.Visualizer.set_spotlight
.. automethod:: condynsate.Visualizer.set_posx_pt_light
.. automethod:: condynsate.Visualizer.set_negx_pt_light
.. automethod:: condynsate.Visualizer.set_ambient_light
.. automethod:: condynsate.Visualizer.set_fill_light
.. automethod:: condynsate.Visualizer.transform_camera

Adding Elements to the Viewer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. automethod:: condynsate.Visualizer.add_obj
.. automethod:: condynsate.Visualizer.add_stl

Adjusting Elements in the Viewer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. automethod:: condynsate.Visualizer.set_link_color
.. automethod:: condynsate.Visualizer.apply_transform



