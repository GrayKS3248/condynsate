# -*- coding: utf-8 -*-
"""
Copyright (c) 2023; Grayson Schaer

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files 
(the “Software”), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge,
publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, 
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH 
THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""


###############################################################################
"""DEPENDENCIES"""
###############################################################################
import numpy as np
import time
import keyboard
from matplotlib import colormaps as cmaps
from condynsate import _format_RGB
from condynsate import Simulator


###############################################################################
"""MAIN LOOP"""
###############################################################################
if __name__ == "__main__":
    # Create an instance of the simulator with visualization
    sim = Simulator(visualization=True)
    
    # Load all urdf objects
    ground_obj = sim.load_urdf(urdf_path='./cmg_urdf/plane.urdf',
                               tex_path='./cmg_urdf/check.png',
                               position=[0., 0., -3.],
                               wxyz_quaternion=[1., 0., 0., 0],
                               fixed=True)
    wall_obj = sim.load_urdf(urdf_path='./cmg_urdf/plane.urdf',
                             tex_path='./cmg_urdf/concrete.png',
                             position=[0., 0., 0.],
                             roll=0.5*np.pi,
                             pitch=0.,
                             yaw=np.pi,
                             fixed=True)
    cmg_obj = sim.load_urdf(urdf_path='./cmg_urdf/cmg.urdf',
                            position=[0., 1.1, 0.],
                            roll=0.0,
                            pitch=-1.57,
                            yaw=0.,
                            fixed=True)
    
    # Set link mass
    sim.set_link_mass(urdf_obj=cmg_obj,
                      link_name="mass",
                      mass=1.0)
    
    # Set wheel velocity
    sim.set_joint_velocity(urdf_obj=cmg_obj,
                           joint_name='inner_to_wheel',
                           velocity = 50.)
    
    # Set joint damping
    sim.set_joint_damping(urdf_obj=cmg_obj,
                          joint_name="world_to_outer",
                          damping=0.0)
    sim.set_joint_damping(urdf_obj=cmg_obj,
                          joint_name="outer_to_inner",
                          damping=0.0)
    
    # Set the camera scale and orientation
    sim.transform_camera(scale = [2.25, 2.25, 2.25],
                         pitch=-0.2,
                         yaw=0.9)

    # Variables to track applied torque
    prev_torque = -1.0
    torque = 0.0
    max_torque = 0.5
    min_torque = -0.5
    torque = 0.5*(max_torque + min_torque)
    torque_sat = (torque - min_torque) / (max_torque - min_torque)
    inner_color = cmaps['coolwarm'](round(255*torque_sat))[0:3]
    
    # Variables to track mass
    prev_mass = 0.0
    max_mass = 2.0
    min_mass = 0.0
    mass = 0.5*(max_mass + min_mass)
    mass_sat = (mass - min_mass) / (max_mass - min_mass)
    mass_color = cmaps['binary'](round(255*mass_sat))[0:3]
    
    # Variables to track wheel RPM
    prev_vel = 0.0
    max_vel = 100.0
    min_vel = 0.0
    vel = 0.5 * (max_vel + min_vel)
    vel_sat = (vel - min_vel) / (max_vel - min_vel)
    vel_color = cmaps['Reds'](round(255*vel_sat))[0:3]
    
    # Run the simulation
    elapsed_time = 0
    done = False
    while(not done):
        # Keep track of time to run sim in real time
        start_time = time.time()
        sim.engine.stepSimulation()
        
        # Collect keyboard IO for termination
        if keyboard.is_pressed("esc"):
            done = True
        
        # Collect keyboard IO data for torque
        if keyboard.is_pressed("shift+d"):
            torque = max_torque
        elif keyboard.is_pressed("d"):
            torque = max_torque / 4.0
        elif keyboard.is_pressed("shift+a"):
            torque = min_torque
        elif keyboard.is_pressed("a"):
            torque = min_torque / 4.0
        else:
            torque = 0.0
            
        # Set the torque and link color based on keyboard inputs
        torque = round(torque,2)
        torque_sat = (torque - min_torque) / (max_torque - min_torque)
        torque_color = cmaps['coolwarm'](round(255*torque_sat))[0:3]
        torque_color = _format_RGB(torque_color,
                                   range_to_255=True)
        sim.set_joint_torque(urdf_obj=cmg_obj,
                             joint_name="outer_to_inner",
                             torque=torque)
        sim.set_link_color(urdf_obj=cmg_obj,
                           link_name='inner',
                           color=torque_color)
        
        # Print the current torque
        if torque != prev_torque:
            print("Torque: " + str(torque) + "Nm")
        prev_torque = torque
        
        # Collect keyboard IO data for mass
        if keyboard.is_pressed('e'):
            mass = mass + 0.005*(max_mass - min_mass)
            if mass > max_mass:
                mass = max_mass
        elif keyboard.is_pressed('q'):
            mass = mass - 0.005*(max_mass - min_mass)
            if mass < min_mass:
                mass = min_mass
            
        # Set the mass and link color based on keyboard inputs
        mass = round(mass,2)
        mass_sat = (mass - min_mass) / (max_mass - min_mass)
        mass_color = cmaps['binary'](round(255*mass_sat))[0:3]
        mass_color = _format_RGB(mass_color,
                                 range_to_255=True)
        sim.set_link_mass(urdf_obj=cmg_obj,
                          link_name='mass',
                          mass = mass)
        sim.set_link_color(urdf_obj=cmg_obj,
                           link_name='mass',
                           color=mass_color)
    
        # Print the current torque
        if mass != prev_mass:
            print("Mass: " + str(mass) + "Kg")
        prev_mass = mass
    
        # Collect keyboard IO data for wheel vel
        if keyboard.is_pressed('w'):
            vel = vel + 0.005*(max_vel - min_vel)
            if vel > max_vel:
                vel = max_vel
        elif keyboard.is_pressed('s'):
            vel = vel - 0.005*(max_vel - min_vel)
            if vel < min_vel:
                vel = min_vel
            
        # Set the wheel vel and link color based on keyboard inputs
        vel = round(vel,2)
        vel_sat = (vel - min_vel) / (max_vel - min_vel)
        vel_color = cmaps['Reds'](round(255*vel_sat))[0:3]
        vel_color = _format_RGB(vel_color,
                                range_to_255=True)
        
        sim.set_joint_velocity(urdf_obj=cmg_obj,
                              joint_name="inner_to_wheel",
                              velocity=vel)
        sim.set_link_color(urdf_obj=cmg_obj,
                           link_name='wheel',
                           color=vel_color)
    
        # Print the current wheel velocity
        if vel != prev_vel:
            print("Wheel Speed: " + str(vel) + "RPM")
        prev_vel = vel
    
        # Update the visualizer
        sim.update_urdf_visual(cmg_obj)
        sim.update_urdf_visual(ground_obj)
        sim.update_urdf_visual(wall_obj)
        
        # Add sleep to run sim in real time
        elapsed_time = elapsed_time + sim.dt
        time_to_wait = sim.dt + start_time - time.time()
        if time_to_wait > 0.:
            time.sleep(time_to_wait)
            