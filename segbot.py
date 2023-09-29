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
from simulator import Simulator
from utils import _format_RGB


###############################################################################
"""MAIN LOOP"""
###############################################################################
if __name__ == "__main__":
    # Create an instance of the simulator with visualization
    sim = Simulator(visualization=True)
    
    # Load all urdf objects
    station_radius = 19.59
    station_center = np.array([0., 0., 19.355])
    station_obj = sim.load_urdf(urdf_path='./segbot_urdf/station.urdf',
                                position=station_center,
                                roll=np.pi/2.0,
                                fixed=True)
    segbot_obj = sim.load_urdf(urdf_path='./segbot_urdf/segbot.urdf',
                                position=[0., 0., 0.],
                                yaw=np.pi,
                                fixed=False)
    
    # Set the camera scale and orientation
    sim.transform_camera(scale = [1.5, 1.5, 1.5],
                         pitch=-0.2,
                         yaw = 0.3)
    
    # Set the chassis and wheel weights
    sim.set_link_mass(urdf_obj=segbot_obj,
                      link_name="chassis",
                      mass=20.)
    sim.set_link_mass(urdf_obj=segbot_obj,
                      link_name="right_wheel",
                      mass=20.)
    sim.set_link_mass(urdf_obj=segbot_obj,
                      link_name="left_wheel",
                      mass=20.)
    
    # Set the camera scale and orientation
    sim.transform_camera(scale = [1.5, 1.5, 1.5],
                         pitch=-0.1,
                         yaw=-1.3)
    
    # Variables to track applied torque
    max_torque = 10.
    min_torque = -10.
    prev_r_torque = 10.
    prev_l_torque = 10.
    
    # Variables to track station velocity
    max_vel = 1.
    min_vel = 0.
    prev_vel = 10.
    vel = 0.025
    
    # Wait for user input
    print("PRESS ENTER TO RUN")
    while not keyboard.is_pressed("enter"):
        pass
    
    # Run the simulation
    elapsed_time = 0
    done = False
    while(not done):
        # Keep track of time to run sim in real time
        start_time = time.time()
        
        # Get the current base position of the robot and change
        # gravity accordingly.
        # This simulates centrifugal gravity from the station
        seg_pos = np.array(sim.get_base_pos(segbot_obj))
        diff = seg_pos - station_center
        dirn = diff / np.linalg.norm(diff)
        grav = 9.81 * dirn
        sim.set_gravity(grav)
        
        # Collect keyboard IO for termination
        if keyboard.is_pressed("esc"):
            done = True
        
        # Collect keyboard IO data for torque
        if keyboard.is_pressed("shift+w"):
            r_torque = 0.75 * max_torque
            l_torque = 0.75 * max_torque
        elif keyboard.is_pressed("w"):
            r_torque = 0.33 * max_torque
            l_torque = 0.33 * max_torque
        elif keyboard.is_pressed("shift+s"):
            r_torque = 0.75 * min_torque
            l_torque = 0.75 * min_torque
        elif keyboard.is_pressed("s"):
            r_torque = 0.33 * min_torque
            l_torque = 0.33 * min_torque
        else:
            r_torque = 0.0
            l_torque = 0.0
        if keyboard.is_pressed("d"):
            l_torque = l_torque + 0.25*max_torque
        if keyboard.is_pressed("a"):
            r_torque = r_torque + 0.25*max_torque
           
        # Set the torque and torque colors
        r_torque = round(r_torque,2)
        r_torque_sat = (r_torque - min_torque) / (max_torque - min_torque)
        r_color = cmaps['coolwarm'](round(255*r_torque_sat))[0:3]
        r_color = _format_RGB(r_color, range_to_255=True)
        l_torque = round(l_torque,2)
        l_torque_sat = (l_torque - min_torque) / (max_torque - min_torque)
        l_color = cmaps['coolwarm'](round(255*l_torque_sat))[0:3]
        l_color = _format_RGB(l_color, range_to_255=True)
        sim.set_joint_torque(urdf_obj=segbot_obj,
                             joint_name='chassis_to_right_wheel',
                             torque=r_torque)
        sim.set_link_color(urdf_obj=segbot_obj,
                            link_name='right_wheel',
                            color=r_color)
        sim.set_joint_torque(urdf_obj=segbot_obj,
                             joint_name='chassis_to_left_wheel',
                             torque=l_torque)
        sim.set_link_color(urdf_obj=segbot_obj,
                            link_name='left_wheel',
                            color=l_color)
        
        # Print the current torque
        if r_torque != prev_r_torque or l_torque != prev_l_torque:
            print("Torque: [" + str(l_torque) + ", " + str(r_torque) + "] Nm")
        prev_r_torque = r_torque
        prev_l_torque = l_torque
        
        # Collect keyboard IO data for station vel
        if keyboard.is_pressed("e"):
            vel = vel + 0.001*(max_vel - min_vel)
            if vel > max_vel:
                vel = max_vel
        elif keyboard.is_pressed("q"):
            vel = vel - 0.001*(max_vel - min_vel)
            if vel < min_vel:
                vel = min_vel
           
        # Set the torque and torque colors
        vel = round(vel,4)
        vel_sat = (vel - min_vel) / (max_vel - min_vel)
        vel_color = cmaps['Reds'](round(255*vel_sat))[0:3]
        vel_color = _format_RGB(vel_color, range_to_255=True)
        sim.set_joint_velocity(urdf_obj=station_obj,
                               joint_name="world_to_station",
                               velocity=vel)
        sim.set_link_color(urdf_obj=station_obj,
                           link_name="station",
                           color=vel_color)
        
        # Print the current station vel
        if vel != prev_vel:
            print("Station Vel: " + str(vel) + "RPM")
        prev_vel = vel
        
        # Step the simulation and update the visualization
        sim.engine.stepSimulation()
        sim.update_urdf_visual(station_obj)
        sim.update_urdf_visual(segbot_obj)
        
        # Add sleep to run sim in real time
        elapsed_time = elapsed_time + sim.dt
        time_to_wait = sim.dt + start_time - time.time()
        if time_to_wait > 0.:
            time.sleep(time_to_wait)
            