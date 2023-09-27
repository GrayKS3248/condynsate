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
    sim = Simulator(visualization=True,
                    gravity=[0., 0., -9.81])
    
    # Load all urdf objects
    station_obj = sim.load_urdf(urdf_path='./segbot_urdf/station.urdf',
                                position=[0., 0., 19.34],
                                roll=np.pi/2.0)
    
    segbot_obj = sim.load_urdf(urdf_path='./segbot_urdf/segbot.urdf',
                                position=[0., 0., 1.])
    
    # Set the station speed
    sim.set_joint_velocity(urdf_obj=station_obj,
                           joint_name="world_to_station",
                           velocity=0.25)
    
    # Set the camera scale and orientation
    sim.transform_camera(scale = [1.5, 1.5, 1.5],
                         pitch=-0.2,
                         yaw = 0.3)
    
    # Run the simulation
    elapsed_time = 0
    done = False
    while(not done):
        # Keep track of time to run sim in real time
        start_time = time.time()
        sim.engine.stepSimulation()
        
        # Update the visualizer
        sim.update_urdf_visual(station_obj)
        sim.update_urdf_visual(segbot_obj)
        
        # Collect keyboard IO for termination
        if keyboard.is_pressed("esc"):
            done = True
        
        sim.set_joint_torque(urdf_obj=segbot_obj,
                             joint_name='chassis_to_left_wheel',
                             torque=-0.02)
        sim.set_joint_torque(urdf_obj=segbot_obj,
                             joint_name='chassis_to_right_wheel',
                             torque=-0.02)
        
        
        # Add sleep to run sim in real time
        elapsed_time = elapsed_time + sim.dt
        time_to_wait = sim.dt + start_time - time.time()
        if time_to_wait > 0.:
            time.sleep(time_to_wait)
            
    