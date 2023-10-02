###############################################################################
#DEPENDENCIES
###############################################################################
import numpy as np
import time
import keyboard
from matplotlib import colormaps as cmaps
import condynsate
from condynsate.utils import format_RGB


###############################################################################
#MAIN LOOP
###############################################################################
if __name__ == "__main__":
    # Create an instance of the simulator with visualization
    sim = condynsate.Simulator(visualization=True,
                               gravity=[0., 0., -9.81])
    
    # Load all objects
    ground_obj = sim.load_urdf(urdf_path='./quadcopter_vis/plane.urdf',
                                tex_path='./quadcopter_vis/wave.png',
                                position=[0., 0., -3.],
                                wxyz_quaternion=[1., 0., 0., 0],
                                fixed=True)
    quad_obj = sim.load_urdf(urdf_path='./quadcopter_vis/drone.urdf',
                             tex_path='./quadcopter_vis/nametag.png',
                             fixed=False)
    
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
        
        # Collect keyboard IO for termination
        if keyboard.is_pressed("esc"):
            done = True
            
        # Step the simulation and update the visualization
        sim.engine.stepSimulation()
        sim.update_urdf_visual(quad_obj)
        
        # Add sleep to run sim in real time
        elapsed_time = elapsed_time + sim.dt
        time_to_wait = sim.dt + start_time - time.time()
        if time_to_wait > 0.:
            time.sleep(time_to_wait)
            