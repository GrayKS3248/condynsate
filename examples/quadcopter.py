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
                               gravity=[0., 0., 0.])
    
    # Load all objects
    ground_obj = sim.load_urdf(urdf_path='./quadcopter_vis/plane.urdf',
                                tex_path='./quadcopter_vis/wave.png',
                                position=[0., 0., -3.],
                                wxyz_quaternion=[1., 0., 0., 0],
                                fixed=True)
    quad_obj = sim.load_urdf(urdf_path='./quadcopter_vis/quadcopter.urdf',
                             tex_path='./quadcopter_vis/nametag.png',
                             fixed=False)
    
    # Apply damping to rotors
    sim.set_joint_damping(urdf_obj=quad_obj,
                          joint_name='spar1_to_rotor1',
                          damping=0.01)
    sim.set_joint_damping(urdf_obj=quad_obj,
                          joint_name='spar2_to_rotor2',
                          damping=0.01)
    sim.set_joint_damping(urdf_obj=quad_obj,
                          joint_name='spar3_to_rotor3',
                          damping=0.01)
    sim.set_joint_damping(urdf_obj=quad_obj,
                          joint_name='spar4_to_rotor4',
                          damping=0.01)
    
    sim.get_base_pos_vel(urdf_obj=quad_obj)
    
    # Variables to track applied RPM
    max_torque = 1.
    min_torque = 0.
    prev_torque_1 = 10.
    prev_torque_2 = 10.
    prev_torque_3 = 10.
    prev_torque_4 = 10.
    
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
        
        # Collect keyboard IO data for torque 1
        if keyboard.is_pressed("a"):
            torque_1 = max_torque
        else:
            torque_1 = min_torque
        torque_1 = round(torque_1,4)
        torque_1_sat = (torque_1 - min_torque) / (max_torque - min_torque)
        torque_1_color = cmaps['coolwarm'](round(255*torque_1_sat))[0:3]
        torque_1_color = format_RGB(torque_1_color,
                                    range_to_255=True)
        sim.set_joint_torque(urdf_obj=quad_obj,
                            joint_name='spar1_to_rotor1',
                            torque=torque_1)
        # sim.set_link_color(urdf_obj=craft_obj,
        #                    link_name='wheel_1',
        #                    color=torque_1_color)
        
        # Collect keyboard IO data for torque 2
        if keyboard.is_pressed("s"):
            torque_2 = max_torque
        else:
            torque_2 = min_torque
        torque_2 = round(torque_2,4)
        torque_2_sat = (torque_2 - min_torque) / (max_torque - min_torque)
        torque_2_color = cmaps['coolwarm'](round(255*torque_2_sat))[0:3]
        torque_2_color = format_RGB(torque_2_color,
                                    range_to_255=True)
        sim.set_joint_torque(urdf_obj=quad_obj,
                            joint_name='spar2_to_rotor2',
                            torque=torque_2)
        # sim.set_link_color(urdf_obj=craft_obj,
        #                    link_name='wheel_2',
        #                    color=torque_2_color)
        
        # Collect keyboard IO data for torque 3
        if keyboard.is_pressed("d"):
            torque_3 = max_torque
        else:
            torque_3 = min_torque
        torque_3 = round(torque_3,4)
        torque_3_sat = (torque_3 - min_torque) / (max_torque - min_torque)
        torque_3_color = cmaps['coolwarm'](round(255*torque_3_sat))[0:3]
        torque_3_color = format_RGB(torque_3_color,
                                    range_to_255=True)
        sim.set_joint_torque(urdf_obj=quad_obj,
                            joint_name='spar3_to_rotor3',
                            torque=torque_3)
        # sim.set_link_color(urdf_obj=craft_obj,
        #                    link_name='wheel_3',
        #                    color=torque_3_color)
        
        # Collect keyboard IO data for torque 4
        if keyboard.is_pressed("f"):
            torque_4 = max_torque
        else:
            torque_4 = min_torque
        torque_4 = round(torque_4,4)
        torque_4_sat = (torque_4 - min_torque) / (max_torque - min_torque)
        torque_4_color = cmaps['coolwarm'](round(255*torque_4_sat))[0:3]
        torque_4_color = format_RGB(torque_4_color,
                                    range_to_255=True)
        sim.set_joint_torque(urdf_obj=quad_obj,
                            joint_name='spar4_to_rotor4',
                            torque=torque_4)
        # sim.set_link_color(urdf_obj=craft_obj,
        #                    link_name='wheel_4',
        #                    color=torque_4_color)
        
        # Print torque changes
        change_1 = torque_1 != prev_torque_1
        change_2 = torque_2 != prev_torque_2
        change_3 = torque_3 != prev_torque_3
        change_4 = torque_4 != prev_torque_4
        if change_1 or change_2 or change_3 or change_4:
            str1 = str(torque_1)
            str2 = str(torque_2)
            str3 = str(torque_3)
            str4 = str(torque_4)
            print("Torque: ["+str1+", "+str2+", "+str3+", "+str4+"] Nm")
        prev_torque_1 = torque_1
        prev_torque_2 = torque_2
        prev_torque_3 = torque_3
        prev_torque_4 = torque_4
        
        # Collect keyboard IO for termination
        if keyboard.is_pressed("esc"):
            done = True
            
        # Step the simulation and update the visualization
        sim.engine.stepSimulation()
        sim.update_urdf_visual(quad_obj)
        sim.update_urdf_visual(ground_obj)
        
        # Add sleep to run sim in real time
        elapsed_time = elapsed_time + sim.dt
        time_to_wait = sim.dt + start_time - time.time()
        if time_to_wait > 0.:
            time.sleep(time_to_wait)
            