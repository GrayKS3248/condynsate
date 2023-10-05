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
    
    # Load the spacecraft
    craft_obj = sim.load_urdf(urdf_path='./spacecraft_vis/spacecraft.urdf',
                              fixed=False,
                              update_vis=True)
    
    # Generate star cartesian coordinates based on their depth and
    # equatorial astronomical coords (right ascension, declination)
    depth = 55.
    equatorials = np.array([[-0.06, -0.09],
                            [ 0.00, -0.09],
                            [ 0.06, -0.09],
                            [ 0.00, -0.03],
                            [ 0.00,  0.03],
                            [-0.06,  0.09],
                            [ 0.00,  0.09],
                            [ 0.06,  0.09]])
    cartesians = []
    for equatorial in equatorials:
        s_a = np.sin(equatorial[0])
        c_a = np.cos(equatorial[0])
        s_d = np.sin(equatorial[1])
        c_d = np.cos(equatorial[1])
        cartesian = np.array([c_a*c_d, s_a*c_d, s_d]) * depth
        cartesians.append(cartesian)
    
    # Load the constellation
    constellation_objs = []
    for base_pos in cartesians:
        urdf_id = sim.load_urdf(urdf_path='./spacecraft_vis/sphere.urdf',
                                position=base_pos,
                                fixed=True,
                                update_vis=False)
        constellation_objs.append(urdf_id)
    
    # Load random stars
    np.random.seed(10)
    star_objs = []
    n_stars = 100
    rand_depths = depth + (np.random.rand(n_stars)*0.2-0.1)*depth
    positions = np.random.randn(n_stars,3)
    positions = positions / np.linalg.norm(positions,axis=1).reshape(n_stars,1)
    positions = positions * rand_depths.reshape(n_stars,1)
    for i in range(n_stars):
        urdf_id = sim.load_urdf(urdf_path='./spacecraft_vis/sphere.urdf',
                                position=positions[i],
                                fixed=True,
                                update_vis=False)
        star_objs.append(urdf_id)
    
    # Set the camera scale and orientation
    sim.transform_camera(scale = [4., 4., 4.],
                         yaw=-2.5,
                         pitch=-0.1)
    
    # Set background and lighting properties
    sim.set_background(top_color=[10,10,10],
                       bot_color=[20, 20, 40])
    sim.set_spotlight(on = False)
    sim.set_fill_light(on=False)
    sim.set_posx_pt_light(on = True,
                          intensity = 0.5,
                          distance = 10.)
    sim.set_negx_pt_light(on = True,
                          intensity = 0.5,
                          distance = 10.)
    sim.set_ambient_light(on=True,
                          intensity=0.65)

    # Variables to track applied torque
    max_torque = 1.
    min_torque = -1.
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
        
        # Collect keyboard IO for termination
        if keyboard.is_pressed("esc"):
            done = True
        
        # Collect keyboard IO data for torque 1
        if keyboard.is_pressed("a"):
            torque_1 = min_torque
        elif keyboard.is_pressed("q"):
            torque_1 = max_torque
        else:
            torque_1 = 0.0
        torque_1 = round(torque_1,4)
        torque_1_sat = (torque_1 - min_torque) / (max_torque - min_torque)
        torque_1_color = cmaps['coolwarm'](round(255*torque_1_sat))[0:3]
        torque_1_color = format_RGB(torque_1_color,
                                    range_to_255=True)
        sim.set_joint_torque(urdf_obj=craft_obj,
                            joint_name='bus_to_wheel_1',
                            torque=torque_1)
        sim.set_link_color(urdf_obj=craft_obj,
                           link_name='wheel_1',
                           color=torque_1_color)
        
        # Collect keyboard IO data for torque 2
        if keyboard.is_pressed("s"):
            torque_2 = min_torque
        elif keyboard.is_pressed("w"):
            torque_2 = max_torque
        else:
            torque_2 = 0.0
        torque_2 = round(torque_2,4)
        torque_2_sat = (torque_2 - min_torque) / (max_torque - min_torque)
        torque_2_color = cmaps['coolwarm'](round(255*torque_2_sat))[0:3]
        torque_2_color = format_RGB(torque_2_color,
                                    range_to_255=True)
        sim.set_joint_torque(urdf_obj=craft_obj,
                            joint_name='bus_to_wheel_2',
                            torque=torque_2)
        sim.set_link_color(urdf_obj=craft_obj,
                           link_name='wheel_2',
                           color=torque_2_color)
        
        # Collect keyboard IO data for torque 3
        if keyboard.is_pressed("d"):
            torque_3 = min_torque
        elif keyboard.is_pressed("e"):
            torque_3 = max_torque
        else:
            torque_3 = 0.0
        torque_3 = round(torque_3,4)
        torque_3_sat = (torque_3 - min_torque) / (max_torque - min_torque)
        torque_3_color = cmaps['coolwarm'](round(255*torque_3_sat))[0:3]
        torque_3_color = format_RGB(torque_3_color,
                                    range_to_255=True)
        sim.set_joint_torque(urdf_obj=craft_obj,
                            joint_name='bus_to_wheel_3',
                            torque=torque_3)
        sim.set_link_color(urdf_obj=craft_obj,
                           link_name='wheel_3',
                           color=torque_3_color)
        
        # Collect keyboard IO data for torque 4
        if keyboard.is_pressed("f"):
            torque_4 = min_torque
        elif keyboard.is_pressed("r"):
            torque_4 = max_torque
        else:
            torque_4 = 0.0
        torque_4 = round(torque_4,4)
        torque_4_sat = (torque_4 - min_torque) / (max_torque - min_torque)
        torque_4_color = cmaps['coolwarm'](round(255*torque_4_sat))[0:3]
        torque_4_color = format_RGB(torque_4_color,
                                    range_to_255=True)
        sim.set_joint_torque(urdf_obj=craft_obj,
                            joint_name='bus_to_wheel_4',
                            torque=torque_4)
        sim.set_link_color(urdf_obj=craft_obj,
                           link_name='wheel_4',
                           color=torque_4_color)
        
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
        
        # Step the simulation and update the visualization
        sim.step()
        
        # Add sleep to run sim in real time
        elapsed_time = elapsed_time + sim.dt
        time_to_wait = sim.dt + start_time - time.time()
        if time_to_wait > 0.:
            time.sleep(time_to_wait)
            