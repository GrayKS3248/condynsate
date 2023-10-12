###############################################################################
#DEPENDENCIES
###############################################################################
import numpy as np
import keyboard
import condynsate


###############################################################################
#MAIN LOOP
###############################################################################
if __name__ == "__main__":
    # Create an instance of the simulator with visualization
    sim = condynsate.Simulator(visualization=True,
                               animation=True,
                               animation_fr=15.,
                               gravity=[0., 0., 0.])
    
    # Load the spacecraft
    craft_obj = sim.load_urdf(urdf_path='./spacecraft_vis/spacecraft.urdf',
                              fixed=False,
                              update_vis=True)
    
    # Generate star cartesian coordinates based on their depth and
    # equatorial astronomical coords (right ascension, declination)
    depth = 55.
    eqi_coords = np.array([[-0.06, -0.09],
                           [ 0.00, -0.09],
                           [ 0.06, -0.09],
                           [ 0.00, -0.03],
                           [ 0.00,  0.03],
                           [-0.06,  0.09],
                           [ 0.00,  0.09],
                           [ 0.06,  0.09]])
    cart_coords = []
    for coord in eqi_coords:
        s_a = np.sin(coord[0])
        c_a = np.cos(coord[0])
        s_d = np.sin(coord[1])
        c_d = np.cos(coord[1])
        cart_coord = np.array([c_a*c_d, s_a*c_d, s_d]) * depth
        cart_coords.append(cart_coord)
    
    # Load the constellation
    constellation_objs = []
    for base_pos in cart_coords:
        urdf_id = sim.load_urdf(urdf_path='./spacecraft_vis/sphere.urdf',
                                position=base_pos,
                                fixed=True,
                                update_vis=False)
        constellation_objs.append(urdf_id)
    
    # Load random stars
    np.random.seed(10)
    star_objs = []
    n_stars = 75
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
    
    # Create desired plots then open the animator
    times=[]
    rolls=[]
    pitches=[]
    yaws=[]
    plot_1 = sim.add_plot_to_animator(title="Roll vs Time",
                                      x_label="Time [s]",
                                      y_label="Roll [Rad]",
                                      color="r",
                                      tail=500,
                                      y_lim=[-np.pi,np.pi])
    plot_2 = sim.add_plot_to_animator(title="Pitch Vs Time",
                                      x_label="Time [s]",
                                      y_label="Pitch [Rad]",
                                      color="g",
                                      tail=500,
                                      y_lim=[-np.pi,np.pi])
    plot_3 = sim.add_plot_to_animator(title="Yaw Vs Time",
                                      x_label="Time [s]",
                                      y_label="Yaw [Rad]",
                                      color="b",
                                      tail=500,
                                      y_lim=[-np.pi,np.pi])
    sim.open_animator_gui()
    
    # Wait for user input
    sim.await_keypress(key="enter")
    
    # Run the simulation
    elapsed_time = 0
    done = False
    while(not done):      
        # Collect keyboard IO data for torques
        if keyboard.is_pressed("a"):
            torque_1 = min_torque
        elif keyboard.is_pressed("q"):
            torque_1 = max_torque
        else:
            torque_1 = 0.0
        if keyboard.is_pressed("s"):
            torque_2 = min_torque
        elif keyboard.is_pressed("w"):
            torque_2 = max_torque
        else:
            torque_2 = 0.0
        if keyboard.is_pressed("d"):
            torque_3 = min_torque
        elif keyboard.is_pressed("e"):
            torque_3 = max_torque
        else:
            torque_3 = 0.0
        if keyboard.is_pressed("f"):
            torque_4 = min_torque
        elif keyboard.is_pressed("r"):
            torque_4 = max_torque
        else:
            torque_4 = 0.0

        # Set wheel torques
        sim.set_joint_torque(urdf_obj=craft_obj,
                            joint_name='bus_to_wheel_1',
                            torque=torque_1,
                            show_arrow=True,
                            arrow_scale=2.,
                            color=True,
                            min_torque=min_torque,
                            max_torque=max_torque)
        sim.set_joint_torque(urdf_obj=craft_obj,
                            joint_name='bus_to_wheel_2',
                            torque=torque_2,
                            show_arrow=True,
                            arrow_scale=2.,
                            color=True,
                            min_torque=min_torque,
                            max_torque=max_torque)
        sim.set_joint_torque(urdf_obj=craft_obj,
                            joint_name='bus_to_wheel_3',
                            torque=torque_3,
                            show_arrow=True,
                            arrow_scale=2.,
                            color=True,
                            min_torque=min_torque,
                            max_torque=max_torque)
        sim.set_joint_torque(urdf_obj=craft_obj,
                            joint_name='bus_to_wheel_4',
                            torque=torque_4,
                            show_arrow=True,
                            arrow_scale=2.,
                            color=True,
                            min_torque=min_torque,
                            max_torque=max_torque)
        
        # Set the plot data
        times.append(sim.time)
        pos, rpy, vel, ang_vel = sim.get_base_state(craft_obj,
                                                    body_coords=True)
        rolls.append(rpy[0])
        pitches.append(rpy[1])
        yaws.append(rpy[2])
        sim.set_plot_data(plot_1, times, rolls)
        sim.set_plot_data(plot_2, times, pitches)
        sim.set_plot_data(plot_3, times, yaws)
        
        # Step the sim
        sim.step(real_time=True,
                 update_vis=True,
                 update_ani=True)
        
        # Collect keyboard IO for termination
        if keyboard.is_pressed("esc"):
            done = True
            