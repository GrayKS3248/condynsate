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
                               animation_fr=15.)
    
    # Load all urdf objects
    station_obj = sim.load_urdf(urdf_path='./segbot_vis/station.urdf',
                                position=[0., 0., 19.355],
                                roll=np.pi/2.0,
                                fixed=True,
                                update_vis=True)
    segbot_obj = sim.load_urdf(urdf_path='./segbot_vis/segbot.urdf',
                                position=[0., 0., 0.],
                                yaw=np.pi,
                                fixed=False,
                                update_vis=True)
    
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
    
    # Variables to track applied torque
    max_torque = 10.
    min_torque = -10.
    
    # Variables to track station velocity
    max_stat_vel = 0.05
    min_stat_vel = 0.
    stat_vel = 0.5 * (min_stat_vel + max_stat_vel)
    
    # Create desired plots then open the animator
    times=[]
    torques=[]
    plot_1 = sim.add_plot_to_animator(title="Torque vs Time",
                                      x_label="Time [s]",
                                      y_label="Torque [Nm]",
                                      color="r",
                                      tail=500)
    pitches = []
    pitch_vels = []
    plot_2 = sim.add_plot_to_animator(title="Pitch Speed Vs Time",
                                      x_label="Time [s]",
                                      y_label="Pitch Velocity [Rad/s]",
                                      color="b",
                                      tail=500)
    sim.open_animator_gui()
    
    # Wait for user input
    sim.await_keypress(key="enter")
    
    # Run the simulation
    done = False
    while(not done):
        
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
        
        # Collect keyboard IO data for station vel
        if keyboard.is_pressed("e"):
            stat_vel = stat_vel + 0.005*(max_stat_vel - min_stat_vel)
            if stat_vel > max_stat_vel:
                stat_vel = max_stat_vel
        elif keyboard.is_pressed("q"):
            stat_vel = stat_vel - 0.005*(max_stat_vel - min_stat_vel)
            if stat_vel < min_stat_vel:
                stat_vel = min_stat_vel
        
        # Set wheel torques
        sim.set_joint_torque(urdf_obj=segbot_obj,
                             joint_name='chassis_to_right_wheel',
                             torque=r_torque,
                             show_arrow=True,
                             arrow_scale=0.1,
                             color=True,
                             min_torque=min_torque,
                             max_torque=max_torque)
        sim.set_joint_torque(urdf_obj=segbot_obj,
                             joint_name='chassis_to_left_wheel',
                             torque=l_torque,
                             show_arrow=True,
                             arrow_scale=0.1,
                             color=True,
                             min_torque=min_torque,
                             max_torque=max_torque)
        
        # Set station velocity
        sim.set_joint_velocity(urdf_obj=station_obj,
                               joint_name="world_to_station",
                               velocity=stat_vel,
                               color=True,
                               min_vel=min_stat_vel,
                               max_vel=max_stat_vel)
        
        # Get the rigid body states of the segbot and station
        seg_pos,_,_,seg_ang_vel = sim.get_base_state(urdf_obj=segbot_obj,
                                                     body_coords=True)
        stat_pos,_,_,_ = sim.get_base_state(urdf_obj=station_obj,
                                            body_coords=False)
        
        # This simulates centrifugal gravity from the station
        seg_relative_pos = np.array(seg_pos) - np.array(stat_pos)
        seg_relative_dirn = seg_relative_pos / np.linalg.norm(seg_relative_pos)
        gravity = 9.81 * seg_relative_dirn
        sim.set_gravity(gravity=gravity)
        
        # Set the plot data
        times.append(sim.time)
        torques.append(0.5*(r_torque + l_torque))
        pitch_vels.append(seg_ang_vel[1])
        sim.set_plot_data(plot_1, times, torques)
        sim.set_plot_data(plot_2, times, pitch_vels)
        
        # Step the sim
        sim.step(real_time=True,
                 update_vis=True,
                 update_ani=True)
        
        # Collect keyboard IO for termination
        if keyboard.is_pressed("esc"):
            done = True
            