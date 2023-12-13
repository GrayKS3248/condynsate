###############################################################################
#DEPENDENCIES
###############################################################################
import numpy as np
import condynsate



###############################################################################
#MAIN LOOP
###############################################################################
# Create an instance of the simulator with visualization
sim = condynsate.Simulator(visualization=True,
                           animation=False,
                           animation_fr=15.)

# Load all urdf objects
station_obj = sim.load_urdf(urdf_path='./segbot_vis/station.urdf',
                            position=[0., 0., 18.],
                            fixed=True,
                            update_vis=True)
segbot_obj = sim.load_urdf(urdf_path='./segbot_vis/segbot.urdf',
                            position=[0., 0., 1.2],
                            fixed=False,
                            update_vis=True)

# Variables to track applied torque
max_torque = 5.
min_torque = -5.

# Variables to track station velocity
max_stat_vel = 0.05
min_stat_vel = 0.
stat_vel = 0.5 * (min_stat_vel + max_stat_vel)

# Create desired plots then open the animator
plot_1, lines_1 = sim.add_subplot(n_artists=3,
                                  title="Angles",
                                  x_label="Time [s]",
                                  y_label="Angles [rad]",
                                  colors=["r","g","b"],
                                  labels=['roll', 'pitch', 'yaw'],
                                  line_widths=[2.5, 2.5, 2.5])
sim.open_animator_gui()

# # Wait for user input
sim.await_keypress(key="enter")

# Run the simulation
while(not sim.is_done):
    
    # Collect keyboard IO data for torque
    if sim.is_pressed("shift+w"):
        r_torque = 0.75 * min_torque
        l_torque = 0.75 * min_torque
    elif sim.is_pressed("w"):
        r_torque = 0.33 * min_torque
        l_torque = 0.33 * min_torque
    elif sim.is_pressed("shift+s"):
        r_torque = 0.75 * max_torque
        l_torque = 0.75 * max_torque
    elif sim.is_pressed("s"):
        r_torque = 0.33 * max_torque
        l_torque = 0.33 * max_torque
    else:
        r_torque = 0.0
        l_torque = 0.0
    if sim.is_pressed("d"):
        l_torque = l_torque + 0.25*max_torque
    if sim.is_pressed("a"):
        r_torque = r_torque + 0.25*max_torque
    
    # Collect keyboard IO data for station vel
    if sim.is_pressed("e"):
        stat_vel = stat_vel + 0.005*(max_stat_vel - min_stat_vel)
        if stat_vel > max_stat_vel:
            stat_vel = max_stat_vel
    elif sim.is_pressed("q"):
        stat_vel = stat_vel - 0.005*(max_stat_vel - min_stat_vel)
        if stat_vel < min_stat_vel:
            stat_vel = min_stat_vel
    
    # Set wheel torques
    sim.set_joint_torque(urdf_obj=segbot_obj,
                          joint_name='chassis_to_right_wheel',
                          torque=r_torque,
                          show_arrow=True,
                          arrow_scale=1,
                          arrow_offset=0.5)
    sim.set_joint_torque(urdf_obj=segbot_obj,
                          joint_name='chassis_to_left_wheel',
                          torque=l_torque,
                          show_arrow=True,
                          arrow_scale=1,
                          arrow_offset=-0.5)
    
    # Set station velocity
    sim.set_joint_velocity(urdf_obj=station_obj,
                            joint_name="world_to_station",
                            velocity=stat_vel,
                            color=True,
                            min_vel=min_stat_vel,
                            max_vel=max_stat_vel)
    
    # Get the rigid body states of the segbot and station
    seg_state = sim.get_base_state(urdf_obj=segbot_obj,
                                    body_coords=True)
    stat_state = sim.get_base_state(urdf_obj=station_obj,
                                    body_coords=False)
    seg_pos = seg_state['position']
    seg_ang_vel = seg_state['angular velocity']
    stat_pos = stat_state['position']
    
    # This simulates centrifugal gravity from the station
    seg_relative_pos = np.array(seg_pos) - np.array(stat_pos)
    seg_relative_dirn = seg_relative_pos / np.linalg.norm(seg_relative_pos)
    gravity = 9.81 * seg_relative_dirn
    sim.set_gravity(gravity=gravity)
    
    # Set the plot data
    sim.add_subplot_point(subplot_index=plot_1,
                          artist_index=lines_1[0],
                          x=sim.time,
                          y=seg_state['roll'])
    sim.add_subplot_point(subplot_index=plot_1,
                          artist_index=lines_1[1],
                          x=sim.time,
                          y=seg_state['pitch'])
    sim.add_subplot_point(subplot_index=plot_1,
                          artist_index=lines_1[2],
                          x=sim.time,
                          y=seg_state['yaw'])
    
    # Step the sim
    sim.step(real_time=True,
              update_vis=True,
              update_ani=False)
            