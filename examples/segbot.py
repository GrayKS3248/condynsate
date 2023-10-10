###############################################################################
#DEPENDENCIES
###############################################################################
import numpy as np
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
                               animation=True,
                               animation_fr=15.)
    
    # Load all urdf objects
    station_radius = 19.59
    station_center = np.array([0., 0., 19.355])
    station_obj = sim.load_urdf(urdf_path='./segbot_vis/station.urdf',
                                position=station_center,
                                roll=np.pi/2.0,
                                fixed=True,
                                update_vis=True)
    segbot_obj = sim.load_urdf(urdf_path='./segbot_vis/segbot.urdf',
                                position=[0., 0., 0.],
                                yaw=np.pi,
                                fixed=False,
                                update_vis=True)
    
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
    
    # Variables to track station velocity
    max_vel = 1.
    min_vel = 0.
    station_vel = 0.025
    
    # Open the animator and create a plot for the torques and phase space
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
    elapsed_time = 0
    done = False
    while(not done):
        # Get the current base position of the robot and change
        # gravity accordingly.
        # This simulates centrifugal gravity from the station
        seg_pos, _, _, _ = np.array(sim.get_base_state(segbot_obj))
        diff = seg_pos - station_center
        dirn = diff / np.linalg.norm(diff)
        grav = 9.81 * dirn
        sim.set_gravity(grav)
        
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
        r_color = format_RGB(r_color,
                             range_to_255=True)
        l_torque = round(l_torque,2)
        l_torque_sat = (l_torque - min_torque) / (max_torque - min_torque)
        l_color = cmaps['coolwarm'](round(255*l_torque_sat))[0:3]
        l_color = format_RGB(l_color,
                             range_to_255=True)
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
        
        # Collect keyboard IO data for station vel
        if keyboard.is_pressed("e"):
            station_vel = station_vel + 0.001*(max_vel - min_vel)
            if station_vel > max_vel:
                station_vel = max_vel
        elif keyboard.is_pressed("q"):
            station_vel = station_vel - 0.001*(max_vel - min_vel)
            if station_vel < min_vel:
                station_vel = min_vel
           
        # Set the torque and torque colors
        station_vel = round(station_vel,4)
        vel_sat = (station_vel - min_vel) / (max_vel - min_vel)
        vel_color = cmaps['Reds'](round(255*vel_sat))[0:3]
        vel_color = format_RGB(vel_color,
                               range_to_255=True)
        sim.set_joint_velocity(urdf_obj=station_obj,
                               joint_name="world_to_station",
                               velocity=station_vel)
        sim.set_link_color(urdf_obj=station_obj,
                           link_name="station",
                           color=vel_color)
        
        # Set the plot data
        times.append(sim.time)
        torques.append(r_torque + l_torque)
        pos, rpy, vel, ang_vel = sim.get_base_state(segbot_obj,
                                                    body_coords=True)
        pitch_vels.append(ang_vel[1])
        sim.set_plot_data(plot_1, times, torques)
        sim.set_plot_data(plot_2, times, pitch_vels)
        
        # Step the sim
        sim.step(real_time=True,
                 update_vis=True,
                 update_ani=True)
        
        # Collect keyboard IO for termination
        if keyboard.is_pressed("esc"):
            done = True
            