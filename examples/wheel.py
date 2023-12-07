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
                           animation=True,
                           animation_fr=15.)

# Load urdf objects
wheel_obj = sim.load_urdf(urdf_path='./wheel_vis/wheel.urdf',
                          position=[0., 0., 0.],
                          fixed=True,
                          update_vis=True)
target_obj = sim.load_urdf(urdf_path='./wheel_vis/target_arrow.urdf',
                           position=[0., 0., 0.],
                           fixed=True,
                           update_vis=True)


# Create desired plots then open the animator
plot1, lines1 = sim.add_subplot_to_animator(n_lines=2,
                                            title="Angle vs Time",
                                            x_label="Time [Seconds]",
                                            y_label="$θ-θ_{0}$ [Rad]",
                                            colors=["r", "b"],
                                            line_widths=[2.5, 2.5],
                                            line_styles=["-", ":"])
plot2, lines2 = sim.add_subplot_to_animator(n_lines=1,
                                            title="Torque vs Time",
                                            x_label="Time [Seconds]",
                                            y_label="Torque [Nm]",
                                            colors=["k"],
                                            line_widths=[2.5])
sim.open_animator_gui()

# Set the target angle for the wheel
angle_tag = np.pi/2.

# Wait for user input
sim.await_keypress(key="enter")

# Run the simulation
while(not sim.is_done):
    
    
    ###########################################################################
    # SENSOR
    # Use a sensor to collect the absolute angle of the wheel and calculate
    # the error from target angle.
    angle,angle_vel,_,_,_ = sim.get_joint_state(urdf_obj=wheel_obj,
                                                joint_name="ground_to_axle")
    ###########################################################################
    # CONTROLLER
    # This is the section where you make a controller.
    # Take the information collected by the sensors and somehow calculate
    # what torque you should apply to the wheel to drive the wheel to
    # some target angle. 
    angle_error = angle - angle_tag
    angle_vel_error = angle_vel - 0.0
    torque = -3.0 * angle_error - 2.0*angle_vel_error
    ###########################################################################
    # ACTUATOR
    # Apply the controller calculated torque to the wheel using an actuator.
    sim.set_joint_torque(urdf_obj=wheel_obj,
                          joint_name="ground_to_axle",
                          torque=torque)
    ###########################################################################
    
    
    # Plot angle, target angle, and torque
    sim.add_subplot_point(subplot_index=plot1,
                          line_index=lines1[0],
                          x=sim.time,
                          y=angle)
    sim.add_subplot_point(subplot_index=plot1,
                          line_index=lines1[1],
                          x=sim.time,
                          y=angle_tag)
    sim.add_subplot_point(subplot_index=plot2,
                          line_index=lines2[0],
                          x=sim.time,
                          y=torque)
    
    # Collect keyboard IO data for changing the target angle
    if sim.is_pressed('a'):
        angle_tag = angle_tag + 0.005*2*np.pi
        if angle_tag > np.pi:
            angle_tag = np.pi
    elif sim.is_pressed('d'):
        angle_tag = angle_tag - 0.005*2*np.pi
        if angle_tag < -np.pi:
            angle_tag = -np.pi

    # Adjust the target arrow so that it is always 
    # pointing in the target angle direction
    sim.set_joint_position(urdf_obj=target_obj,
                           joint_name='world_to_arrow',
                           position=angle_tag,
                           physics=False)

    # Step the sim
    sim.step(real_time=True,
              update_vis=True,
              update_ani=True)
            