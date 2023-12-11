###############################################################################
#DEPENDENCIES
###############################################################################
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
                           position=[0., 0., 0.6655],
                           fixed=True,
                           update_vis=True)


# Create desired plots then open the animator
plot1, artists1 = sim.add_subplot(n_artists=2,
                                  subplot_type='line',
                                  title="Angles vs Time",
                                  x_label="Time [Seconds]",
                                  y_label="Angles [Rad]",
                                  colors=["r", "b"],
                                  line_widths=[2.5, 2.5],
                                  line_styles=["-", ":"],
                                  labels=["Angle", "Target"])
plot2, artists2 = sim.add_subplot(n_artists=1,
                                  subplot_type='line',
                                  title="Torque vs Time",
                                  x_label="Time [Seconds]",
                                  y_label="Torque [Nm]",
                                  colors=["k"],
                                  line_widths=[2.5])
plot3, artists3 = sim.add_subplot(n_artists=2,
                                  subplot_type='bar',
                                  title="Gains",
                                  x_label="Values [-]",
                                  y_label="Names [-]",
                                  labels=["Proportional", "Derivative"],
                                  colors=["m", "c"],
                                  line_widths=[1.0, 1.0],
                                  x_lim=[0.0, 10.0])
sim.open_animator_gui()

# Set the target angle for the wheel
angle_tag = 3.1415926
P = 3
D = 2

# Wait for user input
sim.await_keypress(key="enter")

# Run the simulation  
while(not sim.is_done):
    
    
    ###########################################################################
    # SENSOR
    # Use a sensor to collect the absolute angle of the wheel and calculate
    # the error from target angle.
    state = sim.get_joint_state(urdf_obj=wheel_obj,
                                joint_name="ground_to_axle")
    angle = state['position']
    angle_vel = state['velocity']
    ###########################################################################
    # CONTROLLER
    # This is the section where you make a controller.
    # Take the information collected by the sensors and somehow calculate
    # what torque you should apply to the wheel to drive the wheel to
    # some target angle. 
    angle_error = angle - angle_tag
    angle_vel_error = angle_vel - 0.0
    torque = -P * angle_error - D*angle_vel_error
    if torque > 5.:
        torque=5.
    elif torque < -5.:
        torque = -5.
    ###########################################################################
    # ACTUATOR
    # Apply the controller calculated torque to the wheel using an actuator.
    sim.set_joint_torque(urdf_obj=wheel_obj,
                          joint_name="ground_to_axle",
                          torque=torque,
                          show_arrow=True,
                          arrow_scale=0.3,
                          arrow_offset=0.52)
    ###########################################################################
    
    # # Plot angle, target angle, and torque
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[0],
                          x=sim.time,
                          y=angle)
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[1],
                          x=sim.time,
                          y=angle_tag)
    sim.add_subplot_point(subplot_index=plot2,
                          artist_index=artists2[0],
                          x=sim.time,
                          y=torque)
    sim.add_subplot_point(subplot_index=plot3,
                          artist_index=artists3[0],
                          x=P)
    sim.add_subplot_point(subplot_index=plot3,
                          artist_index=artists3[1],
                          x=D)
    
    # Collect keyboard IO data for changing the target angle
    if sim.is_pressed('a') and not sim.paused:
        angle_tag = angle_tag + 0.005*6.2831854
        if angle_tag > 3.1415927:
            angle_tag = 3.1415927
    elif sim.is_pressed('d') and not sim.paused:
        angle_tag = angle_tag - 0.005*6.2831854
        if angle_tag < -3.1415927:
            angle_tag = -3.1415927

    # Collect keyboard IO for changing gains
    if sim.is_pressed('r'):
        P = P + 0.005*4.0
        if P > 10.:
            P = 10.
    elif sim.is_pressed('f'):
        P = P - 0.005*4.0
        if P < 0.:
            P = 0.
    if sim.is_pressed('t'):
        D = D + 0.005*4.0
        if D > 10.:
            D = 10.
    elif sim.is_pressed('g'):
        D = D - 0.005*4.0
        if D < 0.:
            D = 0.
            
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
            