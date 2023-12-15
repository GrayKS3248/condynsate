###############################################################################
#DEPENDENCIES
###############################################################################
import condynsate


###############################################################################
#BUILD A CONTROLLER
###############################################################################
# This is the section where you make a controller.
# Take the information collected by the sensors and somehow calculate
# what torque you should apply to the wheel to drive the wheel to
# some target angle. 
def controller(**kwargs):
    # PD method
    try:
        prop = kwargs['ang'] - kwargs['tag']
        derv = kwargs['ang_vel']
        P = kwargs['P']
        D = kwargs['D']
        torque = -P*prop - D*derv
        if torque > 5.0:
            torque = 5.0
        elif torque < -5.0:
            torque = -5.0
            
    # Single value method
    except:
        print("Did not get enough arguments to controller(). Passing")
        torque = 0.0
    
    # Return the torque
    return torque


###############################################################################
#BUILD THE SIMULATOR ENVIRONMENT
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


###############################################################################
#SIMULATION LOOP
###############################################################################
# Define the target angle and some useful constants before the simulation
angle_tag = 3.1415926
P = 0.0
D = 0.0

# Open the animator GUI, wait for user input, then run the simulation  
sim.open_animator_gui()
sim.await_keypress(key="enter")
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
    # Get the torque as calculated by the controller
    torque = controller(tag=angle_tag,
                        ang=angle,
                        ang_vel=angle_vel,
                        P=P,
                        D=D)
    
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
    # UPDATE THE PLOTS
    # Plot angle and target angle vs time
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[0],
                          x=sim.time,
                          y=angle)
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[1],
                          x=sim.time,
                          y=angle_tag)
    
    # Plot torque vs time
    sim.add_subplot_point(subplot_index=plot2,
                          artist_index=artists2[0],
                          x=sim.time,
                          y=torque)
    
    # Plot the P and D variables
    sim.add_subplot_point(subplot_index=plot3,
                          artist_index=artists3[0],
                          x=P)
    sim.add_subplot_point(subplot_index=plot3,
                          artist_index=artists3[1],
                          x=D)
    
    ###########################################################################
    # UPDATE THE TARGET ANGLE
    # Iterate the target angle
    angle_tag = sim.iterate_val(curr_val=angle_tag,
                                down_key='a',
                                up_key='d',
                                iter_val=0.03,
                                min_val=-3.1415927,
                                max_val=3.1415927)
    
    # Adjust the target arrow so that it is always 
    # pointing in the target angle direction
    sim.set_joint_position(urdf_obj=target_obj,
                            joint_name='world_to_arrow',
                            position=angle_tag,
                            physics=False)
    
    ###########################################################################
    # UPDATE THE CONTROL GAINS
    # Iterate the proportional and derivative gains
    P = sim.iterate_val(curr_val=P,
                        down_key='f',
                        up_key='r',
                        iter_val=0.02,
                        min_val=0,
                        max_val=10)
    D = sim.iterate_val(curr_val=D,
                        down_key='g',
                        up_key='t',
                        iter_val=0.02,
                        min_val=0,
                        max_val=10)
    
    ###########################################################################
    # STEP THE SIMULATION
    # Step the sim
    sim.step(real_time=True,
              update_vis=True,
              update_ani=True)
            