###############################################################################
#DEPENDENCIES
###############################################################################
import numpy as np
import condynsate


###############################################################################
#BUILD A CONTROLLER
###############################################################################
def manual_controller(**kwargs):
    # Get the simulator
    sim = kwargs['sim']
    
    # Set the torque to min
    torque = 0.0
    
    # Listen for keyboard presses:
    if sim.is_pressed('d'):
        torque = torque + 0.125
    if sim.is_pressed('shift+d'):
        torque = torque + 0.5
    if sim.is_pressed('a'):
        torque = torque - 0.125
    if sim.is_pressed('shift+a'):
        torque = torque - 0.5

    # Return the manually set torque
    return torque


###############################################################################
#BUILD THE SIMULATOR ENVIRONMENT
###############################################################################
# Create an instance of the simulator with visualization
sim = condynsate.Simulator(visualization=True,
                           animation=True,
                           animation_fr=15.)

# Load urdf objects
ground_obj = sim.load_urdf(urdf_path='./cmg_vis/plane.urdf',
                           tex_path='./cmg_vis/check.png',
                           position=[0., 0., -3.],
                           fixed=True,
                           update_vis=False)
wall_obj = sim.load_urdf(urdf_path='./cmg_vis/plane.urdf',
                         tex_path='./cmg_vis/concrete.png',
                         roll=0.5*np.pi,
                         yaw=np.pi,
                         fixed=True,
                         update_vis=False)
cmg_obj = sim.load_urdf(urdf_path='./cmg_vis/cmg.urdf',
                        position=[0., 1.1, 0.],
                        pitch=-0.5*np.pi,
                        fixed=True,
                        update_vis=True)

# Apply damping to pendulum
sim.set_joint_damping(urdf_obj=cmg_obj,
                      joint_name='world_to_frame',
                      damping=0.3)

# Make plot for phase space
plot1, artists1 = sim.add_subplot(n_artists=1,
                                  subplot_type='line',
                                  title="Phasespace",
                                  x_label="Rate $[Rad-s^{-1}]$",
                                  y_label="Angle $[Rad]$",
                                  colors=["m"],
                                  line_widths=[2.5],
                                  line_styles=["-"],
                                  h_zero_line=True,
                                  v_zero_line=True,
                                  tail=800)

###############################################################################
#SIMULATION LOOP
###############################################################################
# Run the simulation
mass = 1.0
min_mass = 0.0
max_mass = 5.0
wheel_vel = 0.0
min_wheel_vel = 0.0
max_wheel_vel = 100.
sim.open_animator_gui()
sim.await_keypress(key="enter")
while(not sim.is_done):    
    ###########################################################################
    # SENSOR
    # Get the pendulum angle
    state = sim.get_joint_state(urdf_obj=cmg_obj,
                                joint_name="world_to_frame")
    angle = state['position']
    angle_vel = state['velocity']
    
    ###########################################################################
    # CONTROLLER
    torque =  manual_controller(sim=sim)

    ###########################################################################
    # ACTUATOR
    # Set the CMG wheel torque
    sim.set_joint_torque(urdf_obj=cmg_obj,
                         joint_name="frame_to_cage",
                         torque=torque,
                         color=True,
                         min_torque=-0.5,
                         max_torque=0.5)
    
    ###########################################################################
    # UPDATE THE PLOT
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[0],
                          x=angle_vel,
                          y=angle)
    
    ###########################################################################
    # SET THE VARIABLE SIMULATION PARAMETERS
    # Set the ball mass
    sim.set_link_mass(urdf_obj=cmg_obj,
                      link_name='mass',
                      mass = mass,
                      color=True,
                      min_mass=min_mass,
                      max_mass=max_mass)
    
    # Set the wheel velocity
    sim.set_joint_velocity(urdf_obj=cmg_obj,
                           joint_name='cage_to_wheel',
                           velocity = wheel_vel,
                           physics=False,
                           color=True,
                           min_vel=min_wheel_vel,
                           max_vel=max_wheel_vel)
    
    ###########################################################################
    # UI TO CHANGE MASS AND WHEEL VELOCITY
    # Update the mass
    mass = sim.iterate_val(curr_val=mass,
                           down_key='q',
                           up_key='e',
                           iter_val=0.05,
                           min_val=min_mass,
                           max_val=max_mass)
    
    # Update the wheel vel
    wheel_vel = sim.iterate_val(curr_val=wheel_vel,
                                down_key='s',
                                up_key='w',
                                iter_val=0.5,
                                min_val=min_wheel_vel,
                                max_val=max_wheel_vel)
    
    ###########################################################################
    # STEP THE SIMULATION
    sim.step(real_time=True,
             update_vis=True,
             update_ani=True)
            