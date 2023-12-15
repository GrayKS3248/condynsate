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

# Make plot for phase space
plot1, artists1 = sim.add_subplot(n_artists=1,
                                  subplot_type='line',
                                  title="Phasespace",
                                  x_label="Momentum $[Kg•m•s^{-1}]$",
                                  y_label="Rate $[Kg•m•s^{-2}]$",
                                  colors=["m"],
                                  line_widths=[2.5],
                                  line_styles=["-"],
                                  tail=750)
sim.open_animator_gui()

# Wait for user input
sim.await_keypress(key="enter")

# Run the simulation
while(not sim.is_done):    
    
    # Collect keyboard IO data for torque
    if sim.is_pressed("shift+d"):
        torque = max_torque
    elif sim.is_pressed("d"):
        torque = max_torque / 4.0
    elif sim.is_pressed("shift+a"):
        torque = min_torque
    elif sim.is_pressed("a"):
        torque = min_torque / 4.0
    else:
        torque = 0.0

    # Collect keyboard IO data for mass
    if sim.is_pressed('e'):
        mass = mass + 0.005*(max_mass - min_mass)
        if mass > max_mass:
            mass = max_mass
    elif sim.is_pressed('q'):
        mass = mass - 0.005*(max_mass - min_mass)
        if mass < min_mass:
            mass = min_mass

    # Collect keyboard IO data for wheel vel
    if sim.is_pressed('w'):
        vel = vel + 0.005*(max_vel - min_vel)
        if vel > max_vel:
            vel = max_vel
    elif sim.is_pressed('s'):
        vel = vel - 0.005*(max_vel - min_vel)
        if vel < min_vel:
            vel = min_vel

    # Set the torque
    sim.set_joint_torque(urdf_obj=cmg_obj,
                         joint_name="outer_to_inner",
                         torque=torque,
                         color=True,
                         min_torque=min_torque,
                         max_torque=max_torque)

    # Set the mass
    sim.set_link_mass(urdf_obj=cmg_obj,
                      link_name='mass',
                      mass = mass,
                      color=True,
                      min_mass=min_mass,
                      max_mass=max_mass)

    # Set wheel velocity
    sim.set_joint_velocity(urdf_obj=cmg_obj,
                           joint_name='inner_to_wheel',
                           velocity = vel,
                           color=True,
                           min_vel=min_vel,
                           max_vel=max_vel)

    # Add the current datapoint to the plot
    state = sim.get_joint_state(urdf_obj=cmg_obj,
                                joint_name="world_to_outer")
    ang = state['position']
    ang_vel = state['velocity']
    ang_momentum = mass*ang_vel
    sim.add_subplot_point(subplot_index=plot,
                          line_index=lines[0],
                          x=ang,
                          y=ang_momentum)

    # Step the sim
    sim.step(real_time=True,
             update_vis=True,
             update_ani=True)
            