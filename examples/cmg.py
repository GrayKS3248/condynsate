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

# Set joint damping
sim.set_joint_damping(urdf_obj=cmg_obj,
                      joint_name="world_to_outer",
                      damping=0.01)
sim.set_joint_damping(urdf_obj=cmg_obj,
                      joint_name="outer_to_inner",
                      damping=0.01)

# Variables to track applied torque
max_torque = 0.5
min_torque = -0.5

# Variables to track mass
max_mass = 2.0
min_mass = 0.0
mass = 0.5*(max_mass + min_mass)

# Variables to track wheel velocity
max_vel = 100.0
min_vel = 0.0
vel = 0.5 * (max_vel + min_vel)

# Create desired plots then open the animator
ang_momentums = []
angles = []
plot = sim.add_plot_to_animator(title="Phase Space",
                                x_label="θ [Rad]",
                                y_label="L $[Kg•m^{2}•s^{-1}]$",
                                color="r",
                                tail=800,
                                line_width=2.5)
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
                         show_arrow=True,
                         arrow_scale=2.5,
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

    # Set the plot data
    ang, ang_vel,_,_,_ = sim.get_joint_state(urdf_obj=cmg_obj,
                                             joint_name="world_to_outer")
    ang_momentum = mass*ang_vel
    ang_momentums.append(ang_momentum)
    angles.append(ang)
    sim.set_plot_data(plot_index=plot,
                      x=angles,
                      y=ang_momentums)

    # Step the sim
    sim.step(real_time=True,
             update_vis=True,
             update_ani=True)
            