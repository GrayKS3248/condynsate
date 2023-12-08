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
for base_pos in cart_coords:
    urdf_id = sim.load_urdf(urdf_path='./spacecraft_vis/sphere.urdf',
                            position=base_pos,
                            fixed=True,
                            update_vis=False)

# Load random stars
np.random.seed(10)
n_stars = 50
rand_depths = depth + (np.random.rand(n_stars)*0.2-0.1)*depth
positions = np.random.randn(n_stars,3)
positions = positions / np.linalg.norm(positions,axis=1).reshape(n_stars,1)
positions = positions * rand_depths.reshape(n_stars,1)
for i in range(n_stars):
    urdf_id = sim.load_urdf(urdf_path='./spacecraft_vis/sphere.urdf',
                            position=positions[i],
                            fixed=True,
                            update_vis=False)

# Set background and lighting properties
sim.set_background(top_color=[10,10,10],
                   bot_color=[20, 20, 40])
sim.set_spotlight(on=False)
sim.set_fill_light(on=False)
sim.set_posx_pt_light(on=True,
                      intensity=0.5,
                      distance=10.)
sim.set_negx_pt_light(on=True,
                      intensity=0.5,
                      distance=10.)
sim.set_ambient_light(on=True,
                      intensity=0.65)

# Variables to track applied torque
max_torque = 1.
min_torque = -1.

# Create desired plots then open the animator
plot,lines = sim.add_subplot(n_lines=3,
                             title="Angles vs Time",
                             x_label="Time [s]",
                             y_label="Angle [Rad]",
                             colors=['r', 'g', 'b'],
                             line_widths=[2.5, 2.5, 2.5],
                             labels=['Roll','Pitch','Yaw'],
                             y_lim=[-np.pi,np.pi])
sim.open_animator_gui()

# Wait for user input
sim.await_keypress(key="enter")

# Run the simulation
done = False
while(not sim.is_done):      
    # Collect keyboard IO data for torques
    if sim.is_pressed("a"):
        torque_1 = min_torque
    elif sim.is_pressed("q"):
        torque_1 = max_torque
    else:
        torque_1 = 0.0
    if sim.is_pressed("s"):
        torque_2 = min_torque
    elif sim.is_pressed("w"):
        torque_2 = max_torque
    else:
        torque_2 = 0.0
    if sim.is_pressed("d"):
        torque_3 = min_torque
    elif sim.is_pressed("e"):
        torque_3 = max_torque
    else:
        torque_3 = 0.0
    if sim.is_pressed("f"):
        torque_4 = min_torque
    elif sim.is_pressed("r"):
        torque_4 = max_torque
    else:
        torque_4 = 0.0

    # Set wheel torques
    sim.set_joint_torque(urdf_obj=craft_obj,
                        joint_name='bus_to_wheel_1',
                        torque=torque_1,
                        show_arrow=True,
                        arrow_scale=2.)
    sim.set_joint_torque(urdf_obj=craft_obj,
                        joint_name='bus_to_wheel_2',
                        torque=torque_2,
                        show_arrow=True,
                        arrow_scale=2.)
    sim.set_joint_torque(urdf_obj=craft_obj,
                        joint_name='bus_to_wheel_3',
                        torque=torque_3,
                        show_arrow=True,
                        arrow_scale=2.)
    sim.set_joint_torque(urdf_obj=craft_obj,
                        joint_name='bus_to_wheel_4',
                        torque=torque_4,
                        show_arrow=True,
                        arrow_scale=2.)
    
    # Set the plot data
    state = sim.get_base_state(urdf_obj=craft_obj,
                                   body_coords=True)
    roll = state['roll']
    pitch = state['pitch']
    yaw = state['yaw']
    sim.add_subplot_point(subplot_index=plot,
                          line_index=lines[0],
                          x=sim.time,
                          y=roll)
    sim.add_subplot_point(subplot_index=plot,
                          line_index=lines[1],
                          x=sim.time,
                          y=pitch)
    sim.add_subplot_point(subplot_index=plot,
                          line_index=lines[2],
                          x=sim.time,
                          y=yaw)
    
    # Step the sim
    sim.step(real_time=True,
             update_vis=True,
             update_ani=True)
            