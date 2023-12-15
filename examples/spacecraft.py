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
    torques = {'torque1' : 0.0,
               'torque2' : 0.0,
               'torque3' : 0.0,
               'torque4' : 0.0,}
    
    # Listen for keyboard presses:
    if sim.is_pressed('a'):
        torques['torque1'] = torques['torque1'] - 1.0
    if sim.is_pressed('q'):
        torques['torque1'] = torques['torque1'] + 1.0
    if sim.is_pressed('s'):
        torques['torque2'] = torques['torque2'] - 1.0
    if sim.is_pressed('w'):
        torques['torque2'] = torques['torque2'] + 1.0
    if sim.is_pressed('d'):
        torques['torque3'] = torques['torque3'] - 1.0
    if sim.is_pressed('e'):
        torques['torque3'] = torques['torque3'] + 1.0
    if sim.is_pressed('f'):
        torques['torque4'] = torques['torque4'] - 1.0
    if sim.is_pressed('r'):
        torques['torque4'] = torques['torque4'] + 1.0
        
    # Return the manually set torque
    return torques


###############################################################################
#BUILD THE SIMULATOR ENVIRONMENT
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

# Make plot for phase space
plot1, artists1 = sim.add_subplot(n_artists=3,
                                  subplot_type='line',
                                  title="Euler Angles vs. Time",
                                  x_label="Time [s]",
                                  y_label="Angle [Rad]",
                                  colors=['g', 'r', 'b'],
                                  labels=['Roll', 'Pitch', 'Yaw'],
                                  line_widths=[2.5, 2.5, 2.5],
                                  line_styles=["-", "-", "-"],
                                  h_zero_line=True)


###############################################################################
#SIMULATION LOOP
###############################################################################
# Run the simulation
sim.open_animator_gui()
sim.await_keypress(key="enter")
while(not sim.is_done):      
    ###########################################################################
    # SENSOR
    # Get the Euler angles of the craft
    state = sim.get_base_state(urdf_obj=craft_obj,
                               body_coords=True)
    roll = state['roll']
    pitch = state['pitch']
    yaw = state['yaw']
    
    ###########################################################################
    # CONTROLLER
    torques =  manual_controller(sim=sim)
    
    ###########################################################################
    # ACTUATOR
    # Set wheel torques
    sim.set_joint_torque(urdf_obj=craft_obj,
                        joint_name='bus_to_wheel_1',
                        torque=torques['torque1'],
                        show_arrow=True,
                        arrow_scale=2.)
    sim.set_joint_torque(urdf_obj=craft_obj,
                        joint_name='bus_to_wheel_2',
                        torque=torques['torque2'],
                        show_arrow=True,
                        arrow_scale=2.)
    sim.set_joint_torque(urdf_obj=craft_obj,
                        joint_name='bus_to_wheel_3',
                        torque=torques['torque3'],
                        show_arrow=True,
                        arrow_scale=2.)
    sim.set_joint_torque(urdf_obj=craft_obj,
                        joint_name='bus_to_wheel_4',
                        torque=torques['torque4'],
                        show_arrow=True,
                        arrow_scale=2.)
    
    ###########################################################################
    # UPDATE THE PLOT
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[0],
                          x=sim.time,
                          y=roll)
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[1],
                          x=sim.time,
                          y=pitch)
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[2],
                          x=sim.time,
                          y=yaw)
    
    ###########################################################################
    # STEP THE SIMULATION
    sim.step(real_time=True,
             update_vis=True,
             update_ani=True)
            