###############################################################################
#DEPENDENCIES
###############################################################################
import condynsate


###############################################################################
#BUILD A CONTROLLER
###############################################################################
def manual_controller(**kwargs):
    # Get the simulator
    sim = kwargs['sim']
    
    # Set the torques to min
    torques = {'torque1' : 0.01,
               'torque2' : 0.01,
               'torque3' : 0.01,
               'torque4' : 0.01}
    
    # Listen for keyboard presses:
    if sim.is_pressed('a'):
        torques['torque1'] = 5.0
    if sim.is_pressed('s'):
        torques['torque2'] = 5.0
    if sim.is_pressed('d'):
        torques['torque3'] = 5.0
    if sim.is_pressed('f'):
        torques['torque4'] = 5.0

    # Return the manually set torques
    return torques


###############################################################################
#BUILD THE SIMULATOR ENVIRONMENT
###############################################################################
# Create an instance of the simulator with visualization
sim = condynsate.Simulator(visualization=True,
                           animation=True,
                           animation_fr=15.)

# Load all objects
ground_obj = sim.load_urdf(urdf_path='./quadcopter_vis/plane.urdf',
                            tex_path='./quadcopter_vis/wave.png',
                            position=[0., 0., -0.051],
                            fixed=True,
                            update_vis=False)
quad_obj = sim.load_urdf(urdf_path='./quadcopter_vis/quadcopter.urdf',
                         tex_path='./quadcopter_vis/nametag.png',
                         fixed=False,
                         update_vis=True)

# Apply damping to rotors
sim.set_joint_damping(urdf_obj=quad_obj,
                      joint_name='spar1_to_rotor1',
                      damping=0.1)
sim.set_joint_damping(urdf_obj=quad_obj,
                      joint_name='spar2_to_rotor2',
                      damping=0.1)
sim.set_joint_damping(urdf_obj=quad_obj,
                      joint_name='spar3_to_rotor3',
                      damping=0.1)
sim.set_joint_damping(urdf_obj=quad_obj,
                      joint_name='spar4_to_rotor4',
                      damping=0.1)

# Make plot for phase space
plot1, artists1 = sim.add_subplot(n_artists=1,
                                  subplot_type='line',
                                  title="Phasespace",
                                  x_label="Vert Momentum $[Kg•m•s^{-1}]$",
                                  y_label="Altitude $[m]$",
                                  colors=["m"],
                                  line_widths=[2.5],
                                  line_styles=["-"],
                                  v_zero_line=True,
                                  y_lim=[0.0, None],
                                  tail=500)

# Make plot for altitude
plot2, artists2 = sim.add_subplot(n_artists=1,
                                  subplot_type='line',
                                  title="Altitude vs Time",
                                  x_label="Time [s]",
                                  y_label="Altitude [m]",
                                  y_lim=[0.0, None],
                                  colors=["c"],
                                  line_widths=[2.5],
                                  line_styles=["-"])


###############################################################################
#SIMULATION LOOP
###############################################################################
# Run the simulation
sim.open_animator_gui()
sim.await_keypress(key="enter")
while(not sim.is_done):
    ###########################################################################
    # SENSOR
    # Use a sensor to collect the rotor speeds
    rotor1 = sim.get_joint_state(urdf_obj=quad_obj,
                                 joint_name="spar1_to_rotor1")
    rotor2 = sim.get_joint_state(urdf_obj=quad_obj,
                                 joint_name="spar2_to_rotor2")
    rotor3 = sim.get_joint_state(urdf_obj=quad_obj,
                                 joint_name="spar3_to_rotor3")
    rotor4 = sim.get_joint_state(urdf_obj=quad_obj,
                                 joint_name="spar4_to_rotor4")
    rotor1_vel = rotor1['velocity']
    rotor2_vel = rotor2['velocity']
    rotor3_vel = rotor3['velocity']
    rotor4_vel = rotor4['velocity']
    
    # Use a sensor to get the position and velocity of the quadrotor's CoM
    state = sim.get_base_state(quad_obj,
                               body_coords=False)
    com_pos = state['position']
    com_vel = state['velocity']
        
    ###########################################################################
    # CONTROLLER
    torques = manual_controller(sim=sim)
    torque1 = torques['torque1']
    torque2 = torques['torque2']
    torque3 = torques['torque3']
    torque4 = torques['torque4']
    
    ###########################################################################
    # ACTUATOR
    # Apply torque to each rotor
    sim.set_joint_torque(urdf_obj=quad_obj,
                        joint_name='spar1_to_rotor1',
                        torque=torque1,
                        show_arrow=True,
                        arrow_offset=-0.1)
    sim.set_joint_torque(urdf_obj=quad_obj,
                        joint_name='spar2_to_rotor2',
                        torque=torque2,
                        show_arrow=True,
                        arrow_offset=0.1)
    sim.set_joint_torque(urdf_obj=quad_obj,
                        joint_name='spar3_to_rotor3',
                        torque=torque3,
                        show_arrow=True,
                        arrow_offset=-0.1)
    sim.set_joint_torque(urdf_obj=quad_obj,
                        joint_name='spar4_to_rotor4',
                        torque=torque4,
                        show_arrow=True,
                        arrow_offset=0.1)
    
    ###########################################################################
    # UPDATE THE PLOTS
    # Get the momentum and the derivative of the momentum
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[0],
                          x=0.5*com_vel[2],
                          y=com_pos[2])
    sim.add_subplot_point(subplot_index=plot2,
                          artist_index=artists2[0],
                          x=sim.time,
                          y=com_pos[2])
    
    ###########################################################################
    # ROTOR PHYSICS
    # Set rotor force based on velocity
    sim.apply_force_to_link(urdf_obj=quad_obj,
                            link_name='rotor1',
                            force=[0., 0., 0.05*rotor1_vel],
                            show_arrow=True,
                            arrow_scale=0.40)
    sim.apply_force_to_link(urdf_obj=quad_obj,
                            link_name='rotor2',
                            force=[0., 0., 0.05*rotor2_vel],
                            show_arrow=True,
                            arrow_scale=0.40)
    sim.apply_force_to_link(urdf_obj=quad_obj,
                            link_name='rotor3',
                            force=[0., 0., 0.05*rotor3_vel],
                            show_arrow=True,
                            arrow_scale=0.40)
    sim.apply_force_to_link(urdf_obj=quad_obj,
                            link_name='rotor4',
                            force=[0., 0., 0.05*rotor4_vel],
                            show_arrow=True,
                            arrow_scale=0.40)
    
    ###########################################################################
    # STEP THE SIMULATION
    sim.step(real_time=True,
             update_vis=True,
             update_ani=True)
            