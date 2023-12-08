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

# Load all objects
ground_obj = sim.load_urdf(urdf_path='./quadcopter_vis/plane.urdf',
                            tex_path='./quadcopter_vis/wave.png',
                            position=[0., 0., -3.],
                            wxyz_quaternion=[1., 0., 0., 0],
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

# Variables to track torque
min_torque = 0.
max_torque = 5.

# Create desired plots then open the animator
plot_1 = sim.add_plot_to_animator(title="Potention Energy vs Kinetic",
                                  x_label="Kinetic Energy [J]",
                                  y_label="Potention Energy [J]",
                                  color="r",
                                  tail=500)
sim.open_animator_gui()

# Wait for user input
sim.await_keypress(key="enter")

# Run the simulation
done = False
while(not sim.is_done):
    # Collect keyboard IO data for torques
    torque_1 = -min_torque
    torque_2 = min_torque
    torque_3 = -min_torque
    torque_4 = min_torque
    if sim.is_pressed("a"):
        torque_1 = -max_torque
    if sim.is_pressed("s"):
        torque_2 = max_torque
    if sim.is_pressed("d"):
        torque_3 = -max_torque
    if sim.is_pressed("f"):
        torque_4 = max_torque
        
    # Set torques
    sim.set_joint_torque(urdf_obj=quad_obj,
                        joint_name='spar1_to_rotor1',
                        torque=torque_1,
                        show_arrow=True,
                        arrow_scale=0.1,
                        color=False)
    sim.set_joint_torque(urdf_obj=quad_obj,
                        joint_name='spar2_to_rotor2',
                        torque=torque_2,
                        show_arrow=True,
                        arrow_scale=0.1,
                        color=False)
    sim.set_joint_torque(urdf_obj=quad_obj,
                        joint_name='spar3_to_rotor3',
                        torque=torque_3,
                        show_arrow=True,
                        arrow_scale=0.1,
                        color=False)
    sim.set_joint_torque(urdf_obj=quad_obj,
                        joint_name='spar4_to_rotor4',
                        torque=torque_4,
                        show_arrow=True,
                        arrow_scale=0.1,
                        color=False)
    
    # Color based on velocity
    sim.set_color_from_vel(urdf_obj=quad_obj,
                            joint_name='spar1_to_rotor1',
                            min_vel=-100.,
                            max_vel=100.)
    sim.set_color_from_vel(urdf_obj=quad_obj,
                            joint_name='spar2_to_rotor2',
                            min_vel=-100.,
                            max_vel=100.)
    sim.set_color_from_vel(urdf_obj=quad_obj,
                            joint_name='spar3_to_rotor3',
                            min_vel=-100.,
                            max_vel=100.)
    sim.set_color_from_vel(urdf_obj=quad_obj,
                            joint_name='spar4_to_rotor4',
                            min_vel=-100.,
                            max_vel=100.)
    
    # Retrieve the joint states for force calculation
    state1 = sim.get_joint_state(urdf_obj=quad_obj,
                                 joint_name="spar1_to_rotor1")
    state2 = sim.get_joint_state(urdf_obj=quad_obj,
                                 joint_name="spar2_to_rotor2")
    state3 = sim.get_joint_state(urdf_obj=quad_obj,
                                 joint_name="spar3_to_rotor3")
    state4 = sim.get_joint_state(urdf_obj=quad_obj,
                                 joint_name="spar4_to_rotor4")
    vel_1 = state1['velocity']
    vel_2 = state2['velocity']
    vel_3 = state3['velocity']
    vel_4 = state4['velocity']
    
    # Set force based on velocity
    sim.apply_force_to_link(urdf_obj=quad_obj,
                            link_name='rotor1',
                            force=[0., 0.001*vel_1, -0.05*vel_1],
                            show_arrow=True,
                            arrow_scale=0.40)
    sim.apply_force_to_link(urdf_obj=quad_obj,
                            link_name='rotor2',
                            force=[0., 0.001*vel_2, 0.05*vel_2],
                            show_arrow=True,
                            arrow_scale=0.40,
                            arrow_offset=1.)
    sim.apply_force_to_link(urdf_obj=quad_obj,
                            link_name='rotor3',
                            force=[0., 0.001*vel_3, -0.05*vel_3],
                            show_arrow=True,
                            arrow_scale=0.40)
    sim.apply_force_to_link(urdf_obj=quad_obj,
                            link_name='rotor4',
                            force=[0., 0.001*vel_4, 0.05*vel_4],
                            show_arrow=True,
                            arrow_scale=0.40)
    
    # Set the plot data
    pos,_,vel,_ = sim.get_base_state(quad_obj,
                                      body_coords=False)
    pot_eng = 0.1*9.81*(pos[2]+3.)
    kin_eng = 0.5*0.1*(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2])
    sim.add_plot_point(plot_1, kin_eng, pot_eng)
    
    # Step the sim
    sim.step(real_time=True,
             update_vis=True,
             update_ani=True)
            