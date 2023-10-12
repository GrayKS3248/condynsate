###############################################################################
#DEPENDENCIES
###############################################################################
import keyboard
import condynsate


###############################################################################
#MAIN LOOP
###############################################################################
if __name__ == "__main__":
    # Create an instance of the simulator with visualization
    sim = condynsate.Simulator(visualization=True,
                               animation=True,
                               animation_fr=3.)
    
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
    
    # Variables to track applied RPM
    max_torque = 5.
    
    # Create desired plots then open the animator
    pot_engs=[]
    kin_engs=[]
    plot_1 = sim.add_plot_to_animator(title="Potention Energy vs Kinetic",
                                      x_label="Kinetic Energy [J]",
                                      y_label="Potention Energy [J]",
                                      color="r",
                                      tail=50)
    sim.open_animator_gui()
    
    # Wait for user input
    sim.await_keypress(key="enter")
    
    # Run the simulation
    elapsed_time = 0
    done = False
    while(not done):
        # Collect keyboard IO data for torque 1
        torque_1 = 0.
        if keyboard.is_pressed("a"):
            torque_1 = -max_torque
        sim.set_joint_torque(urdf_obj=quad_obj,
                            joint_name='spar1_to_rotor1',
                            torque=torque_1,
                            show_arrow=True,
                            arrow_scale=0.1)
        
        # Collect keyboard IO data for torque 2
        torque_2 = 0.
        if keyboard.is_pressed("s"):
            torque_2 = max_torque
        sim.set_joint_torque(urdf_obj=quad_obj,
                            joint_name='spar2_to_rotor2',
                            torque=torque_2,
                            show_arrow=True,
                            arrow_scale=0.1)
        
        # Collect keyboard IO data for torque 3
        torque_3 = 0.
        if keyboard.is_pressed("d"):
            torque_3 = -max_torque
        sim.set_joint_torque(urdf_obj=quad_obj,
                            joint_name='spar3_to_rotor3',
                            torque=torque_3,
                            show_arrow=True,
                            arrow_scale=0.1)
        
        # Collect keyboard IO data for torque 4
        torque_4 = 0.
        if keyboard.is_pressed("f"):
            torque_4 = max_torque
        sim.set_joint_torque(urdf_obj=quad_obj,
                            joint_name='spar4_to_rotor4',
                            torque=torque_4,
                            show_arrow=True,
                            arrow_scale=0.1)
        
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
        _,vel_1,_,_,_ = sim.get_joint_state(urdf_obj=quad_obj,
                                            joint_name="spar1_to_rotor1")
        _,vel_2,_,_,_ = sim.get_joint_state(urdf_obj=quad_obj,
                                            joint_name="spar2_to_rotor2")
        _,vel_3,_,_,_ = sim.get_joint_state(urdf_obj=quad_obj,
                                            joint_name="spar3_to_rotor3")
        _,vel_4,_,_,_ = sim.get_joint_state(urdf_obj=quad_obj,
                                            joint_name="spar4_to_rotor4")
        
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
                                arrow_scale=0.40)
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
        pos, rpy, vel, ang_vel = sim.get_base_state(quad_obj,
                                                    body_coords=True)
        pot_eng = 0.1*9.81*(pos[2]+3.)
        kin_eng = 0.5*0.1*(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2])
        pot_engs.append(pot_eng)
        kin_engs.append(kin_eng)
        sim.set_plot_data(plot_1, kin_engs, pot_engs)
        
        # Step the sim
        sim.step(real_time=True,
                 update_vis=True,
                 update_ani=True)
        
        # Collect keyboard IO for termination
        if keyboard.is_pressed("esc"):
            done = True
            