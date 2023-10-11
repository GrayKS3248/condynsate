###############################################################################
#DEPENDENCIES
###############################################################################

import keyboard
from matplotlib import colormaps as cmaps
import condynsate
from condynsate.utils import format_RGB


###############################################################################
#MAIN LOOP
###############################################################################
if __name__ == "__main__":
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
    
    # Variables to track applied RPM
    max_torque = 5.
    
    # Create desired plots then open the animator
    pot_engs=[]
    kin_engs=[]
    plot_1 = sim.add_plot_to_animator(title="Potention Energy vs Kinetic",
                                      x_label="Kinetic Energy [J]",
                                      y_label="Potention Energy [J]",
                                      color="r")
    sim.open_animator_gui()
    
    # Wait for user input
    sim.await_keypress(key="enter")
    
    # Run the simulation
    elapsed_time = 0
    done = False
    while(not done):
        # Collect keyboard IO data for torque 1
        if keyboard.is_pressed("a"):
            torque_1 = -max_torque
        else:
            torque_1 = 0.
        torque_1 = round(torque_1,4)
        sim.set_joint_torque(urdf_obj=quad_obj,
                            joint_name='spar1_to_rotor1',
                            torque=torque_1)
        
        # Collect keyboard IO data for torque 2
        if keyboard.is_pressed("s"):
            torque_2 = max_torque
        else:
            torque_2 = 0.
        torque_2 = round(torque_2,4)
        sim.set_joint_torque(urdf_obj=quad_obj,
                            joint_name='spar2_to_rotor2',
                            torque=torque_2)
        
        # Collect keyboard IO data for torque 3
        if keyboard.is_pressed("d"):
            torque_3 = -max_torque
        else:
            torque_3 = 0.
        torque_3 = round(torque_3,4)
        sim.set_joint_torque(urdf_obj=quad_obj,
                            joint_name='spar3_to_rotor3',
                            torque=torque_3)
        
        # Collect keyboard IO data for torque 4
        if keyboard.is_pressed("f"):
            torque_4 = max_torque
        else:
            torque_4 = 0.
        torque_4 = round(torque_4,4)
        sim.set_joint_torque(urdf_obj=quad_obj,
                            joint_name='spar4_to_rotor4',
                            torque=torque_4)
        
        # Retrieve the joint states for coloring and force
        _, rotor1_vel = sim.get_joint_state(urdf_obj=quad_obj,
                                            joint_name="spar1_to_rotor1")
        _, rotor2_vel = sim.get_joint_state(urdf_obj=quad_obj,
                                            joint_name="spar2_to_rotor2")
        _, rotor3_vel = sim.get_joint_state(urdf_obj=quad_obj,
                                            joint_name="spar3_to_rotor3")
        _, rotor4_vel = sim.get_joint_state(urdf_obj=quad_obj,
                                            joint_name="spar4_to_rotor4")

        # Color based on velocity
        rotor1_sat = abs(rotor1_vel) / 100.
        rotor1_col = cmaps['Reds'](round(255*rotor1_sat))[0:3]
        rotor1_col = format_RGB(rotor1_col,
                                range_to_255=True)
        rotor2_sat = abs(rotor2_vel) / 100.
        rotor2_col = cmaps['Reds'](round(255*rotor2_sat))[0:3]
        rotor2_col = format_RGB(rotor2_col,
                                range_to_255=True)
        rotor3_sat = abs(rotor3_vel) / 100.
        rotor3_col = cmaps['Reds'](round(255*rotor3_sat))[0:3]
        rotor3_col = format_RGB(rotor3_col,
                                range_to_255=True)
        rotor4_sat = abs(rotor4_vel) / 100.
        rotor4_col = cmaps['Reds'](round(255*rotor4_sat))[0:3]
        rotor4_col = format_RGB(rotor4_col,
                                range_to_255=True)
        sim.set_link_color(urdf_obj=quad_obj,
                           link_name='rotor1',
                           color=rotor1_col)
        sim.set_link_color(urdf_obj=quad_obj,
                           link_name='rotor2',
                           color=rotor2_col)
        sim.set_link_color(urdf_obj=quad_obj,
                           link_name='rotor3',
                           color=rotor3_col)
        sim.set_link_color(urdf_obj=quad_obj,
                           link_name='rotor4',
                           color=rotor4_col)
        
        # Set force based on velocity
        sim.apply_force_to_link(urdf_obj=quad_obj,
                                link_name='rotor1',
                                force=[0., 0.001*rotor1_vel, -0.05*rotor1_vel],
                                show_arrow=True)
        sim.apply_force_to_link(urdf_obj=quad_obj,
                                link_name='rotor2',
                                force=[0., 0.001*rotor2_vel, 0.05*rotor2_vel],
                                show_arrow=True)
        sim.apply_force_to_link(urdf_obj=quad_obj,
                                link_name='rotor3',
                                force=[0., 0.001*rotor3_vel, -0.05*rotor3_vel],
                                show_arrow=True)
        sim.apply_force_to_link(urdf_obj=quad_obj,
                                link_name='rotor4',
                                force=[0., 0.001*rotor4_vel, 0.05*rotor4_vel],
                                show_arrow=True)
        
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
            