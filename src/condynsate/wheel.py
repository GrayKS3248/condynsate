"""
This modules provides a backend for the wheel example
"""

###############################################################################
#DEPENDENCIES
###############################################################################
import condynsate
from pathlib import Path


###############################################################################
#SIMULATION CLASS
###############################################################################
class Wheel_sim():
    def __init__(self,
                 visualization=True,
                 visualization_fr=30.,
                 animation=True,
                 animation_fr=15.):
        # Set the visualization and animation options
        self.visualization = visualization
        self.animation = animation
        
        # Define the target angle and some useful constants
        # before the simulation
        self.angle_tag = 3.1415926
        self.P = 0.0
        self.D = 0.0

        # Initialize and instance of the simulator
        self.sim = condynsate.Simulator(visualization=visualization,
                                        visualization_fr=visualization_fr,
                                        animation=animation,
                                        animation_fr=animation_fr)
        
        # Load urdf objects
        if visualization:
            cond_p = (Path(__file__).parents[2]).absolute().as_posix()
            wheel_path = cond_p + "/examples/wheel_vis/wheel.urdf"
            self.wheel_obj = self.sim.load_urdf(urdf_path=wheel_path,
                                                position=[0., 0., 0.],
                                                fixed=True,
                                                update_vis=True)
            target_path = cond_p + "/examples/wheel_vis/target_arrow.urdf"
            self.target_obj = self.sim.load_urdf(urdf_path=target_path,
                                                 position=[0., 0., 0.6655],
                                                 fixed=True,
                                                 update_vis=True)
        
        # If there is no animation, do not add subplots
        if not animation:
            return
        
        # Create desired plots then open the animator
        self.p1, self.a1 = self.sim.add_subplot(n_artists=2,
                                                subplot_type='line',
                                                title="Angles vs Time",
                                                x_label="Time [Seconds]",
                                                y_label="Angles [Rad]",
                                                colors=["r", "b"],
                                                line_widths=[2.5, 2.5],
                                                line_styles=["-", ":"],
                                                labels=["Angle", "Target"])
        self.p2, self.a2 = self.sim.add_subplot(n_artists=1,
                                                subplot_type='line',
                                                title="Torque vs Time",
                                                x_label="Time [Seconds]",
                                                y_label="Torque [Nm]",
                                                colors=["k"],
                                                line_widths=[2.5])
        self.p3, self.a3 = self.sim.add_subplot(n_artists=2,
                                                subplot_type='bar',
                                                title="Gains",
                                                x_label="Values [-]",
                                                y_label="Names [-]",
                                                labels=["Prop", "Derv"],
                                                colors=["m", "c"],
                                                line_widths=[1.0, 1.0],
                                                x_lim=[0.0, 10.0])
        
        # Open the animator GUI
        self.sim.open_animator_gui()

    def run(self,
            controller):
        # Run the simulation loop
        while(not self.sim.is_done):
            ##################################################################
            # SENSOR
            # Use a sensor to collect the absolute angle of the wheel and
            # calculate the error from target angle.
            state = self.sim.get_joint_state(urdf_obj=self.wheel_obj,
                                             joint_name="ground_to_axle")
            angle = state['position']
            angle_vel = state['velocity']
            
            ###################################################################
            # CONTROLLER
            # Get the torque as calculated by the controller
            torque = controller.get_inputs(target_angle=self.angle_tag,
                                           angle=angle,
                                           angle_vel=angle_vel,
                                           P=self.P,
                                           D=self.D)
            
            ###################################################################
            # ACTUATOR
            # Apply the controller calculated torque to the wheel using
            # an actuator.
            self.sim.set_joint_torque(urdf_obj=self.wheel_obj,
                                      joint_name="ground_to_axle",
                                      torque=torque,
                                      show_arrow=True,
                                      arrow_scale=0.3,
                                      arrow_offset=0.52)
            
            ###################################################################
            # UPDATE THE PLOTS
            # Plot angle and target angle vs time
            self.sim.add_subplot_point(subplot_index=self.p1,
                                       artist_index=self.a1[0],
                                       x=self.sim.time,
                                       y=angle)
            self.sim.add_subplot_point(subplot_index=self.p1,
                                       artist_index=self.a2[1],
                                       x=self.sim.time,
                                       y=self.angle_tag)
            
            # Plot torque vs time
            self.sim.add_subplot_point(subplot_index=self.p2,
                                       artist_index=self.a1[0],
                                       x=self.sim.time,
                                       y=torque)
            
            # Plot the P and D variables
            self.sim.add_subplot_point(subplot_index=self.p3,
                                       artist_index=self.a1[0],
                                       x=self.P)
            self.sim.add_subplot_point(subplot_index=self.p3,
                                       artist_index=self.a2[1],
                                       x=self.D)
            
            ###################################################################
            # UPDATE THE TARGET ANGLE
            # Iterate the target angle
            self.angle_tag = self.sim.iterate_val(curr_val=self.angle_tag,
                                                  down_key='a',
                                                  up_key='d',
                                                  iter_val=0.03,
                                                  min_val=-3.1415927,
                                                  max_val=3.1415927)
                
            # Adjust the target arrow so that it is always 
            # pointing in the target angle direction
            self.sim.set_joint_position(urdf_obj=self.target_obj,
                                        joint_name='world_to_arrow',
                                        position=self.angle_tag,
                                        physics=False)
                
            ###################################################################
            # UPDATE THE CONTROL GAINS
            # Iterate the proportional and derivative gains
            self.P = self.sim.iterate_val(curr_val=self.P,
                                          down_key='f',
                                          up_key='r',
                                          iter_val=0.02,
                                          min_val=0,
                                          max_val=10)
            self.D = self.sim.iterate_val(curr_val=self.D,
                                          down_key='g',
                                          up_key='t',
                                          iter_val=0.02,
                                          min_val=0,
                                          max_val=10)
            
            ###################################################################
            # STEP THE SIMULATION
            # Step the sim
            self.sim.step(real_time=True,
                          update_vis=self.visualization,
                          update_ani=self.animation)
            