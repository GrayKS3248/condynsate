"""
This modules provides a backend for the ae353 wheel example
"""

###############################################################################
#DEPENDENCIES
###############################################################################
from condynsate.simulator import Simulator
from pathlib import Path
import time


###############################################################################
#SIMULATION CLASS
###############################################################################
class Wheel_sim():
    def __init__(self,
                 use_keyboard=True,
                 visualization=True,
                 visualization_fr=20.,
                 animation=True,
                 animation_fr=10.):
        """
        Initializes an instance of the simulation class.

        Parameters
        ----------
        keyboard : bool, optional
            A boolean flag that indicates whether the simulation will allow
            the use of keyboard interactivity. The default is True.
        visualization : bool, optional
            A boolean flag that indicates whether the simulation will be 
            visualized in meshcat. The default is True.
        visualization_fr : float, optional
            The frame rate (frames per second) at which the visualizer is
            updated. The default is 20..
        animation : bool, optional
            A boolean flag that indicates whether animated plots are created
            in real time. The default is True.
        animation_fr : float, optional
            The frame rate (frames per second) at which the animated plots are
            updated. The default is 10..

        Returns
        -------
        None.

        """
        # Set the visualization and animation options
        self.visualization = visualization
        self.animation = animation
        
        # Define the target angle and some useful constants
        # before the simulation
        self.angle_tag = 0.0
        self.P = 0.0
        self.D = 0.0

        # Initialize and instance of the simulator
        self.sim = Simulator(keyboard=use_keyboard,
                             visualization=visualization,
                             visualization_fr=visualization_fr,
                             animation=animation,
                             animation_fr=animation_fr)
        
        # Load urdf objects
        if visualization:
            path = (Path(__file__).parents[0]).absolute().as_posix()
            wheel_path = path + "/wheel_vis/wheel.urdf"
            self.wheel_obj = self.sim.load_urdf(urdf_path=wheel_path,
                                                position=[0., 0., 0.],
                                                fixed=True,
                                                update_vis=True)
            target_path = path + "/wheel_vis/target_arrow.urdf"
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
            controller,
            max_time = None,
            initial_angle = 0.0,
            initial_target_angle = 3.14159,
            initial_P = 0.0,
            initial_D = 0.0):
        """
        Runs a complete simulation

        Parameters
        ----------
        controller : class
            A custom class that, at a minimum, must provide the functions
            controller.run() and controller.reset()
        max_time : Float, optional
            The total amount of time the simulation is allowed to run. When
            set to None, the simulator will run until the terminal command is 
            called. If the keyboard is disabled, users are not allowed to 
            set max time as None. The default value is None.
        initial_angle : Float, optional
            The initial angle of the wheel in radians. The wheel is set to 
            this angle when the simulation starts and when the simulation is
            reset. The default value is 0.0.
        initial_target_angle : Float, optioanl
            The initial target angle in radians. This is set when the
            simulation starts and when the simulation is reset. Must be within
            the range [-pi, pi]. The default value is pi.
        initial_P : Float, optional
            The initial value of the proportional gain. This is set when the
            simulation starts and when the simulation is reset. Must be within
            the range [0.0, 10.0]. The default value is 0.0.
        initial_D : Float, optional
            The initial value of the derivative gain. This is set when the
            simulation starts and when the simulation is reset. Must be within
            the range [0.0, 10.0]. The default value is 0.0.

        Returns
        -------
        data : Dictionary of Lists
            data["angle"] : List of Floats
                A list of all wheel angles during the simulation.
            data["target_angle"] : List of Floats
                A list of all target wheel angles during the simulation.
            data["time"] : List of Floats
                A list of the time during the simulation.
            data["P"] : List of Floats
                A list of all proportional gains set during the simulation.
            data["D"] : List of Floats
                A list of all derivative gains set during the simulation.
            data["torque"] : List of Floats
                A list of all torques applied during the simulation.

        """
        # Set the initial values
        self.sim.set_joint_position(urdf_obj = self.wheel_obj,
                                    joint_name = "ground_to_axle",
                                    position = initial_angle,
                                    physics = False,
                                    initial_cond = True)
        self.angle_tag = initial_target_angle
        self.sim.set_joint_position(urdf_obj=self.target_obj,
                                    joint_name='world_to_arrow',
                                    position=self.angle_tag,
                                    physics=False)
        self.P = initial_P
        self.D = initial_D

        # Reset the controller
        controller.reset()
        
        # Create a lists to hold the simulation data
        time_history = []
        angle_history =[]
        target_angle_history = []
        P_history = []
        D_history = []
        torque_history = []

        # Await run command
        self.sim.await_keypress(key='enter')
            
        # Run the simulation
        while( not self.sim.is_done ):
            ##################################################################
            # SENSOR
            # Use a sensor to collect the absolute angle of the wheel and
            # calculate the error from target angle.
            state = self.sim.get_joint_state(urdf_obj=self.wheel_obj,
                                             joint_name="ground_to_axle")
            angle = state['position']
            
            ###################################################################
            # CONTROLLER
            # Get the torque as calculated by the controller
            inputs = controller.run(target_angle=self.angle_tag,
                                    angle=angle,
                                    time=self.sim.time,
                                    P=self.P,
                                    D=self.D)
            torque = inputs[0]
            
            ###################################################################
            # SIMULATION DATA
            # Append data to history lists
            time_history.append(self.sim.time)
            angle_history.append(angle)
            target_angle_history.append(self.angle_tag)
            P_history.append(self.P)
            D_history.append(self.D)
            torque_history.append(torque)
            
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
                                       artist_index=self.a1[1],
                                       x=self.sim.time,
                                       y=self.angle_tag)
            
            # Plot torque vs time
            self.sim.add_subplot_point(subplot_index=self.p2,
                                       artist_index=self.a2[0],
                                       x=self.sim.time,
                                       y=torque)
            
            # Plot the P and D variables
            self.sim.add_subplot_point(subplot_index=self.p3,
                                       artist_index=self.a3[0],
                                       x=self.P)
            self.sim.add_subplot_point(subplot_index=self.p3,
                                       artist_index=self.a3[1],
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
            val = self.sim.step(real_time=True,
                                update_vis=self.visualization,
                                update_ani=self.animation,
                                max_time=max_time)
            
            # Handle resetting the controller and simulation data
            if val == 3:
                # Reset the controller
                controller.reset()
                
                # Reset the history
                time_history = []
                angle_history =[]
                target_angle_history = []
                P_history = []
                D_history = []
                torque_history = []
                
                # Reset the initial conditions
                self.angle_tag = initial_target_angle
                self.P = initial_P
                self.D = initial_D
            
        # When the simulation is done running, gather all simulation data 
        # into a dictionary and return it
        data = {'time' : time_history,
                'angle' : angle_history,
                'target_angle' : target_angle_history,
                'P' : P_history,
                'D' : D_history,
                'torque' : torque_history}
        return data
            