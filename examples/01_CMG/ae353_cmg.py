"""
This modules provides a backend for the ae353 cmg example
"""

###############################################################################
#DEPENDENCIES
###############################################################################
from condynsate.simulator import Simulator
from pathlib import Path
import numpy as np


###############################################################################
#SIMULATION CLASS
###############################################################################
class CMG_sim():
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

        # Initialize and instance of the simulator
        self.sim = Simulator(keyboard=use_keyboard,
                             visualization=visualization,
                             visualization_fr=visualization_fr,
                             animation=animation,
                             animation_fr=animation_fr)
        
        # Get the path to the current directory
        path = (Path(__file__).parents[0]).absolute().as_posix()
        
        # Load the ground
        cmg_path = path + "/cmg_vis/cmg.urdf"
        self.cmg_obj = self.sim.load_urdf(urdf_path=cmg_path,
                                roll=np.pi/2,
                                yaw=3*np.pi/4.,
                                pitch=np.pi,
                                fixed=True,
                                update_vis=True)

        # If there is no animation, do not add subplots
        if not animation:
            return
        
        # Make plot for states and input
        self.p1, self.a1 = self.sim.add_subplot(n_artists=2,
                                                subplot_type='line',
                                                title="State",
                                                x_label="Time [s]",
                                                y_label="Angles [Deg]",
                                                colors=["m", "c"],
                                                line_widths=[2.5, 2.5],
                                                line_styles=["-", "-"],
                                                labels=['Frame', 'Gimbal'],
                                                y_lim=[-90.,90],
                                                h_zero_line=True)
        self.p2, self.a2 = self.sim.add_subplot(n_artists=1,
                                                subplot_type='line',
                                                title="Input",
                                                x_label="Time [s]",
                                                y_label="Torque [Nm]",
                                                colors=["k"],
                                                line_widths=[2.5],
                                                line_styles=["-"],
                                                y_lim=[-1.,1.],
                                                h_zero_line=True)
        
        # Open the animator GUI
        self.sim.open_animator_gui()


    def run(self,
            controller,
            max_time = None,
            initial_frame_angle = 0.0,
            initial_gimbal_angle = 0.0,
            initial_frame_velocity = 0.0,
            initial_gimbal_velocity = 0.0,
            rotor_velocity = 100.0,
            frame_damping = 0.1):
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
        initial_frame_angle : Float, optional
            The initial angle of the frame in radians. This is set
            when the simulation starts and when the simulation is
            reset. The default value is 0.0.
        initial_gimbal_angle : Float, optional
            The initial angle of the gimbal in radians. This is set
            when the simulation starts and when the simulation is
            reset. The default value is 0.0.
        initial_frame_velocity : Float, optional
            The initial velocity of the frame in radians/second.
            This is set when the simulation starts and when the
            simulation is reset. The default value is 0.0.
        initial_gimbal_velocity : Float, optional
            The initial velocity of the gimbal in radians/second.
            This is set when the simulation starts and when the
            simulation is reset. The default value is 0.0.
        rotor_velocity : Float, optional
            The fixed velocity of the rotor. Remember, if this is changed,
            the dynamics will also be changed. Make sure to update your 
            controller accordingly! The default value is 100.0.
        frame_damping : Float, optional
            The damping applied to the frame axle. If set to 0, no energy
            is lost. Anything greater than 0 results in energy loss while
            the frame is moving. The default value is 0.1.

        Returns
        -------
        data : Dictionary of lists
            data["frame_angle"] : List of Floats
                A list of the frame angle in radians at each time stamp during
                the simulation.
            data["gimbal_angle"] : List of Floats
                A list of the gimbal angle in radians at each time stamp during
                the simulation.
            data["frame_velocity"] : List of Floats
                A list of the frame velocity in radians/secondat each time
                stamp during the simulation.
            data["gimbal_velocity"] : List of Floats
                A list of the gimbal velocity in radians/second at each time
                stamp during the simulation.
            data["rotor_velocity"] : List of Floats
                A list of the rotor velocity in radians/secondat each time
                stamp during the simulation.
            data["torque"] : List of Floats
                A list of the applied torque in Newton-meters at each time
                stamp during the simulation.
            data["time"] : List of Floats  
                A list of the time stamps in seconds.

        """          
        # Set the initial values
        self.sim.set_joint_position(urdf_obj=self.cmg_obj,
                                joint_name='wall_to_frame_axle',
                                position=initial_frame_angle,
                                initial_cond=True,
                                physics=False)
        self.sim.set_joint_velocity(urdf_obj=self.cmg_obj,
                                joint_name='wall_to_frame_axle',
                                velocity=initial_frame_velocity,
                                initial_cond=True,
                                physics=False)
        self.sim.set_joint_position(urdf_obj=self.cmg_obj,
                                joint_name='frame_to_cage_axle',
                                position=initial_gimbal_angle,
                                initial_cond=True,
                                physics=False)
        self.sim.set_joint_velocity(urdf_obj=self.cmg_obj,
                                joint_name='frame_to_cage_axle',
                                velocity=initial_gimbal_velocity,
                                initial_cond=True,
                                physics=False)
        
        # Set the gimbal rate and pendulum damping
        self.sim.set_joint_velocity(urdf_obj=self.cmg_obj,
                                    joint_name='cage_to_wheel',
                                    velocity = rotor_velocity,
                                    physics=True)
        self.sim.set_joint_damping(urdf_obj=self.cmg_obj,
                                   joint_name='wall_to_frame_axle',
                                   damping=frame_damping)
        self.sim.set_joint_damping(urdf_obj=self.cmg_obj,
                                   joint_name='frame_to_cage_axle',
                                   damping=0.06)
        
        # Reset the controller
        controller.reset()
        
        # Create a lists to hold the simulation data
        time_history = []
        frame_angle_history =[]
        frame_velocity_history = []
        gimbal_angle_history =[]
        gimbal_velocity_history = []
        rotor_velocity_history = []
        torque_history = []

        # Await run command
        self.sim.await_keypress(key='enter')
            
        # Run the simulation loop
        while(not self.sim.is_done):
            ##################################################################
            # SENSOR
            # Get the frame angle and rate
            frame_state = self.sim.get_joint_state(urdf_obj=self.cmg_obj,
                                               joint_name="wall_to_frame_axle")
            frame_angle = frame_state['position']
            frame_velocity = frame_state['velocity']
            
            # Get the gimbal angle and rate
            gimbal_state = self.sim.get_joint_state(urdf_obj=self.cmg_obj,
                                            joint_name="frame_to_cage_axle")
            gimbal_angle = gimbal_state['position']
            gimbal_velocity = gimbal_state['velocity']
            
            # Get the rotor state
            rotor_state = self.sim.get_joint_state(urdf_obj=self.cmg_obj,
                                                   joint_name='cage_to_wheel')
            curr_rotor_velocity = rotor_state['velocity']
            
            ###################################################################
            # CONTROLLER
            # Get the torque as calculated by the controller
            sd = self.sim.is_pressed("shift+d")
            sa = self.sim.is_pressed("shift+a")
            d = self.sim.is_pressed("d")
            a = self.sim.is_pressed("a")
            inputs = controller.run(frame_angle=frame_angle,
                                    frame_velocity=frame_velocity,
                                    gimbal_angle=gimbal_angle,
                                    gimbal_velocity=gimbal_velocity,
                                    rotor_velocity=curr_rotor_velocity,
                                    time=self.sim.time,
                                    sd=sd,
                                    sa=sa,
                                    d=d,
                                    a=a)
            torque = inputs[0]
            if torque > 1.0:
                torque = 1.0
            if torque < -1.0:
                torque = -1.0
            
            ###################################################################
            # SIMULATION DATA
            # Append data to history lists
            time_history.append(self.sim.time)
            frame_angle_history.append(frame_angle)
            frame_velocity_history.append(frame_velocity)
            gimbal_angle_history.append(gimbal_angle)
            gimbal_velocity_history.append(gimbal_velocity)
            rotor_velocity_history.append(rotor_velocity)
            torque_history.append(torque)
            
            ###################################################################
            # ACTUATOR
            # Set the CMG wheel torque
            self.sim.set_joint_torque(urdf_obj=self.cmg_obj,
                                      joint_name="frame_to_cage_axle",
                                      torque=torque,
                                      show_arrow=True,
                                      arrow_scale=1.)
            
            ###################################################################
            # UPDATE THE PLOT
            if self.animation:
                self.sim.add_subplot_point(subplot_index=self.p1,
                                      artist_index=self.a1[0],
                                      x=self.sim.time,
                                      y=frame_angle*180/np.pi)
                self.sim.add_subplot_point(subplot_index=self.p1,
                                      artist_index=self.a1[1],
                                      x=self.sim.time,
                                      y=gimbal_angle*180/np.pi)
                self.sim.add_subplot_point(subplot_index=self.p2,
                                      artist_index=self.a2[0],
                                      x=self.sim.time,
                                      y=torque)
            
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
                
                # Set the rotor speed
                self.sim.set_joint_velocity(urdf_obj=self.cmg_obj,
                                            joint_name='cage_to_wheel',
                                            velocity = rotor_velocity,
                                            physics=True)
                
                # Reset the history
                time_history = []
                frame_angle_history =[]
                frame_velocity_history = []
                gimbal_angle_history =[]
                gimbal_velocity_history = []
                rotor_velocity_history = []
                torque_history = []
                
                
        # When the simulation is done running, gather all simulation data 
        # into a dictionary and return it
        data = {'time' : time_history,
                'frame_angle' : frame_angle_history,
                'gimbal_angle' : gimbal_angle_history,
                'frame_velocity' : frame_velocity_history,
                'gimbal_velocity' : gimbal_velocity_history,
                'rotor_velocity' : rotor_velocity_history,
                'torque' : torque_history}
        return data
            